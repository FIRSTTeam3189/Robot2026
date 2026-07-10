#include <units/angular_acceleration.h>

#include "simulation/drive/SwerveModuleSimulation.h"
#include "simulation/CommonMath.h"
#include "constants/simulation/SimulatedArena.h"

#include <iostream>


namespace simulation::drive {
    SwerveModuleSimulation::SwerveModuleSimulation(const Config& config)
    : m_config(config) 
    {
        const frc::DCMotor& driveMotor = m_config.driveMotorModel;
        m_driveMotorWindingResistance =  
            driveMotor.nominalVoltage 
            / driveMotor.stallCurrent;

        m_driveMotorKv = 
            driveMotor.freeSpeed
            / (driveMotor.nominalVoltage 
            - m_driveMotorWindingResistance 
            * driveMotor.freeCurrent);

        const frc::DCMotor& angleMotor = m_config.angleMotorModel;
        m_angleMotorWindingResistance = 
                angleMotor.nominalVoltage 
                / angleMotor.stallCurrent;

        m_angleMotorKv = 
            angleMotor.freeSpeed
            / (angleMotor.nominalVoltage 
            - m_angleMotorWindingResistance
            * angleMotor.freeCurrent);
    }

    void SwerveModuleSimulation::Stop() {
        m_driveAppliedVoltage = 0_V;
        m_angleAppliedVoltage = 0_V;
    }

    
    /**
     * Runs the passed in swerve state (target speed + target angle), 
     * calculates the necessary voltages and applies them to the motors
     */
    void SwerveModuleSimulation::RunSwerveState(const frc::SwerveModuleState& state) {
        // Projecting = scaling the commanded speed down by how misaligned the wheel 
        // currently is from the target angle. If we command full speed while the wheel is
        // pointing in the wrong direction to where it should go (target angle)
        // it would go way in the wrong direction!!!
        // projectedSpeed = desiredSpeed * cos(angleError)

        units::meters_per_second_t projectedSpeed = 
            simulation::commonmath::ProjectSwerveState(state, m_angleRotation);

        // Open loop feedforward for drive. Calculate what voltage
        // would make the motor naturally settle at target speed
        // appliedVoltage = motorVelocity / Kv
        units::radians_per_second_t driveMotorVelocitySetpoint{
            ( projectedSpeed / m_config.wheelRadius ).value() * m_config.driveGearRatio
        };

        m_driveAppliedVoltage = std::clamp(
            units::volt_t{ driveMotorVelocitySetpoint / m_driveMotorKv },
            -12_V, 12_V
        );

        units::radian_t angleError{ (state.angle - m_angleRotation).Radians() };
        m_angleAppliedVoltage = std::clamp(
            units::volt_t{
                m_config.angleKp * angleError.value()
                - m_config.angleKd * m_angleMotorVelocity.value()
            },
            -12_V, 
            12_V
        );
    }
    

    b2Vec2 SwerveModuleSimulation::ComputeModuleForce(b2Vec2 bodyLinearVelocity, frc::Rotation2d bodyRotation, units::newton_t gravityForce) {
        const frc::DCMotor& driveMotor = m_config.driveMotorModel;

        // Wheel direction in world space = robot-relative wheel angle + body rotation
        frc::Rotation2d worldModuleAngle = m_angleRotation + bodyRotation;

        units::radian_t worldAngleRad = worldModuleAngle.Radians();
        b2Vec2 wheelDirection{
            static_cast<float>(units::math::cos(worldAngleRad)),
            static_cast<float>(units::math::sin(worldAngleRad))
        };

        units::meters_per_second_t wheelGroundSpeed{ b2Dot(bodyLinearVelocity, wheelDirection) };
        units::radians_per_second_t effectiveDriveMotorVelocity{
            (wheelGroundSpeed / m_config.wheelRadius).value() * m_config.driveGearRatio
        };

        // Same Ohm's-law current calc used for the angle motor
        units::ampere_t statorCurrent = CalculateStatorCurrent(
            m_driveAppliedVoltage,
            effectiveDriveMotorVelocity,
            m_driveMotorWindingResistance,
            m_driveMotorKv
        );

        units::newton_meter_t driveTorque{
            driveMotor.Torque(statorCurrent) * m_config.driveGearRatio
        };

        units::newton_t propellingForce{ driveTorque / m_config.wheelRadius };

        // Don't let the wheel exert more force than the tire can actually grip
        units::newton_t maxGrip = GetGrippingForce(gravityForce);
        units::newton_t clampedForce = units::math::min(
            units::math::abs(propellingForce), maxGrip
        );

        propellingForce = units::math::copysign(clampedForce, propellingForce);

        return simulation::commonmath::PolarToCartesian(
            propellingForce,
            worldModuleAngle.Radians()
        );
    }


    void SwerveModuleSimulation::SimulationSubTick() {
        using namespace constants::simulation;
        const frc::DCMotor& angleMotor = m_config.angleMotorModel;

        auto angleKt{ angleMotor.stallTorque / angleMotor.stallCurrent };

        // Get current from angle motor applied voltage
        // Ohm's law: current = voltage / resistance
        units::ampere_t statorCurrent = CalculateStatorCurrent(
            m_angleAppliedVoltage,
            m_angleMotorVelocity,
            m_angleMotorWindingResistance,
            m_angleMotorKv
        );
       
        units::newton_meter_t electricTorque{ angleKt * statorCurrent * m_config.angleGearRatio };
        units::newton_meter_t frictionTorque{ 
            angleKt * (m_config.angleFrictionVoltage / m_angleMotorWindingResistance)
        };
        units::newton_meter_t signedFriction{ 
            units::math::copysign(frictionTorque, -m_angleMotorVelocity )
        };

        // Integrate acceleration into velocity

        units::radians_per_second_squared_t angularAcceleration{
            (electricTorque.value() + signedFriction.value()) / m_config.angleMOI.value()
        };

        units::radians_per_second_t newVelocity{
            m_angleMotorVelocity + angularAcceleration * simulated_arena::kSubDeltaTime
        };

        bool frictionReversesMotion = (newVelocity.value() * m_angleMotorVelocity.value()) < 0.0;
        if (frictionReversesMotion) newVelocity = 0_rad_per_s;
        m_angleMotorVelocity = newVelocity;

        m_angleRotation = m_angleRotation + frc::Rotation2d{ 
            m_angleMotorVelocity * simulated_arena::kSubDeltaTime 
        };
    }

    
    frc::SwerveModuleState SwerveModuleSimulation::GetState() const {
        return frc::SwerveModuleState{
            .speed = GetDriveSpeed(),
            .angle = m_angleRotation
        };
    }


    frc::SwerveModuleState SwerveModuleSimulation::GetEquilibriumState() const {
        units::radians_per_second_t equilibriumMotorVelocity{
            m_driveMotorKv * m_driveAppliedVoltage
        };

        units::meters_per_second_t equilibriumWheelSpeed{
            (equilibriumMotorVelocity / m_config.driveGearRatio).value() * m_config.wheelRadius.value()
        };

        return frc::SwerveModuleState{ 
            .speed = equilibriumWheelSpeed, 
            .angle = m_angleRotation 
        };
    }


    frc::Rotation2d SwerveModuleSimulation::GetAngleMotorAbsoluteFacing() const {
        return m_angleRotation;
    }


    const SwerveModuleSimulation::Config& SwerveModuleSimulation::GetConfig() const {
        return m_config;
    }


    frc::SwerveModulePosition SwerveModuleSimulation::GetPosition() const {
        units::meter_t wheelDistance{
            (m_driveMotorPosition / m_config.driveGearRatio).value() * m_config.wheelRadius.value()
        };

        return frc::SwerveModulePosition{
            .distance = wheelDistance,
            .angle = m_angleRotation
        };
    }


    /**
     * Returns the maximum friction a tire can generate before it starts to skid 
     */
    units::newton_t SwerveModuleSimulation::GetGrippingForce(units::newton_t gravityForce) const {
        return gravityForce * m_config.wheelCOF;
    }


    units::meters_per_second_t SwerveModuleSimulation::GetDriveSpeed() const {
        units::radians_per_second_t shaftAngularVelocity{ m_driveMotorVelocity / m_config.driveGearRatio };
        units::meters_per_second_t driveSpeed{ shaftAngularVelocity.value() * m_config.wheelRadius.value() };

        return driveSpeed;
    }


    units::meters_per_second_t SwerveModuleSimulation::GetMaxGroundSpeed() const {
        const frc::DCMotor& driveMotor = m_config.driveMotorModel;
        units::radians_per_second_t equilibriumMechanismVelocity {
            driveMotor.freeSpeed / m_config.driveGearRatio
        };

        return units::meters_per_second_t{
            equilibriumMechanismVelocity.value()
            * m_config.wheelRadius.value()
        };
    }


    /**
     * Obtains the theoretical maximum propelling force of one module.a
     * Calculates in respect to the gripping force & drive motor's torque
     * under its current limit.
     */
    units::newton_t SwerveModuleSimulation::GetMaxPropellingForce(
        units::kilogram_t robotMass,
        size_t moduleCount,
        units::ampere_t statorCurrentLimit,
        units::newton_t gravityForcePerModule
    ) const {
        const frc::DCMotor& driveMotor = m_config.driveMotorModel;

        units::newton_meter_t maxDriveTorque{
            driveMotor.Torque(statorCurrentLimit) * m_config.driveGearRatio
        };

        units::newton_t maxThrust{
            maxDriveTorque / m_config.wheelRadius
        };

        return units::math::min(maxThrust, gravityForcePerModule);
    }


    units::ampere_t SwerveModuleSimulation::CalculateStatorCurrent(
        units::volt_t appliedVoltage,
        units::radians_per_second_t motorVelocity,
        units::ohm_t windingResistance,
        frc::DCMotor::radians_per_second_per_volt_t Kv
    ) const {
        units::volt_t backEMF{ motorVelocity / Kv };
        return (appliedVoltage - backEMF) / windingResistance;
    }

    units::radians_per_second_t SwerveModuleSimulation::CalculateDriveMechanismVelocity(
            units::ampere_t current, 
            units::volt_t voltage
    ) const {
        frc::DCMotor motor = m_config.driveMotorModel;
        return motor.Speed(motor.Torque(current), voltage)
            / m_config.driveGearRatio;
    }

    units::ampere_t SwerveModuleSimulation::CalculateDriveCurrent(units::newton_meter_t torque) const {
        frc::DCMotor motor = m_config.driveMotorModel;
        return motor.Current(torque / m_config.driveGearRatio);
    }
}