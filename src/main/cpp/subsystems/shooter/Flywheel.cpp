// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/shooter/Flywheel.h"

using namespace ctre::phoenix6;

namespace subsystems::shooter
{

    Flywheel::Flywheel(subsystems::swerve::Drive *swerveDrive)
        : m_swerveDrive(swerveDrive),
          m_powerInterpolator(constants::auto_align::kPowerInterpolatorDistances, constants::auto_align::kPowerInterpolatorPowers),
          m_flywheelMotor(constants::shooter::flywheel::kShooterFlywheelID, rev::spark::SparkFlex::MotorType::kBrushless),
          m_flywheelFollowMotor(constants::shooter::flywheel::kShooterFlywheelFollowID, rev::spark::SparkFlex::MotorType::kBrushless),
          m_flywheelConfig(),
          m_flywheelFollowConfig(),
          m_flywheelController(m_flywheelMotor.GetClosedLoopController())
    {
        ConfigFlywheelMotor();
    }

    // This method will be called once per scheduler run
    void Flywheel::Periodic()
    {
        frc::Pose2d robotPose = m_swerveDrive->GetEstimatedPose();
        double distanceToTarget = util::align::turret::GetDistanceToTarget(robotPose).value();
        frc::SmartDashboard::PutNumber("distance to hub", distanceToTarget);
            
        frc::SmartDashboard::PutNumber("Test Bindings/flywheel current rpm", m_flywheelMotor.GetEncoder().GetVelocity());
        frc::SmartDashboard::PutNumber("Test Bindings/flywheel target rpm", GetFlywheelTargetVelocity());
        frc::SmartDashboard::PutNumber("Test bindings/flywheel m_rpm target", m_rpm);
        // frc::SmartDashboard::PutNumber("Test Bindings/flywheel current",  m_flywheelMotor.GetSupplyCurrent().GetValue().value());

        // SetFlywheelMotorVelocity(m_rpm); FOR RUNNING AT ALL TIMES 
    }

    void Flywheel::ConfigFlywheelMotor()
    {
        m_flywheelConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);

        m_flywheelConfig.signals
            .PrimaryEncoderVelocityAlwaysOn(true)
            .PrimaryEncoderVelocityPeriodMs(20)
            .AppliedOutputPeriodMs(20)
            .BusVoltagePeriodMs(100)
            .OutputCurrentPeriodMs(100);

        m_flywheelConfig
            .Inverted(constants::shooter::flywheel::kFlywheelMotorInverted)
            .ClosedLoopRampRate(0.25)
            .SetIdleMode(rev::spark::SparkFlexConfig::IdleMode::kCoast)
            .SmartCurrentLimit(80, 40)
            .closedLoop
            .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
            .P(constants::shooter::flywheel::kP)
            .I(constants::shooter::flywheel::kI)
            .D(constants::shooter::flywheel::kD)
            .OutputRange(-1, 1)
            .feedForward
            .kS(constants::shooter::flywheel::kS)
            .kV(constants::shooter::flywheel::kV);

        m_flywheelFollowConfig
            .Follow(m_flywheelMotor, constants::shooter::flywheel::kInvertSecondMotor)
            .SetIdleMode(rev::spark::SparkFlexConfig::IdleMode::kCoast)
            .SmartCurrentLimit(80, 40)
            .ClosedLoopRampRate(0.25);

        m_flywheelMotor.Configure(m_flywheelConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
        m_flywheelFollowMotor.Configure(m_flywheelFollowConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    }

    void Flywheel::SetState(constants::shooter::FlywheelState flywheelState)
    {
        m_flywheelState = flywheelState;
        frc::Pose2d robotPose = m_swerveDrive->GetEstimatedPose();
        switch (flywheelState)
        {
        case (constants::shooter::FlywheelState::AutoVelocity):
        {
            frc::Pose2d robotPose = m_swerveDrive->GetEstimatedPose();
            double distanceToTarget = util::align::turret::GetDistanceToTarget(robotPose).value();
            // double power = m_powerInterpolator.Predict(distanceToTarget);
            // frc::SmartDashboard::PutNumber("Test Bindings/flywheel target power", power);
            // m_rpm = (power * constants::shooter::flywheel::kFlywheelMaxRPM);
            
            m_rpm = constants::shooter::flywheel::kA3 * std::pow(distanceToTarget, 3) +
                    constants::shooter::flywheel::kA2 * std::pow(distanceToTarget, 2) +
                    constants::shooter::flywheel::kA1 * distanceToTarget +
                    constants::shooter::flywheel::kA0;
            break;
        }
        case (constants::shooter::FlywheelState::FixedVelocity):
        {
            m_rpm = constants::shooter::flywheel::kFlywheelShootVelocity;
            break;
        }
        case (constants::shooter::FlywheelState::ManualVelocity):
        {
            break;
        }
        case (constants::shooter::FlywheelState::ZeroVelocity):
        {
            m_rpm = 0.0;
            break;
        }
        default:
        {
            m_rpm = 0.0;
            break;
        }
        }
    }

    
    void Flywheel::SetFlywheelMotorVelocity(double RPM)
    {
        frc::SmartDashboard::PutNumber("Test bindings/Did flywheel get RPM?", RPM); 
        // Uncomment if you want to reimplement TalonFX
        m_flywheelController.SetSetpoint(RPM, rev::spark::SparkFlex::ControlType::kVelocity);
    }

    void Flywheel::SetFlywheelMotorVelocityToTarget()
    {
        m_flywheelController.SetSetpoint(m_rpm, rev::spark::SparkFlex::ControlType::kVelocity);
    }

    void Flywheel::SetFlywheelTargetRPM(double RPM)
    {
        m_rpm = RPM;
    }

    double Flywheel::GetFlywheelTargetRPM()
    {
        return m_rpm * constants::shooter::flywheel::kFlywheelMultiplier;
    }

    double Flywheel::GetFlywheelTargetVelocity()
    {
        frc::SmartDashboard::PutNumber("Maybe velocityyyyy ", m_flywheelController.GetSetpoint());
        return m_flywheelController.GetSetpoint();
    }

    void Flywheel::SetFlywheelMotorPower(double power)
    {
        frc::SmartDashboard::PutNumber("Requested Power %", power);
        m_flywheelMotor.Set(power);
    }
};