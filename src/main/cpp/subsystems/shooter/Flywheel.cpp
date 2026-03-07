// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/shooter/Flywheel.h"

using namespace ctre::phoenix6;

namespace subsystems::shooter { 
    
    Flywheel::Flywheel(subsystems::swerve::Drive *swerveDrive) 
      : 
        m_swerveDrive(swerveDrive),
        m_powerInterpolator(constants::auto_align::kPowerInterpolatorDistances, constants::auto_align::kPowerInterpolatorPowers),
        m_flywheelMotor(constants::shooter::flywheel::kShooterFlywheelID, rev::spark::SparkFlex::MotorType::kBrushless),
        m_flywheelFollowMotor(constants::shooter::flywheel::kShooterFlywheel2ID, rev::spark::SparkFlex::MotorType::kBrushless),
        m_flywheelConfig(),
        m_flywheelFollowConfig(),
        m_flywheelController(m_flywheelMotor.GetClosedLoopController())
    {
    
        ConfigFlywheelMotor();
    }
    
    // This method will be called once per scheduler run
    void Flywheel::Periodic() { 
    
        // frc::SmartDashboard::PutNumber("Power Interpolator/power", m_targetPower); 
        // frc::SmartDashboard::PutNumber("Test Bindings/flywheel target power", m_targetPower);
        // frc::SmartDashboard::PutNumber("Test Bindings/flywheel power", m_flywheelMotor.Get());
        frc::SmartDashboard::PutNumber("Test Bindings/flywheel current rpm", m_flywheelController.GetMAXMotionSetpointVelocity());
        frc::SmartDashboard::PutNumber("Test Bindings/flywheel target rpm", GetFlywheelTargetVelocity());
    
        // frc::SmartDashboard::PutNumber("Test Bindings/flywheel current",  m_flywheelMotor.GetSupplyCurrent().GetValue().value());
       
        SetFlywheelMotorVelocity(std::abs(m_rpm)); 
    }
    
    void Flywheel::ConfigFlywheelMotor() {
        m_flywheelConfig
            .Inverted(constants::shooter::flywheel::kFlywheelMotorInverted)
            .OpenLoopRampRate(1.0)
            .SetIdleMode(rev::spark::SparkFlexConfig::IdleMode::kCoast)
            .closedLoop
            .P(constants::shooter::flywheel::kP)
            .I(constants::shooter::flywheel::kI)
            .D(constants::shooter::flywheel::kD)
            .feedForward
            .kS(constants::shooter::flywheel::kS)
            .kV(constants::shooter::flywheel::kV);

        m_flywheelFollowConfig
            .Follow(m_flywheelMotor, constants::shooter::flywheel::kInvertSecondMotor)
            .SetIdleMode(rev::spark::SparkFlexConfig::IdleMode::kCoast)
            .OpenLoopRampRate(1.0);

        m_flywheelMotor.Configure(m_flywheelConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
        m_flywheelFollowMotor.Configure(m_flywheelFollowConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    }

    void Flywheel::SetState(constants::shooter::FlywheelState flywheelState) {
        m_flywheelState = flywheelState;
        frc::Pose2d robotPose = m_swerveDrive->GetEstimatedPose(); 
        switch (flywheelState) {
            case (constants::shooter::FlywheelState::AutoVelocity) :
            {
                frc::Pose2d robotPose = m_swerveDrive->GetEstimatedPose(); 
                double distanceToTarget = util::align::turret::GetDistanceToTarget(robotPose).value(); 
                double power = m_powerInterpolator.Predict(distanceToTarget); 
                frc::SmartDashboard::PutNumber("Test Bindings/flywheel target power", power); 
                m_rpm = (power * constants::shooter::flywheel::kFlywheelMaxRPM); 
                break;
            }
            case (constants::shooter::FlywheelState::FixedVelocity) :
            {
                m_rpm = constants::shooter::flywheel::kFlywheelShootVelocity;
                break;
            } 
            case (constants::shooter::FlywheelState::ManualVelocity) : 
            {
                break; 
            }
            case (constants::shooter::FlywheelState::ZeroVelocity) : 
            {
                m_rpm = 0.0;
                break;
            }
            default: {
                m_rpm = 0.0; 
                break; 
            }
        }
        
    }
    
    
    void Flywheel::SetFlywheelMotorVelocity(double RPM) {
        // Uncomment if you want to reimplement TalonFX
        // controls::VelocityVoltage m_request = controls::VelocityVoltage{0_tps}.WithSlot(0);
        m_flywheelController.SetSetpoint(RPM, rev::spark::SparkBase::ControlType::kMAXMotionVelocityControl); // TODO: make it a constant
    }
    
    double Flywheel::GetFlywheelTargetVelocity() {
        return m_flywheelController.GetMAXMotionSetpointVelocity(); 
    }
    
    void Flywheel::SetFlywheelMotorPower(double power) {
        m_flywheelMotor.Set(power);
    } 
};