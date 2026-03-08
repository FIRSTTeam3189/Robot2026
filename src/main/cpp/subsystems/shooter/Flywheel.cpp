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
        m_flywheelMotor(constants::shooter::flywheel::kShooterFlywheelID, "Swerve"),
        m_flywheelConfig()
    {
    
        ConfigFlywheelMotor();
    }
    
    // This method will be called once per scheduler run
    void Flywheel::Periodic() { 
        double distanceToTarget = util::align::turret::GetDistanceToTarget(m_swerveDrive->GetEstimatedPose()).value(); 
        frc::SmartDashboard::PutNumber("Test Bindings/distance to target", distanceToTarget);

        // frc::SmartDashboard::PutNumber("Power Interpolator/power", m_targetPower); 
        // frc::SmartDashboard::PutNumber("Test Bindings/flywheel target power", m_targetPower);
        // frc::SmartDashboard::PutNumber("Test Bindings/flywheel power", m_flywheelMotor.Get());
        frc::SmartDashboard::PutNumber("Test Bindings/flywheel current rpm", m_flywheelMotor.GetVelocity().GetValue().value() * 60);
        frc::SmartDashboard::PutNumber("Test Bindings/flywheel target rpm", GetFlywheelTargetVelocity().value() * 60);
    
        // frc::SmartDashboard::PutNumber("Test Bindings/flywheel current",  m_flywheelMotor.GetSupplyCurrent().GetValue().value());
       
        SetFlyWheelMotorVelocity(units::turns_per_second_t{std::abs(m_rps)}); 
    }
    
    void Flywheel::ConfigFlywheelMotor() {
        m_flywheelConfig.MotorOutput.Inverted = constants::shooter::flywheel::kFlywheelMotorInverted;
        m_flywheelConfig.MotorOutput.NeutralMode = constants::shooter::flywheel::kFlywheelNeutralMode;
        m_flywheelConfig.OpenLoopRamps.WithDutyCycleOpenLoopRampPeriod(1.0_s);
    
        auto& slot0Configs = m_flywheelConfig.Slot0;
        slot0Configs.kS = constants::shooter::flywheel::kS;
        slot0Configs.kV = constants::shooter::flywheel::kV;
        slot0Configs.kI = constants::shooter::flywheel::kI; 
        slot0Configs.kD = constants::shooter::flywheel::kD;
    
        m_flywheelMotor.GetConfigurator().Apply(m_flywheelConfig);  
    }

    void Flywheel::SetState(constants::shooter::FlywheelState flywheelState) {
        m_flywheelState = flywheelState;
        frc::Pose2d robotPose = m_swerveDrive->GetEstimatedPose(); 
        switch (flywheelState) {
            case (constants::shooter::FlywheelState::AutoVelocity) :
            {
                frc::Pose2d robotPose = m_swerveDrive->GetEstimatedPose(); 
                double distanceToTarget = util::align::turret::GetDistanceToTarget(robotPose).value(); 
                // double power = m_powerInterpolator.Predict(distanceToTarget); 
                // double rpm = 18.85663215*std::pow(distanceToTarget,2) + 46.37148976*distanceToTarget + 2244.7636984;
                double rpm = constants::shooter::flywheel::kA4*std::pow(distanceToTarget,4) +
                             constants::shooter::flywheel::kA3*std::pow(distanceToTarget,3) + 
                             constants::shooter::flywheel::kA2*std::pow(distanceToTarget,2) + 
                             constants::shooter::flywheel::kA1*distanceToTarget + 
                             constants::shooter::flywheel::kA0;
                frc::SmartDashboard::PutNumber("Test Bindings/flywheel target cubic rpm", rpm); 
                // m_rps = (power * constants::shooter::flywheel::kFlywheelMaxRPM) / constants::shooter::flywheel::kMotorConversionRPMToRPS; 
                m_rps = rpm / constants::shooter::flywheel::kMotorConversionRPMToRPS; 
                break;
            }
            case (constants::shooter::FlywheelState::FixedVelocity) :
            {
                m_rps = constants::shooter::flywheel::kFlywheelShootVelocity.value();
                break;
            } 
            case (constants::shooter::FlywheelState::ManualVelocity) : 
            {
                break; 
            }
            case (constants::shooter::FlywheelState::ZeroVelocity) : 
            {
                m_rps = 0.0;
                break;
            }
            default: {
                m_rps = 0.0; 
                break; 
            }
        }
        
    }
    
    
    void Flywheel::SetFlyWheelMotorVelocity(units::turns_per_second_t turnsPerSecond) {
        controls::VelocityVoltage m_request = controls::VelocityVoltage{0_tps}.WithSlot(0);
        m_flywheelMotor.SetControl(m_request.WithVelocity(turnsPerSecond).WithFeedForward(constants::shooter::flywheel::kFeedForwardVoltage).WithEnableFOC(true)); // TODO: make it a constant
    }
    
    units::turns_per_second_t Flywheel::GetFlywheelTargetVelocity() {
        return units::turns_per_second_t{m_flywheelMotor.GetClosedLoopReference().GetValue()}; 
    }
    

    double Flywheel::GetFlywheelTargetRPS() {
        return m_rps; 
    }

    void Flywheel::SetFlywheelTargetRPS(double target) {
        frc::SmartDashboard::PutNumber("flywheel should be", target); 
        m_rps = target; 
    }
    
    void Flywheel::SetFlywheelMotorPower(double power) {
        m_flywheelMotor.Set(power);
    } 
};