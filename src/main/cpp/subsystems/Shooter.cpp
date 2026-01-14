// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

Shooter::Shooter()
  : m_shooterTurretMotor(ShooterConstants::kShooterRotationID, "Swerve"),
    m_shooterFlywheelMotor(ShooterConstants::kShooterFlywheelID, "Swerve"),
    m_shooterConfig(),
    m_flywheelConfig()

    // drive motor (loader), fkywheel, turretrotationmotor
{
    ConfigShooter();
}

void Shooter::ConfigShooter() {
    m_shooterTurretMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_shooterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = ShooterConstants::kSoftLimitEnabledForward;
    m_shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = ShooterConstants::kSoftLimitEnabledReverse;
    m_shooterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ShooterConstants::kTopSoftLimit;
    m_shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ShooterConstants::kBottomSoftLimit;

    m_shooterConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
    m_shooterConfig.Feedback.FeedbackRotorOffset = ShooterConstants::kExtensionOffset;

    m_shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants::kExtensionContinuousCurrentLimit;
    m_shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants::kExtensionPeakCurrentLimit;
    m_shooterConfig.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants::kExtensionPeakCurrentDuration;
    m_shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants::kExtensionEnableCurrentLimit;

    m_shooterConfig.MotorOutput.Inverted = ShooterConstants::kExtensionMotorInverted;
    m_shooterConfig.MotorOutput.NeutralMode = ShooterConstants::kExtensionNeutralMode;

    m_flywheelConfig.MotorOutput.Inverted = ShooterConstants::kExtensionMotorInverted;
    m_flywheelConfig.MotorOutput.NeutralMode = ShooterConstants::kExtensionNeutralMode;
    
    m_shooterTurretMotor.GetConfigurator().Apply(m_shooterConfig);
    m_shooterFlywheelMotor.GetConfigurator().Apply(m_flywheelConfig);
    // m_shooterTurretMotor.SetPosition(0.0_tr);

  
}




void Shooter::OverrideSoftLimits(bool isEnabled) {
    m_shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = isEnabled;
    frc::SmartDashboard::PutBoolean("Shooter soft limit enabled: ", isEnabled);
    m_shooterTurretMotor.GetConfigurator().Apply(m_shooterConfig);
}



void Shooter::SetShooterRotationMotorPower(double power) {
    m_shooterTurretMotor.Set(power);
}

void Shooter::SetShooterFlywheelMotorPower(double power) {
    m_shooterFlywheelMotor.Set(power);
}



void Shooter::SetRotationBrakeMode(Shooter::BrakeMode mode) {
    switch (mode) {
        case Shooter::BrakeMode::Brake:
            m_shooterConfig.MotorOutput.NeutralMode =
                ctre::phoenix6::signals::NeutralModeValue::Brake;
            break;

        case Shooter::BrakeMode::Coast:
            m_shooterConfig.MotorOutput.NeutralMode =
                ctre::phoenix6::signals::NeutralModeValue::Coast;
            break;

        case Shooter::BrakeMode::Default:
            m_shooterConfig.MotorOutput.NeutralMode =
                ShooterConstants::kExtensionNeutralMode;
            break;
    }

    m_shooterTurretMotor.GetConfigurator().Apply(m_shooterConfig);
}

// This method will be called once per scheduler run
void Shooter::Periodic() {}
