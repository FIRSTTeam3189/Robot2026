// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterLoader.h"
#include "utilCommands/SetShooterLoaderPower.h"


Loader::Loader() :
    m_ShooterLoaderMotor(ShooterConstants::kShooterLoaderID, "swerve"),

    m_shooterLoaderConfig(){
        ConfigLoaderMotor();
    }

void Loader::ConfigLoaderMotor() {
    m_ShooterLoaderMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_shooterLoaderConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
    m_shooterLoaderConfig.Feedback.FeedbackRotorOffset = ShooterConstants::kExtensionOffset;

    m_shooterLoaderConfig.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants::kExtensionContinuousCurrentLimit;
    m_shooterLoaderConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants::kExtensionPeakCurrentLimit;
    m_shooterLoaderConfig.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants::kExtensionPeakCurrentDuration;
    m_shooterLoaderConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants::kExtensionEnableCurrentLimit;

    m_shooterLoaderConfig.MotorOutput.Inverted = ShooterConstants::kExtensionMotorInverted;
    m_shooterLoaderConfig.MotorOutput.NeutralMode = ShooterConstants::kExtensionNeutralMode;

    m_ShooterLoaderMotor.GetConfigurator().Apply(m_shooterLoaderConfig);
    m_ShooterLoaderMotor.SetPosition(0.0_tr);
}

void Loader::SetShooterShooterLoaderMotorPower(double power) {
    m_ShooterLoaderMotor.Set(power);
}

// This method will be called once per scheduler run
void Loader::Periodic() {}
