// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterLoader.h"



Loader::Loader() :
    m_ShooterLoaderMotor(ShooterConstants::kShooterLoaderID, "swerve"),

    m_shooterLoaderConfig(){
        ConfigLoaderMotor();
    }

void Loader::ConfigLoaderMotor() {
    m_ShooterLoaderMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_shooterLoaderConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
    m_shooterLoaderConfig.Feedback.FeedbackRotorOffset = ShooterConstants::kRotationOffset;

    m_shooterLoaderConfig.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants::kRotationContinuousCurrentLimit;
    m_shooterLoaderConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants::kRotationPeakCurrentLimit;
    m_shooterLoaderConfig.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants::kRotationPeakCurrentDuration;
    m_shooterLoaderConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants::kRotationEnableCurrentLimit;

    m_shooterLoaderConfig.MotorOutput.Inverted = ShooterConstants::kRotationMotorInverted;
    m_shooterLoaderConfig.MotorOutput.NeutralMode = ShooterConstants::kRotationNeutralMode;

    m_ShooterLoaderMotor.GetConfigurator().Apply(m_shooterLoaderConfig);
    m_ShooterLoaderMotor.SetPosition(0.0_tr);
}

void Loader::SetShooterLoaderMotorPower(double power) {
    m_ShooterLoaderMotor.Set(power);
}

// This method will be called once per scheduler run
void Loader::Periodic() {}
