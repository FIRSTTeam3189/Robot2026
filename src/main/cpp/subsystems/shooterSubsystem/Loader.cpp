// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/shooterSubsystem/Loader.h"



Loader::Loader() :
    m_ShooterLoaderMotor(LoaderConstants::kShooterLoaderID, "swerve"),

    m_shooterLoaderConfig(){
        ConfigLoaderMotor();
    }

void Loader::ConfigLoaderMotor() {
    m_ShooterLoaderMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_shooterLoaderConfig.MotorOutput.Inverted = LoaderConstants::kLoaderMotorInverted;
    m_shooterLoaderConfig.MotorOutput.NeutralMode = LoaderConstants::kLoaderNeutralMode;

    m_ShooterLoaderMotor.GetConfigurator().Apply(m_shooterLoaderConfig);
    m_ShooterLoaderMotor.SetPosition(0.0_tr);
}

void Loader::SetShooterLoaderMotorPower(double power) {
    m_ShooterLoaderMotor.Set(power);
}

// This method will be called once per scheduler run
void Loader::Periodic() {}
