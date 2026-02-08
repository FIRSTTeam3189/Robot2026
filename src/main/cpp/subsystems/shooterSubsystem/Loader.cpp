// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/shooterSubsystem/Loader.h"



Loader::Loader() :
    m_loaderMotor(LoaderConstants::kShooterLoaderID),

    m_loaderConfig(){
        ConfigLoaderMotor();
    }

void Loader::ConfigLoaderMotor() {
    m_loaderMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_loaderConfig.MotorOutput.Inverted = LoaderConstants::kLoaderMotorInverted;
    m_loaderConfig.MotorOutput.NeutralMode = LoaderConstants::kLoaderNeutralMode;

    m_loaderMotor.GetConfigurator().Apply(m_loaderConfig);
}

void Loader::SetLoaderMotorPower(double power) {
    frc::SmartDashboard::PutNumber("Test Bindings/loaderPower", power); 
    m_loaderMotor.Set(power);
}


// This method will be called once per scheduler run
void Loader::Periodic() {}
