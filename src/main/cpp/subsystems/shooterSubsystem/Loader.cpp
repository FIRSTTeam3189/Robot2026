// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of

// the WPILib BSD license file in the root directory of this project.



#include "subsystems/shooterSubsystem/Loader.h"



Loader::Loader() :
    m_loaderMotor(LoaderConstants::kLoaderID, rev::spark::SparkMax::MotorType::kBrushless),
    m_loaderConfig()
{
    ConfigLoaderMotor();
}


void Loader::ConfigLoaderMotor() {
    rev::spark::SparkBaseConfig defaultConfig = rev::spark::SparkBaseConfig{};
    m_loaderMotor.Configure(defaultConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_loaderConfig
        .Inverted(LoaderConstants::kLoaderMotorInverted)
        .SmartCurrentLimit(LoaderConstants::kLoaderCurrentLimit)
        .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
        
    // Set to factory default
    m_loaderMotor.Configure(m_loaderConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void Loader::SetLoaderMotorPower(double power) {
    frc::SmartDashboard::PutNumber("Test Bindings/LoaderPower", power); 
    m_loaderMotor.Set(power);
}

// This method will be called once per scheduler run
void Loader::Periodic() {}