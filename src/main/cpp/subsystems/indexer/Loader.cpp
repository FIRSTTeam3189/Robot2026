// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of

// the WPILib BSD license file in the root directory of this project.


#include "subsystems/indexer/Loader.h"

namespace subsystems::indexer {
    
    Loader::Loader() :
        m_loaderMotor(constants::indexer::loader::kLoaderID, rev::spark::SparkMax::MotorType::kBrushless),
        m_loaderConfig()
    {
        ConfigLoaderMotor();
    }
    
    
    void Loader::ConfigLoaderMotor() {
    
        m_loaderConfig
            .Inverted(constants::indexer::loader::kLoaderMotorInverted)
            .SmartCurrentLimit(constants::indexer::loader::kLoaderCurrentLimit)
            .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
            
        // // Set to factory default
        m_loaderMotor.Configure(m_loaderConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
        // frc::SmartDashboard::PutBoolean("configurerd loaderrr motor", true); 
    }
    
    void Loader::SetLoaderMotorPower(double power) {
        frc::SmartDashboard::PutNumber("Test Bindings/LoaderPower", power); 
        m_loaderMotor.Set(power);
    }
    
    // This method will be called once per scheduler run
    void Loader::Periodic() {
        // frc::SmartDashboard::PutBoolean(m_loaderConfig.)
    
    }
}

