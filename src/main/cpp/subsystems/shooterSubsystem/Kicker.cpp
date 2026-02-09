// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/shooterSubsystem/Kicker.h"



Kicker::Kicker() :
    m_kickerMotor(KickerConstants::kShooterKickerID),

    m_kickerConfig(){
        ConfigKickerMotor();
    }

void Kicker::ConfigKickerMotor() {
    m_kickerMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_kickerConfig.MotorOutput.Inverted = KickerConstants::kKickerMotorInverted;
    m_kickerConfig.MotorOutput.NeutralMode = KickerConstants::kKickerNeutralMode;

    m_kickerMotor.GetConfigurator().Apply(m_kickerConfig);
}

void Kicker::SetKickerMotorPower(double power) {
    frc::SmartDashboard::PutNumber("Test Bindings/kickerPower", power); 
    m_kickerMotor.Set(power);
}


// This method will be called once per scheduler run
void Kicker::Periodic() {}
