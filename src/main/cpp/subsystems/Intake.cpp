// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

// This method will be called once per scheduler run
void Intake::Periodic() {}

Intake::Intake(): 
    m_rollerMotor(IntakeConstants::kIntakeMotorID, "Swerve"), 
    m_rollerConfig() 
{
    ConfigRollerMotor(); 
}


void Intake::ConfigRollerMotor() {
    m_rollerMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_rollerConfig.MotorOutput.Inverted = IntakeConstants::kExtensionMotorInverted;
    m_rollerConfig.MotorOutput.NeutralMode = IntakeConstants::kExtensionNeutralMode;

    m_rollerMotor.GetConfigurator().Apply(m_rollerConfig);
}

void Intake::SetRollerPower(double power) {
    m_rollerMotor.Set(power); 
}