// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/shooterSubsystem/Flywheel.h"

Flywheel::Flywheel() 
  : 
    m_flywheelMotor(FlywheelConstants::kShooterFlywheelID, "Swerve"),
    m_flywheelConfig(),
    m_targetPower()
{}

// This method will be called once per scheduler run
void Flywheel::Periodic() {

    frc::SmartDashboard::PutNumber("Test Bindings/flywheel target power", m_targetPower); 

}


void Flywheel::ConfigFlywheelMotor() {
    m_flywheelConfig.MotorOutput.Inverted = FlywheelConstants::kFlywheelMotorInverted;
    m_flywheelConfig.MotorOutput.NeutralMode = FlywheelConstants::kFlywheelNeutralMode;

    m_flywheelMotor.GetConfigurator().Apply(m_flywheelConfig);
}

void Flywheel::SetFlyWheelTargetPower(double power) {
    m_targetPower = power; 
}

double Flywheel::GetFlywheelTargetPower() {
    return m_targetPower; 
}

void Flywheel::SetFlywheelMotorPower(double power) {
    frc::SmartDashboard::PutNumber("Test Bindings/flywheel current power", m_targetPower); 
    m_flywheelMotor.Set(power);
}

void Flywheel::SetFlywheelMotorPower() {
    SetFlywheelMotorPower(m_targetPower); 
}