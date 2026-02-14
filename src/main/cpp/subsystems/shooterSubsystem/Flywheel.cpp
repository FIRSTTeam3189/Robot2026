// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/shooterSubsystem/Flywheel.h"

Flywheel::Flywheel(Articulation *articulation) 
  : 
    m_articulation(articulation),
    m_powerInterpolator(AutoAlignConstants::kPowerInterpolatorAngles, AutoAlignConstants::kPowerInterpolatorPowers),
    m_flywheelMotor(FlywheelConstants::kShooterFlywheelID, "Swerve"),
    m_flywheelConfig(),
    m_targetPower()
{

    ConfigFlywheelMotor();
}

// This method will be called once per scheduler run
void Flywheel::Periodic() {

    // double hoodAngle = m_articulation->GetTargetAngle().value(); 
    // m_targetPower = m_powerInterpolator.Predict(hoodAngle); 

    // frc::SmartDashboard::PutNumber("Power Interpolator/power", m_targetPower); 
    frc::SmartDashboard::PutNumber("Test Bindings/flywheel target power", m_targetPower);
    
}

void Flywheel::ConfigFlywheelMotor() {
    m_flywheelConfig.MotorOutput.Inverted = FlywheelConstants::kFlywheelMotorInverted;
    m_flywheelConfig.MotorOutput.NeutralMode = FlywheelConstants::kFlywheelNeutralMode;
    m_flywheelConfig.OpenLoopRamps.WithDutyCycleOpenLoopRampPeriod(1.0_s);

    m_flywheelMotor.GetConfigurator().Apply(m_flywheelConfig);
}

void Flywheel::SetFlyWheelTargetPower(double power) {
    m_targetPower = power; 
}

double Flywheel::GetFlywheelTargetPower() {
    return m_targetPower; 
}

// void Flywheel::SetFlywheelMotorPower(double power) {
//     frc::SmartDashboard::PutNumber("Test Bindings/flywheel current power", power); 
//     m_flywheelMotor.Set(power);
//     m_targetPower = power; 
// }

void Flywheel::SetFlywheelMotorPower(double power) {
    frc::SmartDashboard::PutNumber("Test Bindings/flywheel current power", power); 
    // int numRampUpPoints = 500;
    // double stepSize = power / numRampUpPoints;
    // double appliedPower = 0.0;

    // for (int i = 0; i< 500; i++){
    //     m_flywheelMotor.Set(appliedPower);
    //     appliedPower += stepSize;
    // }

    // m_targetPower = power; 

    m_flywheelMotor.Set(power);
}

void Flywheel::SetFlywheelMotorPower() {
    SetFlywheelMotorPower(m_targetPower); 
}