// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/utilCommands/SetFlywheelMotorPower.h"

SetFlywheelMotorPower::SetFlywheelMotorPower(Flywheel *flywheel, double power)
  : m_flywheel(flywheel),
    m_power(power)
{
    AddRequirements(flywheel);
}

void SetFlywheelMotorPower::Execute() {
    m_flywheel->SetShooterFlywheelMotorPower(m_power);
}

void SetFlywheelMotorPower::End(bool interrupted) {
    m_flywheel->SetShooterFlywheelMotorPower(0.0);
}

bool SetFlywheelMotorPower::IsFinished() {
    return false;
}
