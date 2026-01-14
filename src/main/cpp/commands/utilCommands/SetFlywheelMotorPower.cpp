// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/utilCommands/SetFlywheelMotorPower.h"

SetFlywheelMotorPower::SetFlywheelMotorPower(Shooter* shooter, double power)
  : m_shooter(shooter),
    m_power(power)
{
    AddRequirements(shooter);
}

void SetFlywheelMotorPower::Execute() {
    m_shooter->SetShooterFlywheelMotorPower(m_power);
}

void SetFlywheelMotorPower::End(bool interrupted) {
    m_shooter->SetShooterFlywheelMotorPower(0.0);
}

bool SetFlywheelMotorPower::IsFinished() {
    return false;
}
