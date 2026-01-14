// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/utilCommands/SetShooterLoaderPower.h"

SetShooterLoaderPower::SetShooterLoaderPower(Shooter* shooter, double power)
  : m_shooter(shooter),
    m_power(power)
{
    AddRequirements(shooter);
}

void SetShooterLoaderPower::Execute() {
    m_shooter->SetShooterLoaderMotorPower(m_power);
}

void SetShooterLoaderPower::End(bool interrupted) {
    m_shooter->SetShooterLoaderMotorPower(0.0);
}

bool SetShooterLoaderPower::IsFinished() {
    return false;
}
