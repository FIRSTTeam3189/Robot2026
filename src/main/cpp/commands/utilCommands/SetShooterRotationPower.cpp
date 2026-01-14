// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/utilCommands/SetShooterRotationPower.h"

SetShooterRotationPower::SetShooterRotationPower(Shooter* shooter, double power)
  : m_shooter(shooter),
    m_power(power)
{
    AddRequirements(shooter);
}

void SetShooterRotationPower::Execute() {
    m_shooter->SetShooterRotationMotorPower(m_power);
}

void SetShooterRotationPower::End(bool interrupted) {
    m_shooter->SetShooterRotationMotorPower(0.0);
}

bool SetShooterRotationPower::IsFinished() {
    return false;
}
