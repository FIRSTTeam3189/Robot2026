// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/utilCommands/SetShooterLoaderPower.h"

SetShooterLoaderPower::SetShooterLoaderPower(Loader* loader, double power)
  : m_loader(loader),
    m_power(power)
{
    AddRequirements(loader);
}

void SetShooterLoaderPower::Initialize() {
    
}

void SetShooterLoaderPower::Execute() {
    m_loader->SetShooterLoaderMotorPower(m_power);
}

void SetShooterLoaderPower::End(bool interrupted) {
    m_loader->SetShooterLoaderMotorPower(0.0);
}

bool SetShooterLoaderPower::IsFinished() {
    return false;
}
