// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/utilCommands/SetArticulationPower.h"

SetArticulationPower::SetArticulationPower(Articulation* Articulation, double power)
  : m_articulation(Articulation),
    m_power(power)
{
    AddRequirements(Articulation);
}

void SetArticulationPower::Execute() {
    m_articulation->SetArticulationMotorPower(m_power);
}

void SetArticulationPower::End(bool interrupted) {
    m_articulation->SetArticulationMotorPower(0.0);
}

bool SetArticulationPower::IsFinished() {
    return false;
}
