// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/utilCommands/SetTurretAngle.h"

SetTurretAngle::SetTurretAngle(Shooter* shooter) : m_shooter(shooter) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_shooter);
}

// Called when the command is initially scheduled.
void SetTurretAngle::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void SetTurretAngle::Execute() {
  m_shooter->SetTurretAngle(90.0_deg);
}

// Called once the command ends or is interrupted.
void SetTurretAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool SetTurretAngle::IsFinished() {
  return false;
}
