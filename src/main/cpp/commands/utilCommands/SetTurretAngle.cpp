// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/utilCommands/SetTurretAngle.h"

SetTurretAngle::SetTurretAngle(Shooter* shooter, TurretTarget state) : m_shooter(shooter), m_state(state) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_shooter);
}

// Called when the command is initially scheduled.
void SetTurretAngle::Initialize() {
  m_shooter->SetState(m_state);
}

// Called repeatedly when this Command is scheduled to run
void SetTurretAngle::Execute() {
}

// Called once the command ends or is interrupted.
void SetTurretAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool SetTurretAngle::IsFinished() {
  // if (m_state == TurretTarget::AlignHub){

  // } else {

  // }
  // if (m_shooter->GetTurretAngle().value())
  return false;
}
