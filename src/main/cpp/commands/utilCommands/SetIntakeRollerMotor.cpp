// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/utilCommands/SetIntakeRollerMotor.h"

SetIntakeRollerMotor::SetIntakeRollerMotor(Intake *intake, double rollerPower) :
m_intake(intake), m_rollerPower(rollerPower) {
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void SetIntakeRollerMotor::Initialize() {
  m_intake->SetRollerPower(m_rollerPower);
}

// Called repeatedly when this Command is scheduled to run
void SetIntakeRollerMotor::Execute() {}

// Called once the command ends or is interrupted.
void SetIntakeRollerMotor::End(bool interrupted) {
  
}

// Returns true when the command should end.
bool SetIntakeRollerMotor::IsFinished() {
  return true;
}
