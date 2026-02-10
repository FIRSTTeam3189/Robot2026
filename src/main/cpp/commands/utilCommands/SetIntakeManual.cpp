// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/utilCommands/SetIntakeManual.h"

SetIntakeManual::SetIntakeManual(Intake *intake, IntakeState state, double power)
 : m_intake(intake), m_state(state), m_power(power) 
{
  AddRequirements(intake);
}


// Called when the command is initially scheduled.
void SetIntakeManual::Initialize() {
  m_intake->SetState(m_state, m_power);
}

// Called repeatedly when this Command is scheduled to run
void SetIntakeManual::Execute() {

}

// Called once the command ends or is interrupted.
void SetIntakeManual::End(bool interrupted) {
}

// Returns true when the command should end.
bool SetIntakeManual::IsFinished() {
  return true;
}
