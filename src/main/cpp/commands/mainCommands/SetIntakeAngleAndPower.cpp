// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/mainCommands/SetIntakeAngleAndPower.h"

SetIntakeAngleAndPower::SetIntakeAngleAndPower(Intake *intake, double rollerPower, bool retractOrExtend) :
m_intake(intake), m_power(rollerPower), m_retractOrExtend(m_retractOrExtend) {
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void SetIntakeAngleAndPower::Initialize() {
  if (m_retractOrExtend){
    m_intake->SetIntake(m_power);
  } else {
    m_intake->RetractIntake();
  }
}

// Called repeatedly when this Command is scheduled to run
void SetIntakeAngleAndPower::Execute() {}

// Called once the command ends or is interrupted.
void SetIntakeAngleAndPower::End(bool interrupted) {
  m_intake->SetRollerPower(0.0);
}

// Returns true when the command should end.
bool SetIntakeAngleAndPower::IsFinished() {
  return false;
}
