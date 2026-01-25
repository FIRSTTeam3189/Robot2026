// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/utilCommands/SetIntakeMotorPower.h"

SetIntakeMotorPower::SetIntakeMotorPower(Intake *intake, double power) : m_intake(intake), m_rollerPower(power) {
  // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(); 
}

// Called when the command is initially scheduled.
void SetIntakeMotorPower::Initialize() {
    m_intake->SetRollerPower(m_rollerPower); 
}

// Called repeatedly when this Command is scheduled to run
void SetIntakeMotorPower::Execute() {

}

// Called once the command ends or is interrupted.
void SetIntakeMotorPower::End(bool interrupted) {}

// Returns true when the command should end.
bool SetIntakeMotorPower::IsFinished() {
  return false;
}
