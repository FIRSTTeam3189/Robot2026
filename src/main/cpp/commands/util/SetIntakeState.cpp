// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/util/SetIntakeState.h"
namespace commands::util {
  SetIntakeState::SetIntakeState(subsystems::intake::Intake *intake, subsystems::intake::State state, double power)
  : m_intake(intake), m_state(state), m_power(power) 
  {
    AddRequirements(intake);
  }


  // Called when the command is initially scheduled.
  void SetIntakeState::Initialize() {
    m_intake->SetState(m_state, m_power);
  }

  // Called repeatedly when this Command is scheduled to run
  void SetIntakeState::Execute() {

  }

  // Called once the command ends or is interrupted.
  void SetIntakeState::End(bool interrupted) {
  }

  // Returns true when the command should end.
  bool SetIntakeState::IsFinished() {
    return true;
  }
}