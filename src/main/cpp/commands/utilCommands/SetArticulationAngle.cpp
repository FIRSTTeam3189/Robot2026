// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of

// the WPILib BSD license file in the root directory of this project.



#include "commands/utilCommands/SetArticulationAngle.h"



SetArticulationAngle::SetArticulationAngle(Articulation* articulation, ArticulationTarget state) : m_articulation(articulation), m_state(state) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_articulation);
}

// Called when the command is initially scheduled.
void SetArticulationAngle::Initialize() {
  // m_shooter->SetState(m_state);
}


// Called repeatedly when this Command is scheduled to run
void SetArticulationAngle::Execute() {
  m_articulation->SetState(m_state);
}

// Called once the command ends or is interrupted.
void SetArticulationAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool SetArticulationAngle::IsFinished() {

  return false;

}

