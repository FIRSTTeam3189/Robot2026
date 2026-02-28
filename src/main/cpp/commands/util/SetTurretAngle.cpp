// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/util/SetTurretAngle.h"

namespace commands::util {
  SetTurretAngle::SetTurretAngle(subsystems::shooter::Turret *shooter, constants::shooter::TurretTarget state) : m_turret(shooter), m_state(state) {
    // Use addRequirements() here to declare subsystem dependencies.
    AddRequirements(m_turret);
  }
  
  // Called when the command is initially scheduled.
  void SetTurretAngle::Initialize() {
    // m_turret->SetState(m_state);
  }
  
  // Called repeatedly when this Command is scheduled to run
  void SetTurretAngle::Execute() {
    m_turret->SetState(m_state);
  }
  
  // Called once the command ends or is interrupted.
  void SetTurretAngle::End(bool interrupted) {}
  
  // Returns true when the command should end.
  bool SetTurretAngle::IsFinished() {
  
    // if (m_state == constants::shooter::TurretTarget::AlignHub){
  
    // } else {
  
    // }
    // if (m_turret->GetTurretAngle().value())
    return True;
  }
}

