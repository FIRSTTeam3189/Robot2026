// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/util/CalibrateTurret.h"

namespace commands::util {
  CalibrateTurret::CalibrateTurret(subsystems::shooter::Turret *turret) : m_turret(turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    AddRequirements(m_turret);
  }
  
  // Called when the command is initially scheduled.
  void CalibrateTurret::Initialize() {}
  
  // Called repeatedly when this Command is scheduled to run
  void CalibrateTurret::Execute() {
    if(m_turretResetSwitchRight.Get()){
      m_turret->SetTurretRotationMotorPower(-0.2);
    }
  }
  
  // Called once the command ends or is interrupted.
  void CalibrateTurret::End(bool interrupted) {
    m_turret->SetTurretRotationMotorPower(0.0);
    m_turret->SetTurretAngle(0.0_deg);
  
  }
  
  // Returns true when the command should end.
  bool CalibrateTurret::IsFinished() {
    return m_turretResetSwitchLeft.Get();
  }
}

