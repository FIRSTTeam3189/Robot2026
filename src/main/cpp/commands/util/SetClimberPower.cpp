// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/util/SetClimberPower.h"

namespace commands::util {
  SetClimberPower::SetClimberPower(subsystems::climber::Climber *climber, double power)
      : m_climber(climber),
        m_power(power)
    {
        AddRequirements(climber);
    }
  
  // Called when the command is initially scheduled.
  void SetClimberPower::Initialize() {
    
  }
  
  // Called repeatedly when this Command is scheduled to run
  void SetClimberPower::Execute() {
    m_climber->SetClimberPower(m_power);
  }
  
  // Called once the command ends or is interrupted.
  void SetClimberPower::End(bool interrupted) {}
  
  // Returns true when the command should end.
  bool SetClimberPower::IsFinished() {
    return true;
  }
}

