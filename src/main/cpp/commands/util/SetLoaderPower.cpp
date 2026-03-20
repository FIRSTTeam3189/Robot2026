// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/util/SetLoaderPower.h"

namespace commands::util {
  SetLoaderPower::SetLoaderPower(subsystems::indexer::Loader *loader, double power)
    : m_loader(loader),
      m_power(power)
  {
      AddRequirements(loader);
  }
  
  // Called when the command is initially scheduled.
  void SetLoaderPower::Initialize() {
    m_loader->SetLoaderMotorPower(m_power);
  }
  
  // Called repeatedly when this Command is scheduled to run
  void SetLoaderPower::Execute() {
  }
  
  // Called once the command ends or is interrupted.
  void SetLoaderPower::End(bool interrupted) {
    // m_loader->SetLoaderMotorPower(0.0);
  }
  
  // Returns true when the command should end.
  bool SetLoaderPower::IsFinished() {
    return true;
  }

}