// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/util/SetLoaderPower.h"

namespace commands::util {
  SetLoaderPower::SetLoaderPower(subsystems::indexer::Loader *loader, double power)
    : m_loader(loader),
      m_power(power),
      m_timer()
  {
      AddRequirements(loader);
  }
  
  // Called when the command is initially scheduled.
  void SetLoaderPower::Initialize() {
    // m_loader->SetLoaderMotorPower(m_power);
    m_timer.Start(); 
  }
  
  // Called repeatedly when this Command is scheduled to run
  void SetLoaderPower::Execute() {
    units::time::second_t elapsed = m_timer.Get(); 
    frc::SmartDashboard::PutNumber("timer stuff/loader timer seconds passed", elapsed.value()); 
    double targetPower = m_power; 
    if (elapsed >= 2.0_s && elapsed <= 2.5_s) { //unstuck balls for 0.5s
        targetPower = -m_power; 
    } else if (elapsed > 2.5_s) {
      m_timer.Restart();
    } 
    m_loader->SetLoaderMotorPower(targetPower);
  }
  
  // Called once the command ends or is interrupted.
  void SetLoaderPower::End(bool interrupted) {
    // m_loader->SetLoaderMotorPower(0.0);
  }
  
  // Returns true when the command should end.
  bool SetLoaderPower::IsFinished() {
    return false;
    // return true; 
  }

}