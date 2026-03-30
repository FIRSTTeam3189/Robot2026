// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/main/OscillateLoaderPower.h"

namespace commands::main {
  OscillateLoaderPower::OscillateLoaderPower(subsystems::indexer::Loader *loader, double power)
    : m_loader(loader),
      m_power(power),
      m_timer()
  {
      AddRequirements(loader);
  }
  
  // Called when the command is initially scheduled.
  void OscillateLoaderPower::Initialize() {
    m_timer.Start(); 
  }
  
  // Called repeatedly when this Command is scheduled to run
  void OscillateLoaderPower::Execute() {
    units::time::second_t elapsed = m_timer.Get(); 
    frc::SmartDashboard::PutNumber("timer stuff/loader timer seconds passed", elapsed.value()); 

    units::time::second_t periodStart = constants::indexer::loader::kOscillatePeriod; 
    units::time::second_t periodStop = periodStart + constants::indexer::loader::kOscillateDuration; 

    double targetPower = m_power; 
    if (elapsed >= periodStart && elapsed <= periodStop) { //unstuck balls for 0.5s
        targetPower = -0.95; 
    } else if (elapsed > periodStop) {
      m_timer.Restart();
    } 
    
    m_loader->SetLoaderMotorPower(targetPower);
  }
  
  // Called once the command ends or is interrupted.
  void OscillateLoaderPower::End(bool interrupted) {
  }
  
  // Returns true when the command should end.
  bool OscillateLoaderPower::IsFinished() {
    return false; 
  }

}