// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/main/ShakeIntake.h"

namespace commands::main {

  ShakeIntake::ShakeIntake(subsystems::intake::Intake *intake, double power) :
  m_intake(intake), m_power(power), m_timer(), m_shakes() {
    AddRequirements(intake);
  }
  
  // Called when the command is initially scheduled.
  void ShakeIntake::Initialize() {
    m_timer.Start(); 
    m_intake->SetDesiredRollerPower(m_power); 
  }

  // Called repeatedly when this Command is scheduled to run
  void ShakeIntake::Execute() {
    frc::SmartDashboard::PutNumber("Intake State", static_cast<int>(m_intake->GetState()));
    units::time::second_t elapsed = m_timer.Get(); 
    frc::SmartDashboard::PutNumber("timer stuff/intake timer seconds passed", elapsed.value()); 

    units::degree_t currentAngle = m_intake->GetArticulationAngle(); 

    if (elapsed > constants::intake::kIntakeShakeRepeat && m_intake->WithinTargetThreshold()) { 
        if (m_intake->GetState() == subsystems::intake::State::CompressIntakeUp) {
            m_intake->SetState(subsystems::intake::State::ExtendIntake); 
        } else {
            m_intake->SetState(subsystems::intake::State::CompressIntakeUp); 
        }
        m_shakes++; // count of number of cycles, one full cycle is 2 full shakes, like go up as 1 shake, and go down as the second 
    } else if (elapsed > constants::intake::kIntakeShakeRepeat && m_shakes >= 2) {
      m_timer.Restart();
      m_shakes = 0; 
    } 
  }
  
  // Called once the command ends or is interrupted.
  void ShakeIntake::End(bool interrupted) {
    // m_intake->SetRollerPower(0.0);
  }
  
  // Returns true when the command should end.
  bool ShakeIntake::IsFinished() {
    return false;
  }
  
}

