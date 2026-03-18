// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/util/SetIntakeState.h"
namespace commands::util {
  SetIntakeState::SetIntakeState(subsystems::intake::Intake *intake, double power)
  : m_intake(intake), m_power(power) 
  {
    AddRequirements(intake);
  }


  // Called when the command is initially scheduled.
  void SetIntakeState::Initialize() {
    if (m_intake->GetState() == subsystems::intake::State::ExtendIntake) {
      m_intake->SetState(subsystems::intake::State::RetractIntake); 
      m_intake->SetDesiredRollerPower(0.0); 
    } else {
      m_intake->SetState(subsystems::intake::State::ExtendIntake); 
      m_intake->SetDesiredRollerPower(m_power); 
    } 
  }

  // Called repeatedly when this Command is scheduled to run
  void SetIntakeState::Execute() {
    // if (m_intake->GetArticulationAngle() <= constants::intake::kAngleThresholdRollers){
    //   if (m_intake->GetState() == subsystems::intake::State::ExtendIntake) {
    //     m_intake->SetRollerPower(m_power); 
    //   } else {
    //     m_intake->SetRollerPower(0.0); 
    //   }
    // }

  }

  // Called once the command ends or is interrupted.
  void SetIntakeState::End(bool interrupted) {
  }

  // Returns true when the command should end.
  bool SetIntakeState::IsFinished() {
    // double difference = std::abs(m_intake->GetArticulationAngle().value() - m_intake->GetTargetAngle()); 
    // return difference <= constants::intake::kTargetAngleThreshold.value();
    return true; 
  }
}