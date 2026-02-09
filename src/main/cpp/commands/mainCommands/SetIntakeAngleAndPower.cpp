// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/mainCommands/SetIntakeAngleAndPower.h"

SetIntakeAngleAndPower::SetIntakeAngleAndPower(Intake *intake, Climber* climber, double power) :
m_intake(intake), m_climber(climber), m_power(power) {
  AddRequirements({intake, climber});
}

// Called when the command is initially scheduled.
void SetIntakeAngleAndPower::Initialize() {
  if (m_climber->GetState() == ClimberState::DEFAULT) {
    frc::SmartDashboard::PutNumber("Intake State", static_cast<int>(m_intake->GetState()));
    if (m_intake->GetState() == IntakeState::ExtendIntake){
      m_intake->SetState(IntakeState::RetractIntake, m_power);
    } else {
      m_intake->SetState(IntakeState::ExtendIntake, m_power);
    }
  }

}

// Called repeatedly when this Command is scheduled to run
void SetIntakeAngleAndPower::Execute() {}

// Called once the command ends or is interrupted.
void SetIntakeAngleAndPower::End(bool interrupted) {
  // m_intake->SetRollerPower(0.0);
}

// Returns true when the command should end.
bool SetIntakeAngleAndPower::IsFinished() {
  return false;
}
