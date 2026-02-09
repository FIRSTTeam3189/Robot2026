// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/utilCommands/SetClimberState.h"

SetClimberState::SetClimberState(Climber* climber, Intake* intake, ClimberState state) 
: m_climber(climber), m_intake(intake), m_state(state) {
  AddRequirements({climber, intake});
}

// Called when the command is initially scheduled.
void SetClimberState::Initialize() {
  if (m_intake->GetState() == IntakeState::RetractIntake) {
     m_climber->SetState(m_state);
  }
}

// Called repeatedly when this Command is scheduled to run
void SetClimberState::Execute() {

  switch (m_state) {
    case (ClimberState::DEFAULT): {
      m_climber->SetPower(0.0, 0.0);
      break;
    }

    case (ClimberState::CLIMB): {
      m_climber->SetPower(-ClimberConstants::kClimberPower, -ClimberConstants::kClimberPower);
      break;
    }

    case (ClimberState::EXTEND_L1): {
      m_climber->SetPower(ClimberConstants::kClimberPower, ClimberConstants::kClimberPower);
      break;
    }

    case (ClimberState::EXTEND_L2L3): {
      m_climber->SetPower(ClimberConstants::kClimberPower, ClimberConstants::kClimberPower);
      break;
    }
  }
}

// Called once the command ends or is interrupted.
void SetClimberState::End(bool interrupted) {}

// Returns true when the command should end.
bool SetClimberState::IsFinished() {
  if (m_intake->GetState() != IntakeState::RetractIntake) {
    return true;
  }

  bool leftReached = false;
  bool rightReached = false;

  units::turn_t leftTargetDiff = m_climber->GetLeftExtension() - m_climber->GetTargetExtension();
  units::turn_t rightTargetDiff = m_climber->GetRightExtension() - m_climber->GetTargetExtension();

  switch (m_state) {
    case (ClimberState::DEFAULT): {
      leftReached = leftTargetDiff <= ClimberConstants::kMinTurnTolerance;
      rightReached = rightTargetDiff <= ClimberConstants::kMinTurnTolerance;

      break;
    }

    case (ClimberState::CLIMB): {
      leftReached = leftTargetDiff <= ClimberConstants::kMinTurnTolerance;
      rightReached = rightTargetDiff <= ClimberConstants::kMinTurnTolerance;

      break;
    }

    case (ClimberState::EXTEND_L1): {
      leftReached = leftTargetDiff >= ClimberConstants::kMaxTurnTolerance;
      rightReached = rightTargetDiff >= ClimberConstants::kMaxTurnTolerance;

      break;
    }

    case (ClimberState::EXTEND_L2L3): {
      leftReached = leftTargetDiff >= ClimberConstants::kMaxTurnTolerance;
      rightReached = rightTargetDiff >= ClimberConstants::kMaxTurnTolerance;

      break;
    }
  }

  return leftReached && rightReached;
}
