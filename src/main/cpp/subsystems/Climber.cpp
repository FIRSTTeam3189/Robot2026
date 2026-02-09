// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"





Climber::Climber() : 
m_leftMotor(ClimberConstants::kLeftClimberMotorID, "Swerve"),
m_rightMotor(ClimberConstants::kRightClimberMotorID, "Swerve"),
m_climberLimitSwitch(ClimberConstants::kClimberLimitSwitchID),
m_climberConfig() {
    ConfigClimberMotors();
}

// This method will be called once per scheduler run
void Climber::Periodic() {
  frc::SmartDashboard::PutBoolean("Climber Latched", m_climberLimitSwitch.Get());
}

void Climber::ConfigClimberMotors() {
    m_leftMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));
    m_rightMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));
    // m_climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = ClimberConstants::kSoftLimitEnabledForward;
    // m_climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = ClimberConstants::kSoftLimitEnabledReverse;
    // m_climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants::kTopSoftLimit;
    // m_climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants::kBottomSoftLimit;
    m_climberConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
    // m_climberConfig.Feedback.FeedbackRotorOffset = ClimberConstants::kExtensionOffset;
    m_climberConfig.CurrentLimits.SupplyCurrentLowerLimit = ClimberConstants::kExtensionContinuousCurrentLimit;
    m_climberConfig.CurrentLimits.SupplyCurrentLimit = ClimberConstants::kExtensionPeakCurrentLimit;
    m_climberConfig.CurrentLimits.SupplyCurrentLowerTime = ClimberConstants::kExtensionPeakCurrentDuration;
   // m_climberConfig.CurrentLimits.SupplyCurrentLimitEnable = ClimberConstants::kExtensionEnableCurrentLimit;
    m_climberConfig.MotorOutput.Inverted = ClimberConstants::kExtensionMotorInverted;
    m_climberConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    m_leftMotor.GetConfigurator().Apply(m_climberConfig);
    m_rightMotor.GetConfigurator().Apply(m_climberConfig);
    m_leftMotor.SetPosition(0.0_tr);
    m_rightMotor.SetPosition(0.0_tr);
}

void Climber::SetPower(double leftPower, double rightPower){
    m_leftMotor.Set(leftPower);
    m_rightMotor.Set(rightPower);

    // frc::SmartDashboard::PutNumber("Climber left output", leftPower);
    // frc::SmartDashboard::PutNumber("Climber right output", rightPower);

    // frc::SmartDashboard::PutNumber("Right Motor Current (amps)", m_rightMotor.GetSupplyCurrent());
    // frc::SmartDashboard::PutNumber("Left Motor Current (amps)", m_leftMotor.GetSupplyCurrent());
}


void Climber::SetState(ClimberState state) {   
    m_state = state;

    switch (state) {
        case (ClimberState::DEFAULT): {
            m_targetPosition = ClimberConstants::kDefaultMotorTurns;
            break;
        }

        case (ClimberState::CLIMB): {
            m_targetPosition = ClimberConstants::kDefaultMotorTurns;
            break;
        }

        case (ClimberState::EXTEND_L1): {
            m_targetPosition = ClimberConstants::kL1MotorTurns;
            break;
        }

        case (ClimberState::EXTEND_L2L3): {
            m_targetPosition = ClimberConstants::kL2L3MotorTurns;
            break;
        }

        default:
            m_targetPosition = ClimberConstants::kDefaultMotorTurns;
    }
}

ClimberState Climber::GetState() {
    return m_state;
}

units::turn_t Climber::GetLeftExtension() {
    return units::turn_t {
        ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_leftExtensionHeight, m_leftExtensionVelocity).value()
    };
}

units::turn_t Climber::GetRightExtension() {
    return units::turn_t {
        ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_rightExtensionHeight, m_rightExtensionVelocity).value()
    };
}

units::turn_t Climber::GetTargetExtension() {
    return m_targetPosition;
}

void Climber::SetBrakeMode(BrakeMode mode) {
    switch (mode) {
        case(BrakeMode::Brake) :
            m_climberConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
            break;
        case(BrakeMode::Coast) :
            m_climberConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
            break;
        case(BrakeMode::Default) :
            m_climberConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
            break;
        default :
            break;
    }

    m_leftMotor.GetConfigurator().Apply(m_climberConfig);
    m_rightMotor.GetConfigurator().Apply(m_climberConfig);
}

void Climber::ToggleGlobalBrakeMode() {
    if (m_brakeMode == BrakeMode::Default) {
        m_brakeMode = BrakeMode::Coast;
    } else {
        m_brakeMode = BrakeMode::Default;
    }
}

BrakeMode Climber::GetBrakeMode() {
    return m_brakeMode;
}