// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkAbsoluteEncoder.h> 
#include "Constants/ClimberConstants.h" 
#include "Constants/GlobalConstants.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants/ClimberConstants.h"
#include <frc/DigitalInput.h>
#include <ctre/phoenix6/StatusSignal.hpp>

enum class ClimberState {
    DEFAULT,
    EXTEND_L1,
    EXTEND_L2L3,
    CLIMB,
};

class Climber : public frc2::SubsystemBase {
 public:
  Climber();
  void SetPower(double leftPower, double rightPower);
  void SetBrakeMode(BrakeMode mode);
  void SetState(ClimberState state);
  ClimberState GetState();
  void ToggleGlobalBrakeMode();
  void ConfigClimberMotors();
  units::turn_t GetLeftExtension();
  units::turn_t GetRightExtension();
  units::turn_t GetTargetExtension();
  

  BrakeMode GetBrakeMode();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  ctre::phoenix6::hardware::TalonFX m_leftMotor;
  ctre::phoenix6::hardware::TalonFX m_rightMotor;

  ctre::phoenix6::configs::TalonFXConfiguration m_climberConfig{};

  frc::DigitalInput m_climberLimitSwitch;
  BrakeMode m_brakeMode;
  bool m_lastLimitSwitchDetectionState;

  ClimberState m_state;
  units::turn_t m_targetPosition;

  ctre::phoenix6::StatusSignal<units::angle::turn_t> m_leftClimberHeight = m_leftMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::angle::turn_t> m_leftExtensionHeight = m_leftMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> m_leftExtensionVelocity = m_leftMotor.GetVelocity();

  ctre::phoenix6::StatusSignal<units::angle::turn_t> m_rightClimberHeight = m_rightMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::angle::turn_t> m_rightExtensionHeight = m_rightMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> m_rightExtensionVelocity = m_rightMotor.GetVelocity();


  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};