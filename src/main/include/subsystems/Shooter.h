// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Preferences.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include <frc/estimator/KalmanFilter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/LinearSystemId.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <ctre/phoenix6/SignalLogger.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>

#include "subsystems/SwerveDrive.h"

#include "Constants/ShooterConstants.h"
#include "Constants/AutoAlignConstants.h"

#include "util/PowerInterpolator.h"

#include <algorithm>
#include <cmath> 

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter(SwerveDrive* swerveDrive, PowerInterpolator* powerInterpolator);

  // SysId helpers (return commands that run the SysId routines)
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

  // PUBLIC control methods (commands call these)
  void SetShooterRotationMotorPower(double power);

  enum class BrakeMode { Brake, Coast, Default };
  void SetRotationBrakeMode(BrakeMode mode);

  frc::Pose2d UpdateTargetWithVelocity(frc::Pose2d robotPose, frc::Pose2d currentPose);

  frc::Pose2d CalculateTargetPose(frc::Pose2d robotPose);

  void OverrideSoftLimits(bool isEnabled);

  void Periodic() override;

  void SetShooterFlywheelMotorPower(double power);

  void UpdatePreferences();

  void ConfigPID();

  units::degree_t GetCurrentTargetAngle();

  void RefreshAllSignals();

  void SetTurretAngle(units::degree_t angle);
  
  units::degree_t GetTurretAngle();

  void SetState(TurretTarget state);

  void SetFlyWheelTargetPower(double power); 

  void SetFlyWheelPowerTest(double power); 

  double GetTargetAngle(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d turretOffset);

  Vector2d GetRobotSpeed(); 
  
  private:
  // HARDWARE (subsystem owns motors)
  ctre::phoenix6::hardware::TalonFX m_shooterTurretMotor;
  ctre::phoenix6::hardware::TalonFX m_shooterFlywheelMotor;
  
  // CONFIG
  ctre::phoenix6::configs::TalonFXConfiguration m_shooterConfig {};
  ctre::phoenix6::configs::TalonFXConfiguration m_flywheelConfig {};
  
  SwerveDrive *m_swerveDrive;
  TurretTarget m_state;
  frc::Field2d m_turretField;
  frc::Field2d m_targetField;
  
  //PID/FF/Motion Magic
  std::string m_rotationPKey;
  std::string m_rotationIKey;
  std::string m_rotationDKey;
  std::string m_rotationGKey;
  std::string m_rotationSKey;
  std::string m_rotationVKey;
  std::string m_rotationAKey;
  std::string m_rotationTargetKey;

  units::degree_t m_targetRotationAngle;
  units::degree_t m_currentRotationAngle;

  double m_targetPower;
  PowerInterpolator *m_powerInterpolator; 

  ctre::phoenix6::StatusSignal<units::angle::turn_t> m_rotationAngle = m_shooterTurretMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> m_rotationVelocity = m_shooterTurretMotor.GetVelocity();
  std::vector<ctre::phoenix6::BaseStatusSignal*> m_allSignals;

  void ConfigShooter();
};

