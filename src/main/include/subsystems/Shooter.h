// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants/ShooterConstants.h"
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/Preferences.h>
#include <ctre/phoenix6/SignalLogger.hpp>
#include "Constants/AutoAlignConstants.h"
#include <ctre/phoenix6/StatusSignal.hpp>
#include "util/Vector2d.h"
#include <frc/system/plant/LinearSystemId.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/system/LinearSystemLoop.h>

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  // SysId helpers (return commands that run the SysId routines)
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

  // PUBLIC control methods (commands call these)
  void SetShooterRotationMotorPower(double power);

  enum class BrakeMode { Brake, Coast, Default };
  void SetRotationBrakeMode(BrakeMode mode);

  void OverrideSoftLimits(bool isEnabled);

  void Periodic() override;

  void SetShooterFlywheelMotorPower(double power);

  void UpdatePreferences();

  void ConfigPID();

  void SetTurretAngle(units::degree_t angle);
  
  units::degree_t GetTurretAngle();

  void SetState(TurretTarget state);

  double GetTargetAngle(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d turretOffset);

 private:
  // HARDWARE (subsystem owns motors)
  ctre::phoenix6::hardware::TalonFX m_shooterTurretMotor;
  ctre::phoenix6::hardware::TalonFX m_shooterFlywheelMotor;

  // CONFIG
  ctre::phoenix6::configs::TalonFXConfiguration m_shooterConfig {};
  ctre::phoenix6::configs::TalonFXConfiguration m_flywheelConfig {};

  frc::LinearSystem<1, 1, 1> m_flywheelPlant;
  frc::KalmanFilter<1, 1, 1> m_observer;
  frc::LinearQuadraticRegulator<1, 1> m_controller;
  frc::LinearSystemLoop<1, 1, 1> m_loop;

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
  TurretTarget m_state;

  ctre::phoenix6::StatusSignal<units::angle::turn_t> m_rotationAngle = m_shooterTurretMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> m_rotationVelocity = m_shooterTurretMotor.GetVelocity();

  //SYSID (super cool PID finder)
  frc2::sysid::SysIdRoutine m_sysIdRoutine;
  ctre::phoenix6::controls::VoltageOut m_voltReq;

  void ConfigShooter();
};

