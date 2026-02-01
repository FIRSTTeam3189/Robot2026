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
#include "subsystems/PoseEstimatorHelper.h"

#include "Constants/shooterConstants/ArticulationConstants.h"

#include "Constants/AutoAlignConstants.h"

#include "util/LinearInterpolator.h"
#include "util/alignUtils/HoodAlign.h"

#include <algorithm>
#include <cmath> 

class Articulation : public frc2::SubsystemBase {
 public:
  Articulation(SwerveDrive *swerveDrive);

  // void SetArticulationMotorPower(double power);

  enum class BrakeMode { Brake, Coast, Default };
  void SetArticulationBrakeMode(BrakeMode mode);

  void SetArticulationAngle(units::degree_t angle);

  void SetState(ArticulationTarget state); 

  units::degree_t GetArticulationAngle();

  void UpdatePreferences();

  void ConfigPID();
  
  void RefreshAllSignals();

  void IncrementManualAngle(units::degree_t angle);

  units::degree_t GetTargetAngle(); 

  void OverrideSoftLimits(bool isEnabled);

  void ConfigArticulation();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.


  ctre::phoenix6::hardware::TalonFX m_articulationMotor;

  ctre::phoenix6::configs::TalonFXConfiguration m_articulationConfig {};

  //PID/FF/Motion Magic
  std::string m_articulationPKey;
  std::string m_articulationIKey;
  std::string m_articulationDKey;
  std::string m_articulationGKey;
  std::string m_articulationSKey;
  std::string m_articulationVKey;
  std::string m_articulationAKey;
  std::string m_articulationTargetKey;

  SwerveDrive *m_swerveDrive;

  ArticulationTarget m_state; 

  units::degree_t m_targetArticulationAngle;
  units::degree_t m_currentArticulationAngle;

  units::degree_t m_manualAngle; 

  ctre::phoenix6::StatusSignal<units::angle::turn_t> m_articulationAngle = m_articulationMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> m_articulationVelocity = m_articulationMotor.GetVelocity();
  std::vector<ctre::phoenix6::BaseStatusSignal*> m_allSignals;


};