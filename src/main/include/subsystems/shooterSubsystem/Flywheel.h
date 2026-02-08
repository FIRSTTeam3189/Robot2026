// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <ctre/phoenix6/SignalLogger.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>

#include "Constants/shooterConstants/FlywheelConstants.h"

class Flywheel : public frc2::SubsystemBase {
 public:
  Flywheel();

  void ConfigFlywheelMotor();

  void SetFlywheelMotorPower(double power);
  void SetFlywheelMotorPower(); 

  void SetFlyWheelTargetPower(double power); 
  
  double GetFlywheelTargetPower();
  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix6::hardware::TalonFX m_flywheelMotor;
  ctre::phoenix6::configs::TalonFXConfiguration m_flywheelConfig {};

  double m_targetPower; 

};
