// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants/ShooterConstants.h"

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  // PUBLIC control methods (commands call these)
  void SetShooterRotationMotorPower(double power);

  enum class BrakeMode { Brake, Coast, Default };
  void SetRotationBrakeMode(BrakeMode mode);

  void OverrideSoftLimits(bool isEnabled);

  void Periodic() override;

 private:
  // HARDWARE (subsystem owns motors)
  ctre::phoenix6::hardware::TalonFX m_shooterTurretMotor;
  ctre::phoenix6::hardware::TalonFX m_loaderMotor;

  // CONFIG
  ctre::phoenix6::configs::TalonFXConfiguration m_shooterConfig {};


  void ConfigShooter();
};

