// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants/shooterConstants/LoaderConstants.h"

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkClosedLoopController.h>



class Loader : public frc2::SubsystemBase {
 public:
  Loader();

  void SetLoaderMotorPower(double power);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::spark::SparkMax m_loaderMotor;
  rev::spark::SparkMaxConfig m_loaderConfig {};

  void ConfigLoaderMotor();

};