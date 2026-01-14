// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include "subsystems/ShooterLoader.h"

class SetShooterLoaderPower : public frc2::CommandHelper<frc2::Command, SetShooterLoaderPower> {
 public:
  explicit SetShooterLoaderPower(Loader* loader, double power);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  Loader* m_loader;
  double m_power;
};
