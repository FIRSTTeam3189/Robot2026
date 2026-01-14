// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Shooter.h"

class SetShooterRotationPower : public frc2::CommandHelper<frc2::Command, SetShooterRotationPower> {
 public:
  SetShooterRotationPower(Shooter* shooter, double power);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  Shooter *m_shooter;
  double m_power;
};
