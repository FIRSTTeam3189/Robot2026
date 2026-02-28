// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include "subsystems/shooter/Flywheel.h"

namespace commands {
  namespace util {
    class SetFlywheelMotorPower : public frc2::CommandHelper<frc2::Command, SetFlywheelMotorPower> {
     public:
      explicit SetFlywheelMotorPower(subsystems::shooter::Flywheel *flywheel, double power);
    
      /**
       * Will be called periodically whenever the CommandScheduler runs.
       */
      void Execute() override;
      void End(bool interrupted) override;
      bool IsFinished() override;
    
     private:
      // Components (e.g. motor controllers and sensors) should generally be
      // declared private and exposed only through public methods.
      subsystems::shooter::Flywheel *m_flywheel;
      double m_power;
    };
  };
};
