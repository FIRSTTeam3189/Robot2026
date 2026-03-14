// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/intake/Intake.h"

namespace commands {
  namespace util {
    class SetIntakeRotationPower : public frc2::CommandHelper<frc2::Command, SetIntakeRotationPower> {
     public:
      SetIntakeRotationPower(subsystems::intake::Intake *Intake, double power);
    
      /**
       * Will be called periodically whenever the CommandScheduler runs.
       */
      void Execute() override;
      void End(bool interrupted) override;
      bool IsFinished() override;
    
     private:
      // Components (e.g. motor controllers and sensors) should generally be
      // declared private and exposed only through public methods.
      subsystems::intake::Intake *m_intake;
      double m_power;
    };
  };
};

