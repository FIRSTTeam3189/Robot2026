// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/intake/Intake.h"

namespace commands {
  namespace util {
    /**
     * An example command.
     *
     * <p>Note that this extends CommandHelper, rather extending Command
     * directly; this is crucially important, or else the decorator functions in
     * Command will *not* work!
     */
    class SetIntakeManual
        : public frc2::CommandHelper<frc2::Command, SetIntakeManual> {
     public:
      /* You should consider using the more terse Command factories API instead
       * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
       */
      explicit SetIntakeManual(subsystems::intake::Intake *intake, subsystems::intake::State state, double power);
    
      void Initialize() override;
    
      void Execute() override;
    
      void End(bool interrupted) override;
    
      bool IsFinished() override;
    
    
    
      private:
      // Components (e.g. motor controllers and sensors) should generally be
      // declared private and exposed only through public methods.
      subsystems::intake::Intake * m_intake;
      subsystems::intake::State m_state;
      double m_power;
    };
  };
};


