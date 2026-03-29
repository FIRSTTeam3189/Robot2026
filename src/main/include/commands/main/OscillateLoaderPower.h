// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/indexer/Loader.h"


namespace commands {
  namespace main {
    /**
     * An example command.
     *
     * <p>Note that this extends CommandHelper, rather extending Command
     * directly; this is crucially important, or else the decorator functions in
     * Command will *not* work!
     */
    class OscillateLoaderPower
        : public frc2::CommandHelper<frc2::Command, OscillateLoaderPower> {
     public:
          explicit OscillateLoaderPower(subsystems::indexer::Loader *loader, double power);
      /* You should consider using the more terse Command factories API instead
       * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
       */
    
      void Initialize() override;
    
      void Execute() override;
    
      void End(bool interrupted) override;
    
      bool IsFinished() override;
    
    
    private:
      // Components (e.g. motor controllers and sensors) should generally be
      // declared private and exposed only through public methods.
      subsystems::indexer::Loader *m_loader;
      double m_power;
      frc::Timer m_timer;
      
    };
  };
};
