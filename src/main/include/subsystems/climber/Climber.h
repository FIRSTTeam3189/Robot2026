// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "constants/Climber.h"

namespace subsystems {
  namespace climber {
    class Climber : public frc2::SubsystemBase {
     public:
      Climber();
      void ConfigClimberMotor();
      void SetClimberPower(double power);
      void RefreshAllSignals();
      units::meter_t GetExtension();
    
      /**
       * Will be called periodically whenever the CommandScheduler runs.
       */
      void Periodic() override;
    
     private:
      // Components (e.g. motor controllers and sensors) should generally be
      // declared private and exposed only through public methods.
      ctre::phoenix6::hardware::TalonFX m_climberMotor;
      ctre::phoenix6::configs::TalonFXConfiguration m_climberConfig{};
    };
  };
};

