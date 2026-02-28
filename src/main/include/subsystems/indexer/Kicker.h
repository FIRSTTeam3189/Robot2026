// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "constants/indexer/Kicker.h"
#include <frc/smartdashboard/SmartDashboard.h>

namespace subsystems {
  namespace indexer {
    class Kicker : public frc2::SubsystemBase {
      public:
        Kicker();
        void SetKickerMotorPower(double power);

        /**
         * Will be called periodically whenever the CommandScheduler runs.
         */
        void Periodic() override;

      private:
        // Components (e.g. motor controllers and sensors) should generally be
        // declared private and exposed only through public methods.
        ctre::phoenix6::hardware::TalonFX m_kickerMotor;

        ctre::phoenix6::configs::TalonFXConfiguration m_kickerConfig {};

        void ConfigKickerMotor();

    };
  };
};