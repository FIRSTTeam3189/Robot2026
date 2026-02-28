// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp> 
#include <ctre/phoenix6/SignalLogger.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp> 

#include "subsystems/swerve/Drive.h" 

#include "constants/shooter/Flywheel.h"
#include "constants/AutoAlign.h"

#include "util/align/Turret.h"
#include "util/interpolator/Linear.h"
#include "util/align/Hood.h"


namespace subsystems {
  namespace shooter {
    class Flywheel : public frc2::SubsystemBase {
      public:
        Flywheel(subsystems::swerve::Drive *swerveDrive);

        void ConfigFlywheelMotor();

        void SetFlywheelMotorPower(double power);

        void SetFlyWheelTargetPower(double power); 
        void SetFlyWheelMotorVelocity(units::turns_per_second_t turnsPerSecond);
        
        units::turns_per_second_t GetFlywheelTargetVelocity();
        

        /**
         * Will be called periodically whenever the CommandScheduler runs.
         */
        void Periodic() override;

      private:
        // Components (e.g. motor controllers and sensors) should generally be
        // declared private and exposed only through public methods.
        subsystems::swerve::Drive *m_swerveDrive; 

        util::interpolator::Linear m_powerInterpolator; 

        ctre::phoenix6::hardware::TalonFX m_flywheelMotor;
        ctre::phoenix6::configs::TalonFXConfiguration m_flywheelConfig {};

    };
  };
};
