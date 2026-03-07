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

#include <rev/SparkFlex.h>
#include <rev/config/SparkFlexConfig.h>
#include <rev/SparkClosedLoopController.h>

namespace subsystems {
  namespace shooter {
    class Flywheel : public frc2::SubsystemBase {
      public:
        Flywheel(subsystems::swerve::Drive *swerveDrive);

        void ConfigFlywheelMotor();

        void SetFlywheelMotorPower(double power);

        void SetState(constants::shooter::FlywheelState flywheelState);

        void SetFlyWheelTargetPower(double power); 
        void SetFlywheelMotorVelocity(double turnsPerSecond);

        double GetFlywheelTargetVelocity();
        

        /**
         * Will be called periodically whenever the CommandScheduler runs.
         */
        void Periodic() override;

      private:
        // Components (e.g. motor controllers and sensors) should generally be
        // declared private and exposed only through public methods.
        subsystems::swerve::Drive *m_swerveDrive; 

        constants::shooter::FlywheelState m_flywheelState = constants::shooter::FlywheelState::AutoVelocity;

        util::interpolator::Linear m_powerInterpolator; 

        rev::spark::SparkFlex m_flywheelMotor;
        rev::spark::SparkFlex m_flywheelFollowMotor;
        rev::spark::SparkFlexConfig m_flywheelConfig {};
        rev::spark::SparkFlexConfig m_flywheelFollowConfig {};
        rev::spark::SparkClosedLoopController m_flywheelController;

        double m_rpm{};

    };
  };
};