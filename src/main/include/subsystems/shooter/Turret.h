// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Preferences.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include <frc/estimator/KalmanFilter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/LinearSystemId.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <ctre/phoenix6/SignalLogger.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>

#include "subsystems/swerve/Drive.h"


#include "constants/shooter/Turret.h"
#include "constants/AutoAlign.h"

#include "util/align/Turret.h"

#include <algorithm>
#include <cmath> 


namespace subsystems {
  namespace shooter {
    class Turret : public frc2::SubsystemBase {
      public:
        Turret(subsystems::swerve::Drive* swerveDrive);

        // SysId helpers (return commands that run the SysId routines)
        frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
        frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

        // PUBLIC control methods (commands call these)
        void SetTurretRotationMotorPower(double power);

        void SetRotationBrakeMode(constants::BrakeMode mode);

        void OverrideSoftLimits(bool isEnabled);

        void Periodic() override;

        void UpdatePreferences();

        void ConfigPID();

        units::degree_t GetCurrentTargetAngle();

        void RefreshAllSignals();

        void SetTurretAngle(units::degree_t angle);
        
        units::degree_t GetTurretAngle();

        void SetState(constants::shooter::TurretTarget state);

        Vector2d GetRobotVelocity(); 
        
        private:
        // HARDWARE (subsystem owns motors)
        ctre::phoenix6::hardware::TalonFX m_turretMotor;
        
        // CONFIG
        ctre::phoenix6::configs::TalonFXConfiguration m_shooterConfig {};
        
        subsystems::swerve::Drive *m_swerveDrive;
        constants::shooter::TurretTarget m_state;
        frc::Field2d m_turretField;
        frc::Field2d m_targetField;
        frc::Field2d m_velocityField;
        
        
        //PID/FF/Motion Magic
        std::string m_rotationPKey;
        std::string m_rotationIKey;
        std::string m_rotationDKey;
        std::string m_rotationGKey;
        std::string m_rotationSKey;
        std::string m_rotationVKey;
        std::string m_rotationAKey;
        std::string m_rotationTargetKey;

        units::degree_t m_targetRotationAngle;
        units::degree_t m_currentRotationAngle;

        double m_targetPower;
      
        ctre::phoenix6::StatusSignal<units::angle::turn_t> m_rotationAngle = m_turretMotor.GetPosition();
        ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> m_rotationVelocity = m_turretMotor.GetVelocity();
        std::vector<ctre::phoenix6::BaseStatusSignal*> m_allSignals;

        void ConfigTurret();
    };
  };
};

