// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/Preferences.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/TimedRobot.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <math.h>
#include <iostream>
#include <string>
#include <frc/geometry/Translation2d.h>
#include "subsystems/vision/PoseEstimatorHelper.h"
#include "util/SwerveModule.h"
#include <ctre/phoenix6/Pigeon2.hpp>
#include "constants/Vision.h"
#include "constants/Automatic.h"
#include "subsystems/swerve/State.h"
#include "subsystems/swerve/Modules.h"

namespace subsystems {
  
  namespace swerve {
    class Drive : public frc2::SubsystemBase {
      public:
        Drive(subsystems::vision::PoseEstimatorHelper *helper);
        void ConfigGyro();
        
        void Stop();
        void Operate(units::meters_per_second_t xSpeed,
                              units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
                              bool fieldRelative,
                              frc::Translation2d centerOfRotation);
        void UpdateEstimator();
        void DriveRobotRelative(frc::ChassisSpeeds speeds);
        void SetModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates);
        units::degree_t GetNormalizedYaw();
        units::degree_t GetNormalizedYawPigeon();
        void RefreshAllSignals();
        void ConfigSignals();
        void SetBrakeMode(constants::BrakeMode mode);
        void SetPose(frc::Pose2d pose, bool justRotation);
        void SetAutoPose(frc::Pose2d pose, bool justRotation);
        pathplanner::PathPlannerTrajectory GetOnFlyTrajectory();
        void SetSlowMode(bool slow);
        frc::Pose2d GetEstimatedPose();
        frc::ChassisSpeeds GetRobotRelativeSpeeds();

        void LogModuleStates(wpi::array<frc::SwerveModulePosition, 4> modulePositions);
        // frc2::SequentialCommandGroup GetPathfindingPath(std::shared_ptr<pathplanner::PathPlannerPath> m_scorePath, std::shared_ptr<pathplanner::PathPlannerPath> m_retreatPath, std::string coralLevelHeightCommand);
        void SetTimestampOffset(units::second_t offset);

        /**
         * Will be called periodically whenever the CommandScheduler runs.
         */
        void Periodic() override;
        void SimulationPeriodic() override;

      private:

        // Components (e.g. motor controllers and sensors) should generally be
        // declared private and exposed only through public methods.
        Modules m_modules;

        wpi::array<SwerveModule*, 4> m_moduleArray;
        subsystems::vision::PoseEstimatorHelper *m_poseHelper;
        ctre::phoenix6::hardware::Pigeon2 m_pigeon;
        ctre::phoenix6::sim::Pigeon2SimState& m_pigeonSim;
        units::degree_t m_pigeonSimYaw {0};
        wpi::array<frc::SwerveModulePosition, 4> m_modulePositions;
        std::string m_drivePKey;
        std::string m_driveIKey;
        std::string m_driveDKey;
        std::string m_anglePKey;
        std::string m_angleIKey;
        std::string m_angleDKey;
        std::string m_rotationSKey;

        double m_rotationS = constants::swerve::swerve_drive::kSRot;

        std::vector<ctre::phoenix6::BaseStatusSignal*> m_allSignals;

        ctre::phoenix6::configs::Pigeon2Configuration m_pigeonConfigs{};

        std::string_view m_tuningModeKey = "Tuning Mode";
        std::string_view m_diagnosticsKey = "Full Diagnostics";
        bool m_slowMode = false;
        units::second_t m_timestampOffset;
    };
  }; 
};
