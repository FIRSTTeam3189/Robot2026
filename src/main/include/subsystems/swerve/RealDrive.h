// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/Pigeon2.hpp>

#include "util/SwerveModule.h"

#include "subsystems/swerve/Modules.h"
#include "subsystems/vision/PoseEstimatorHelper.h"
#include "subsystems/swerve/Drive.h"


namespace subsystems {
  namespace swerve {
    class RealDrive : public Drive {
      public:
        RealDrive(subsystems::vision::PoseEstimatorHelper *helper);
        
        // Unique methods
        void ConfigGyro();
        void ConfigSignals();
        void RefreshAllSignals();
        units::degree_t GetNormalizedYawPigeon();
        pathplanner::PathPlannerTrajectory GetOnFlyTrajectory();
        
        // Methods inherited from Drive.h

        // Motion methods
        void Stop() override;
        void Operate(
          units::meters_per_second_t xSpeed,
          units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
          bool fieldRelative,
          frc::Translation2d centerOfRotation
        ) override;
        void DriveRobotRelative(frc::ChassisSpeeds speeds) override;
        void SetModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates) override;
        
        // Pose/odometry methods
        void UpdateEstimator() override;
        void SetPose(frc::Pose2d pose, bool justRotation) override;
        void SetAutoPose(frc::Pose2d pose, bool justRotation) override;
        frc::Pose2d GetEstimatedPose() override;

        // Sensor methods
        units::degree_t GetNormalizedYaw() override;
        frc::ChassisSpeeds GetRobotRelativeSpeeds() override;

        // Configuration methods
        void SetBrakeMode(constants::BrakeMode mode) override;
        void SetSlowMode(bool slow) override;
        void SetTimestampOffset(units::second_t offset) override;
        void LogModuleStates(wpi::array<frc::SwerveModulePosition, 4> modulePositions) override;
        // frc2::SequentialCommandGroup GetPathfindingPath(std::shared_ptr<pathplanner::PathPlannerPath> m_scorePath, std::shared_ptr<pathplanner::PathPlannerPath> m_retreatPath, std::string coralLevelHeightCommand);

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
