#pragma once

#include <frc/smartdashboard/Field2d.h>

// #include "simulation/drivesims/AbstractDriveTrainSimulation.h"
#include "simulation/drive/SwerveDriveSimulation.h"
#include "simulation/SimulatedArena.h"
#include "subsystems/vision/PoseEstimatorHelper.h"

#include "subsystems/swerve/Drive.h"


namespace subsystems::swerve {
    class SimDrive : public Drive {
    public:
        SimDrive(subsystems::vision::PoseEstimatorHelper* poseEstimator);
        
        // simulation::drivesims::configs::DriveTrainSimulationConfig GetDriveTrainConfig();

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
        units::degree_t GetNormalizedYaw() override;
        void SetBrakeMode(constants::BrakeMode mode) override;
        void SetPose(frc::Pose2d pose, bool justRotation) override;
        void SetAutoPose(frc::Pose2d pose, bool justRotation) override;
        frc::Pose2d GetEstimatedPose() override;

        // Sensor methods
        frc::ChassisSpeeds GetRobotRelativeSpeeds() override;

        // Configuration methods
        void SetSlowMode(bool slow) override;
        void SetTimestampOffset(units::second_t offset) override;
        void LogModuleStates(wpi::array<frc::SwerveModulePosition, 4> modulePositions) override;
        // frc2::SequentialCommandGroup GetPathfindingPath(std::shared_ptr<pathplanner::PathPlannerPath> m_scorePath, std::shared_ptr<pathplanner::PathPlannerPath> m_retreatPath, std::string coralLevelHeightCommand);

        /**
         * Will be called periodically whenever the CommandScheduler runs.
         */
        void SimulationPeriodic() override;

    private:
        frc::Field2d m_field;
        simulation::drive::SwerveDriveSimulation m_simulatedDrive;
        subsystems::vision::PoseEstimatorHelper *m_poseHelper;
        units::second_t m_timestampOffset;
        bool m_slowMode{ false }; // NOTE: Is this needed for simulation anyway?
    };
}