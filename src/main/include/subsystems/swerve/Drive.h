#pragma once

#include <units/angle.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <pathplanner/lib/auto/AutoBuilder.h>

#include "constants/Automatic.h"
#include "constants/Global.h"
#include "constants/Vision.h"

#include "subsystems/swerve/State.h"


namespace subsystems::swerve {
    /**
     * The interface class for a drivetrain.
     * Anything that inherits this class must implement
     * all of the pure virtual methods (methods that
     * start with virtual and end in = 0) or else the
     * program will not compile.
     */
    class Drive : public frc2::SubsystemBase {
    public:
        virtual ~Drive() = default;

        // Non-interface methods. The default implementations
        // of these methods are defined in the base class Drive.cpp,
        // but you can override these implementations in a subclass
        // if needed
		virtual void ConfigurePathPlannerAutoBuilder();

        // Motion methods
        virtual void Stop() = 0;
        virtual void Operate(
          units::meters_per_second_t xSpeed,
          units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
          bool fieldRelative,
          frc::Translation2d centerOfRotation
        ) = 0;
        virtual void DriveRobotRelative(frc::ChassisSpeeds speeds) = 0;
        virtual void SetModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates) = 0;

        // Pose/odometry methods
        virtual void UpdateEstimator() = 0;
        virtual units::degree_t GetNormalizedYaw() = 0;
        virtual void SetBrakeMode(constants::BrakeMode mode) = 0;
        virtual void SetPose(frc::Pose2d pose, bool justRotation) = 0;
        virtual void SetAutoPose(frc::Pose2d pose, bool justRotation) = 0;
        virtual frc::Pose2d GetEstimatedPose() = 0;

        // Sensor methods
        virtual frc::ChassisSpeeds GetRobotRelativeSpeeds() = 0;

        // Configuration methods
        virtual void SetSlowMode(bool slow) = 0;
        virtual void SetTimestampOffset(units::second_t offset) = 0;
        virtual void LogModuleStates(wpi::array<frc::SwerveModulePosition, 4> modulePositions) = 0;
        // frc2::SequentialCommandGroup GetPathfindingPath(std::shared_ptr<pathplanner::PathPlannerPath> m_scorePath, std::shared_ptr<pathplanner::PathPlannerPath> m_retreatPath, std::string coralLevelHeightCommand);
	
		private:
		
	};
}