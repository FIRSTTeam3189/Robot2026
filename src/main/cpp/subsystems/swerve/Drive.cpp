#include <frc/DriverStation.h>

#include "subsystems/swerve/Drive.h"


namespace subsystems::swerve {
    void Drive::ConfigurePathPlannerAutoBuilder() {
        pathplanner::RobotConfig guiConfig = pathplanner::RobotConfig::fromGUISettings();

        pathplanner::AutoBuilder::configure(
            [this](){ return GetEstimatedPose(); }, // Robot pose supplier
            [this](frc::Pose2d pose){ SetAutoPose(pose, false); }, // Method to reset odometry (will be called if your auto has a starting pose)
            [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            [this](frc::ChassisSpeeds speeds){ DriveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            std::make_shared<pathplanner::PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
                pathplanner::PIDConstants(constants::automatic::kPTranslationAuto, constants::automatic::kITranslationAuto, constants::automatic::kDTranslationAuto), // Translation PID Constants
                pathplanner::PIDConstants(constants::automatic::kPRotationAuto, constants::automatic::kIRotationAuto, constants::automatic::kDRotationAuto) // Rotation PID Constants
            ),
            // m_autoRobotConfig,
            guiConfig,
            []() {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    
                std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();
                if (alliance) {
                    return alliance.value() == frc::DriverStation::Alliance::kRed;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }
}