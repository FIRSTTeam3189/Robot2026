#include "util/alignUtils/AutoAlign.h"

namespace AutoAlign {

    frc::Pose2d GetClosestRobotPose(frc::Pose2d robotPose) {

        double lowestDist = 1000; // Huge number just as a value

        frc::Pose2d closestPoint;

        double robotX = robotPose.X().value();
        double robotY = robotPose.Y().value(); 

        for (int i = 0; i < AutoAlignConstants::kNumSetpoints; i++){

            // Get setpoint
            frc::Pose2d setpointPose = AutoAlignConstants::kBlueKnownSetPoints[i];  // just as a default

            if (frc::DriverStation::GetAlliance()) {
                if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue) {
                    setpointPose = AutoAlignConstants::kBlueKnownSetPoints[i];
                } else {
                    setpointPose = AutoAlignConstants::kRedKnownSetPoints[i];
                }
            }

            double dist = std::hypot(robotX - setpointPose.X().value(), robotY - setpointPose.Y().value()); //  distance formula for that code
            if (dist < lowestDist){
                closestPoint = setpointPose; 
                lowestDist = dist;
            }

        }

        // now that the loop is over we got the lowestDist and the setpoint
        return closestPoint;
    }

    

    frc2::CommandPtr AutoAlignToPose(SwerveDrive* swerveDrive, pathplanner::PathConstraints constraints) {

        frc::Pose2d robotPose = swerveDrive->GetEstimatedPose();
        frc::Pose2d targetSetpointPose = GetClosestRobotPose(robotPose);

        frc::SmartDashboard::PutNumber("Backup Pathfinding/x", targetSetpointPose.X().value());
        frc::SmartDashboard::PutNumber("Backup Pathfinding/y", targetSetpointPose.Y().value());
        frc::SmartDashboard::PutNumber("Backup Pathfinding/theta", targetSetpointPose.Rotation().Degrees().value());

        auto pathFindCmd = pathplanner::AutoBuilder::pathfindToPose(targetSetpointPose, constraints); 
        return std::move(pathFindCmd);
    }  



    int GetIndexOfPathfindingPose(frc::Pose2d pathfindingPose) {
        frc::Pose2d poses[AutoAlignConstants::kNumSetpoints] = {}; 

        if (frc::DriverStation::GetAlliance()) {
            if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue) {
                memcpy(poses, AutoAlignConstants::kBlueKnownSetPoints, sizeof(AutoAlignConstants::kBlueKnownSetPoints)); 
            } else {
                memcpy(poses, AutoAlignConstants::kRedKnownSetPoints, sizeof(AutoAlignConstants::kRedKnownSetPoints)); 
            }
        }

        for (int i = 0; i < AutoAlignConstants::kNumSetpoints; i++) {
            if (poses[i] == pathfindingPose) {
                return i; 
            } 
        }

        return 0; 
    } 



};