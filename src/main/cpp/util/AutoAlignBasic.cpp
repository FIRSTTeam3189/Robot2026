#include "util/AutoAlignBasic.h"

namespace AutoAlign {

    
    frc::Pose2d GetClosestRobotPose(double robotX, double robotY) {
        double lowestDist = 1000; // Huge number just as a value
        frc::Pose2d closestPoint;

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
    
    frc2::CommandPtr AutoAlignToPose(frc::Pose2d robotPose, pathplanner::PathConstraints constraints) {
        frc::Pose2d targetSetpointPose = GetClosestRobotPose(robotPose.X().value(), robotPose.Y().value());
        auto pathFindCmd = pathplanner::AutoBuilder::pathfindToPose(targetSetpointPose, constraints); 
        return std::move(pathFindCmd);
    }  


    // // Returns angle from 0 to 180 
    // double GetTargetAngle(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d turretOffset) {
    //     double robotAngle = robotPose.Rotation().Degrees().value(); // Get robot angle
    //     Vector2d robotVec(robotPose); // Get robot's heading 
    //     robotVec = robotVec.Rotate(-robotAngle); // Rotate robot's heading to align with normal 

    //     Vector2d turretVec = robotVec + turretOffset; // Calculate turret's vector based on robot vector and offset
    //     Vector2d targetVec(targetPose); // Get target's position as a vector for calculation 

    //     Vector2d turretToTarget = (targetVec - turretVec).Normalize(); // Get vector from turret to target

    //     // Get the angle 
    //     return -turretToTarget.Angle() + 90; // + 90 Adjust from -90-90 to 0-180 
    // }



};

