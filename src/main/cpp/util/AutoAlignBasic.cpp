#include "util/AutoAlignBasic.h"
#include <math.h>


namespace AutoAlign {

    
    frc::Pose2d getClosestRobotPose(double robotX, double robotY) {
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

        

    frc2::CommandPtr autoAlignToPose(frc::Pose2d robotPose, pathplanner::PathConstraints constraints) {
        frc::Pose2d targetSetpointPose = getClosestRobotPose(robotPose.X().value(), robotPose.Y().value());
        auto pathFindCmd = pathplanner::AutoBuilder::pathfindToPose(targetSetpointPose, constraints); 
        return std::move(pathFindCmd);
    }  
};

