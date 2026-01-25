#include "util/alignUtils/TurretAlign.h"


namespace TurretAlign {

    frc::Pose2d UpdateTargetWithVelocity(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d robotVelocity,
                                         frc::Field2d &velocityField) {
        Vector2d targetVec(targetPose); // Account for velocity 
        double robotSpeed = robotVelocity.Length(); 
        if (robotSpeed > AutoAlignConstants::kMinimumVelocityMultiplier * SwerveModuleConstants::kMaxSpeed.value()) {
            // Needs to multiply magnitude by a linear interpolation based on ball air time 
            targetVec = targetVec + (robotVelocity * AutoAlignConstants::kVelocityMultiplier * -1); 
            return targetVec.ToPose2d(); 
        }

        // For FIELD 2D ONLY 
        Vector2d robotVec(robotPose); 
        frc::Pose2d velocityEnd = (robotVec + robotVelocity).ToPose2d(); 
        velocityField.SetRobotPose(velocityEnd); 
        frc::SmartDashboard::PutData("Robot Velocities/velocityVec", &velocityField); 

        return targetPose; 
    }
    

    frc::Pose2d CalculateTargetPose(frc::Pose2d robotPose) {
        frc::Pose2d targetPose(0.0_m, 0.0_m, frc::Rotation2d(0.0_deg));
        if (frc::DriverStation::GetAlliance()) {
            if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue) {

                if (robotPose.X() < AutoAlignConstants::kBlueHubPose.X()){
                    targetPose = AutoAlignConstants::kBlueHubPose;
                } else {
                    targetPose = frc::Pose2d(AutoAlignConstants::kMidpointOnFieldBlue, robotPose.Y(), frc::Rotation2d(0.0_deg)); // rot dont matter
                }

            } else {
                if (robotPose.X() > AutoAlignConstants::kRedHubPose.X()){
                    targetPose = AutoAlignConstants::kRedHubPose;
                } else {
                    targetPose = frc::Pose2d(AutoAlignConstants::kMidpointOnFieldRed, robotPose.Y(), frc::Rotation2d(0.0_deg)); // rot dont matter
                }
            }
        }

        return targetPose;
    }

    // returns angle from 180-0
    double GetTargetAngle(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d turretOffset, 
                          frc::Field2d &turretField, frc::Field2d &targetField) {

        double robotRotation = robotPose.Rotation().Degrees().value(); 
        Vector2d robotHeading(robotRotation); // assuming robot Heading = turret default Heading 

        // Get the poses as vectors 
        Vector2d turretVec = Vector2d(robotPose) + turretOffset; 
        Vector2d targetVec(targetPose); 

        Vector2d turretToTarget = targetVec - turretVec; 

        double errorAngle = turretToTarget.SignedAngle(robotHeading);

        double targetAngle = std::clamp(errorAngle + 90, 0.0, 180.0);

        frc::Pose2d endTargetPose = frc::Pose2d{units::meter_t{turretToTarget.x} + robotPose.X(), units::meter_t{turretToTarget.y} + robotPose.Y(), 
            frc::Rotation2d(0.0_deg)};


        targetField.SetRobotPose(endTargetPose); 

        frc::SmartDashboard::PutData("Auto Align/End Target Pose", &targetField); 

        frc::Pose2d turretPoseForViz = frc::Pose2d{robotPose.X(), robotPose.Y(), frc::Rotation2d(units::degree_t(-targetAngle + 90 + robotRotation))};
        turretField.SetRobotPose(turretPoseForViz);
        
        frc::SmartDashboard::PutData("Auto Align/Turret Pose", &turretField);

        return targetAngle; // where 90 is the turret's default position  
    }


};