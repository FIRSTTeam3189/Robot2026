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

    Vector2d GetTurretToTargetVec(frc::Pose2d robotPose, Vector2d turretOffset, frc::Pose2d targetPose) {
        double robotRotation = robotPose.Rotation().Degrees().value(); 
        Vector2d robotHeading(robotRotation); // assuming robot Heading = turret default Heading 

        // Calculate the offset of the turret based on the robot's heading 
        // Effectively set the length of the turretOffsetWithHeading to the magnitude of the turretOffset 
        Vector2d turretOffsetWithHeading = robotHeading.Normalize() * turretOffset.Length();;

        Vector2d turretVec = Vector2d(robotPose) + turretOffsetWithHeading; 
        Vector2d targetVec(targetPose); 

        return targetVec - turretVec; 
    }

    double GetRawAngle(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d turretOffset) {
        double robotRotation = robotPose.Rotation().Degrees().value(); 
        Vector2d robotHeading(robotRotation); // assuming robot Heading = turret default Heading 

        Vector2d turretToTarget = GetTurretToTargetVec(robotPose, turretOffset, targetPose);

        triple errorAngle = turretToTarget.SignedAngle(robotHeading);

        frc::SmartDashboard::PutNumber("Turret Align/error", errorAngle); 

        double targetAngle = std::clamp(errorAngle + 90, -90.0, 270.0);
        
        if (targetAngle < 0) {
            targetAngle += 360; 
        }

        return targetAngle; 
    }

    // returns angle from 180-0
    double GetTargetAngle(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d turretOffset, 
                          frc::Field2d &turretField, frc::Field2d &targetField) {

        double robotRotation = robotPose.Rotation().Degrees().value(); 

        Vector2d turretToTarget = GetTurretToTargetVec(robotPose, turretOffset, targetPose); 

        double targetAngle = GetRawAngle(robotPose, targetPose, turretOffset); 

        frc::SmartDashboard::PutNumber("Turret Align/before adjusting", targetAngle); 
        // check if the targetAngle is within an unreachable range of angles, 225-315 exclusive
        if (targetAngle > 225.0 && targetAngle < 315.0) {
            // check if the targetAngle is closer to 225 than 270 
            if (std::abs(targetAngle - 225.0) < std::abs(targetAngle - 315.0)) {
                targetAngle = 225.0; 
            } else {
                targetAngle = -45.0; 
            }
        } else if (targetAngle >= 315.0) {
            targetAngle = targetAngle - 360; 
        }

        frc::SmartDashboard::PutNumber("Turret Align/after adjusting", targetAngle); 

        frc::Pose2d endTargetPose = frc::Pose2d{units::meter_t{turretToTarget.x} + robotPose.X(), units::meter_t{turretToTarget.y} + robotPose.Y(), 
            frc::Rotation2d(0.0_deg)};

        targetField.SetRobotPose(endTargetPose); 

        frc::SmartDashboard::PutData("Auto Align/End Target Pose", &targetField); 

        frc::Pose2d turretPoseForViz = frc::Pose2d{robotPose.X(), robotPose.Y(), frc::Rotation2d(units::degree_t(-targetAngle + 90 + robotRotation))};
        turretField.SetRobotPose(turretPoseForViz);
        
        frc::SmartDashboard::PutData("Auto Align/Turret Pose", &turretField);

        return targetAngle; // where 90 is the turret's default position  
    }

    bool InvalidTargetAngle(frc::Pose2d robotPose, Vector2d turretOffset) {
        double targetAngle = GetRawAngle(robotPose, CalculateTargetPose(robotPose), turretOffset);
        frc::SmartDashboard::PutNumber("turret desired angle invalid", targetAngle);
        frc::SmartDashboard::PutBoolean("Is turret valid?", !(targetAngle > 270 && targetAngle < 315));
        return targetAngle > 223 && targetAngle < 315;
    }

    // triple GetRobotDesiredAngle(frc::Pose2d robotPose) {
    //     double robotAngle = robotPose.Rotation().Degrees().value();
    //     Vector2d robotHeading(robotAngle);
    //     Vector2d robotVec(robotPose);
    //     Vector2d targetVec = CalculateTargetPose(robotPose);
    //     // Vector2d robotToTarget = targetVec - robotVec;
    //     Vector2d robotToTarget = targetVec;
    //     return robotToTarget.SignedAngle(robotHeading);
    // }

    triple GetRobotDesiredAngle(frc::Pose2d robotPose) {
        double poseX = CalculateTargetPose(robotPose).X().value() - robotPose.X().value();
        double poseY = CalculateTargetPose(robotPose).Y().value() - robotPose.Y().value();

        double angle = std::atan2(poseY, poseX);

        double desiredAngle = angle * (180/ M_PI);

        if (TurretAlign::InvalidTargetAngle(robotPose, AutoAlignConstants::kTurretOffset)) {
            return desiredAngle;
        } else {
            return robotPose.Rotation().Degrees().value();
        }
        }
    }
;