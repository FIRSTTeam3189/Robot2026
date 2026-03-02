#include "util/align/Turret.h"


namespace util::align::turret {

    frc::Pose2d UpdateTargetWithVelocity(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d robotVelocity,
                                         frc::Field2d &velocityField) {
        Vector2d targetVec(targetPose); // Account for velocity 
        double robotSpeed = robotVelocity.Length(); 
        // if (robotSpeed > constants::auto_align::kMinimumVelocityMultiplier * SwerveModuleConstants::kMaxSpeed.value()) {
        //     // Needs to multiply magnitude by a linear interpolation based on ball air time 
        //     targetVec = targetVec + (robotVelocity * constants::auto_align::kVelocityMultiplier * -1); 
        //     return targetVec.ToPose2d(); 
        // }
        targetVec = targetVec + (robotVelocity * constants::auto_align::kVelocityMultiplier * -1); 
        return targetVec.ToPose2d(); 
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

                if (robotPose.X() < constants::auto_align::kBlueHubPose.X()){
                    targetPose = constants::auto_align::kBlueHubPose;
                } else {
                    targetPose = constants::auto_align::kFerryBluePose; 
                    // targetPose = frc::Pose2d(constants::auto_align::kMidpointOnFieldBlue, robotPose.Y(), frc::Rotation2d(0.0_deg)); // rot dont matter
                }

            } else {
                if (robotPose.X() > constants::auto_align::kRedHubPose.X()){
                    targetPose = constants::auto_align::kRedHubPose;
                } else {
                    targetPose = constants::auto_align::kFerryRedPose;
                    // targetPose = frc::Pose2d(constants::auto_align::kMidpointOnFieldRed, robotPose.Y(), frc::Rotation2d(0.0_deg)); // rot dont matter
                }
            }
        }

        return targetPose;
    }

    units::meter_t GetDistanceToTarget(frc::Pose2d robotPose) {
        Vector2d robotToTarget(robotPose, CalculateTargetPose(robotPose)); 
        return units::meter_t {robotToTarget.Length()};
    }

    // returns angle from 180-0
    double GetTargetAngle(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d turretOffset, 
                          frc::Field2d &turretField, frc::Field2d &targetField) {

        double robotRotation = robotPose.Rotation().Degrees().value(); 
        Vector2d robotHeading(robotRotation); // assuming robot Heading = turret default Heading 

        // Calculate the offset of the turret based on the robot's heading 
        // Effectively set the length of the turretOffsetWithHeading to the magnitude of the turretOffset 
        Vector2d turretOffsetWithHeading = robotHeading.Normalize() * turretOffset.Length();;

        // Get the poses as vectors 
        Vector2d turretVec = Vector2d(robotPose) + turretOffsetWithHeading; 
        Vector2d targetVec(targetPose); 

        Vector2d turretToTarget = targetVec - turretVec; 

        double errorAngle = turretToTarget.SignedAngle(robotHeading);

        frc::SmartDashboard::PutNumber("Turret Align/error", errorAngle); 

        double targetAngle = std::clamp(errorAngle + 90, -90.0, 270.0);
        
        if (targetAngle < 0) {
            targetAngle += 360; 
        }

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


};