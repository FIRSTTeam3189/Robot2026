#include "util/align/Turret.h"

namespace util::align::turret
{

    frc::Pose2d UpdateTargetWithVelocity(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d robotVelocity, 
                                         units::angular_velocity::radians_per_second_t angularVelocity, frc::Field2d &velocityField, 
                                         bool isPhysics, double articulationAngle)
    {
        Vector2d targetVec(targetPose); // Account for velocity
        double robotSpeed = robotVelocity.Length();

        if (isPhysics) {
            targetVec = targetVec + (robotVelocity * constants::auto_align::kVelocityMultiplier * -1);
        } else {
            targetVec = targetVec + (robotVelocity * FindFuelAirTime(articulationAngle, robotPose) * -1);
        }
        return targetVec.ToPose2d();
        // For FIELD 2D ONLY
        Vector2d robotVec(robotPose);
        frc::Pose2d velocityEnd = (robotVec + robotVelocity).ToPose2d();
        velocityField.SetRobotPose(velocityEnd);
        frc::SmartDashboard::PutData("Robot Velocities/velocityVec", &velocityField);

        return targetPose;
    }

    double FindFuelAirTime(double articulationAngle, frc::Pose2d robotpose) {
        double initialAngle = 90.0 - articulationAngle; 
        return std::sqrt((2.0 / constants::shooter::turret::kGravity) * (GetDistanceToTarget(robotpose).value() * std::tan(initialAngle) - constants::shooter::turret::kHeightDifference)
    );
    }

    // Vector2d GetRotationCompensationVector(frc::Pose2d robotPose, frc::Pose2d targetPose, units::angular_velocity::radians_per_second_t angularVelocity) {
    //     if (angularVelocity < constants::auto_align::kAngularVelocityThreshold) {
    //         return {0.0, 0.0}; 
    //     }
    //     if (frc::DriverStation::GetAlliance())
    //     {
    //         if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
    //         {
    //             if (angularVelocity.value() > 0) {
                    
    //             }
    //         }
    //         else
    //         {

    //         }
    //     }

    //     return {0.0, 0.0}; 
    // }

    frc::Pose2d CalculateTargetPose(frc::Pose2d robotPose)
    {
        frc::Pose2d targetPose(0.0_m, 0.0_m, frc::Rotation2d(0.0_deg));
        if (frc::DriverStation::GetAlliance())
        {
            if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
            {

                if (robotPose.X() < constants::auto_align::kBlueHubPose.X())
                {
                    // scoring here - blue 
                    targetPose = constants::auto_align::kBlueHubPose;
                }
                else
                {    
                    // ferrying here - blue 
                    if (robotPose.Y() < constants::auto_align::kBlueHubPose.Y()) {
                        targetPose = constants::auto_align::kFerryBlueBottomPose; 
                    } else {
                        targetPose = constants::auto_align::kFerryBlueTopPose; 
                    }
                    // targetPose = constants::auto_align::kFerryBluePose;
                    // targetPose = frc::Pose2d(constants::auto_align::kMidpointOnFieldBlue, robotPose.Y(), frc::Rotation2d(0.0_deg)); // rot dont matter
                }
            }
            else
            {
                if (robotPose.X() > constants::auto_align::kRedHubPose.X())
                {
                    // scoring here - red 
                    targetPose = constants::auto_align::kRedHubPose;
                }
                else
                {
                    // ferrying here - red 
                    if (robotPose.Y() < constants::auto_align::kRedHubPose.Y()) {
                        targetPose = constants::auto_align::kFerryRedBottomPose; 
                    } else {
                        targetPose = constants::auto_align::kFerryRedTopPose; 
                    }
                    // targetPose = constants::auto_align::kFerryRedPose;
                    // targetPose = frc::Pose2d(constants::auto_align::kMidpointOnFieldRed, robotPose.Y(), frc::Rotation2d(0.0_deg)); // rot dont matter
                }
            }
        }

        return targetPose;
    }

    units::meter_t GetDistanceToTarget(frc::Pose2d robotPose)
    {
        Vector2d robotToTarget(robotPose, CalculateTargetPose(robotPose));
        return units::meter_t{robotToTarget.Length()};
    }

    // returns angle from 180-0
    double GetTargetAngle(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d turretOffset,
                          frc::Field2d &turretField, frc::Field2d &targetField)
    {

        double robotRotation = robotPose.Rotation().Degrees().value();
        Vector2d robotHeading(robotRotation); // assuming robot Heading = turret default Heading

        // Calculate the offset of the turret based on the robot's heading
        // Effectively set the length of the turretOffsetWithHeading to the magnitude of the turretOffset
        Vector2d turretOffsetWithHeading = robotHeading.Normalize() * turretOffset.Length();
        

        // Get the poses as vectors
        Vector2d turretVec = Vector2d(robotPose) + turretOffsetWithHeading;
        Vector2d targetVec(targetPose);

        Vector2d turretToTarget = targetVec - turretVec;

        double errorAngle = turretToTarget.SignedAngle(robotHeading);

        frc::SmartDashboard::PutNumber("Turret Align/error", errorAngle);

        // This is just converting our raw error angle which is -180 to 180 to -90 to 270.0 
        double targetAngle = std::clamp(errorAngle + 90, -90.0, 270.0);

        // unwrap all negative angles and make em positive 
        if (targetAngle < 0)
        {
            targetAngle += 360;
        }
        // all the code above gets the error angle and does some formatting to make it a nice 0-360 degree target range

        frc::SmartDashboard::PutNumber("Turret Align/before adjusting", targetAngle);
        // check if the targetAngle is within an unreachable range of angles, 310-320 exclusive and clamp to nearest limit 
        if (targetAngle > constants::shooter::turret::kMaximumAngle.value() 
            && targetAngle < constants::shooter::turret::kMinimumAnglePositive.value())
        {
            // check if the targetAngle is closer to 310 than 320
            if (std::abs(targetAngle - constants::shooter::turret::kMaximumAngle.value())
                < std::abs(targetAngle - constants::shooter::turret::kMinimumAnglePositive.value()))
            {
                targetAngle = constants::shooter::turret::kMaximumAngle.value();
            }
            else
            {
                targetAngle = constants::shooter::turret::kMinimumAngleNegative.value();
            }
        }
        else if (targetAngle >= constants::shooter::turret::kMaximumAngle.value())
        {
            targetAngle = targetAngle - 360; // convert any angle past 310 to negative angle because our realistic range is -40 to 310
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