// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PhotonVision.h"

PhotonVision::PhotonVision(PoseEstimatorHelper *helper, std::string camera_name, units::meter_t xOffset, units::meter_t yOffset, frc::Rotation2d yawOffset, frc::Rotation2d pitchOffset, units::meter_t heightOffset) :
    m_poseEstimator(helper),
    m_camera(camera_name),
    m_aprilTagFieldLayout(frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltWelded)),

    m_cameraToRobotTransform(xOffset, yOffset, yawOffset),
    m_timestampOffset(0.0_s),

    m_yawOffset(yawOffset),
    m_heightOffset(heightOffset),
    m_pitchOffset(pitchOffset),

    m_cameraName(camera_name) {
    (void)VisionConstants::kSyncBytes[0];
    m_folderName = "Photonvision/" + camera_name + "/";
}

void PhotonVision::SetTimestampOffset(units::second_t offset) {
    m_timestampOffset = offset;
}

// This method will be called once per scheduler run
void PhotonVision::Periodic() {
    // Query the latest result from PhotonVision
    photon::PhotonPipelineResult result = m_camera.GetLatestResult();

    // frc::SmartDashboard::PutNumber("Vision/timestampstamp", result.GetTimestamp().value());

    // Get the current best target.
    photon::PhotonTrackedTarget target = result.GetBestTarget();

    // Calculate robot's field relative pose
    if (m_aprilTagFieldLayout.GetTagPose(target.GetFiducialId()).has_value()) {
        frc::SmartDashboard::PutBoolean(m_folderName + "Detected", true);
        frc::Transform3d transform3d = target.GetBestCameraToTarget();
        frc::Transform2d transform2d = frc::Transform2d{transform3d.X(), transform3d.Y(), transform3d.Rotation().ToRotation2d()};
        
        frc::Pose3d tagPose3d = m_aprilTagFieldLayout.GetTagPose(target.GetFiducialId()).value();
        frc::Pose2d tagPose = tagPose3d.ToPose2d();
        frc::Pose2d cameraPose = photon::PhotonUtils::EstimateFieldToCamera(transform2d, m_aprilTagFieldLayout.GetTagPose(target.GetFiducialId()).value().ToPose2d());
        // frc::Pose2d visionEstimatedRobotPose = cameraPose.TransformBy(m_cameraToRobotTransform);
        frc::Pose2d visionEstimatedRobotPose = photon::PhotonUtils::EstimateFieldToRobot(
            transform2d, 
            m_aprilTagFieldLayout.GetTagPose(target.GetFiducialId()).value().ToPose2d(), 
            m_cameraToRobotTransform);

        if (frc::DriverStation::GetAlliance()) {
            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) {
                visionEstimatedRobotPose = frc::Pose2d(visionEstimatedRobotPose.X() + 0.439_m, visionEstimatedRobotPose.Y(), visionEstimatedRobotPose.Rotation()); 
            } else {
                visionEstimatedRobotPose = frc::Pose2d(visionEstimatedRobotPose.X() - 0.439_m, visionEstimatedRobotPose.Y(), visionEstimatedRobotPose.Rotation()); 
            }
        }

        // frc::Pose2d visionEstimatedRobotPose = photon::PhotonUtils::EstimateFieldToRobot(
        //     m_heightOffset, // camera height
        //     tagPose3d.Z(),// tag height

        //     m_pitchOffset.Radians(), // camera pitch
        //     tagPose3d.Rotation().Y(), // target pitch

        //     tagPose3d.Rotation().Z(), // tag yaw
        //     m_poseEstimator->GetEstimatedPose().Rotation(), // gyro angle

        //     m_aprilTagFieldLayout.GetTagPose(target.GetFiducialId()).value().ToPose2d(), // field to target
        //     m_cameraToRobotTransform // camera to robot
        // );
        
        double xDistance = transform2d.X().value();
        double yDistance = transform2d.Y().value();

        units::second_t timestamp = result.GetTimestamp();
        frc::SmartDashboard::PutNumber(m_folderName + "cameraToTarget x", xDistance);
        frc::SmartDashboard::PutNumber(m_folderName + "cameraToTarget y", yDistance);
        frc::SmartDashboard::PutNumber(m_folderName + "robot x", visionEstimatedRobotPose.X().value());
        frc::SmartDashboard::PutNumber(m_folderName + "robot y", visionEstimatedRobotPose.Y().value());
        frc::SmartDashboard::PutNumber(m_folderName + "timestamp", timestamp.value());
        frc::SmartDashboard::PutNumber(m_folderName + "Tag ID", target.GetFiducialId());
        frc::SmartDashboard::PutNumber(m_folderName + "tag x", tagPose.X().value());
        frc::SmartDashboard::PutNumber(m_folderName + "tag y", tagPose.Y().value());
        
        double tagDistance = sqrt(pow(xDistance, 2.0) + pow(yDistance, 2.0));
        // Calculate vision std devs based on tag distance
        double stdDevDistanceCompensation = tagDistance * VisionConstants::kPhotonVisionStdDevPerMeter;
        wpi::array<double, 3U> baseVisionStdDevs = VisionConstants::kPhotonVisionTrustCoefficients;
        wpi::array<double, 3U> distanceCompensatedStdDevs = wpi::array<double, 3>{
            baseVisionStdDevs[0] + stdDevDistanceCompensation,
            baseVisionStdDevs[1] + stdDevDistanceCompensation,
            baseVisionStdDevs[2] + stdDevDistanceCompensation};

        // frc::SmartDashboard::PutNumber("Vision/trust x", distanceCompensatedStdDevs.at(0));
        // frc::SmartDashboard::PutNumber("Vision/trust y", distanceCompensatedStdDevs.at(1));

        // Only add measurements if the ambiguity is low and tag is closer than max dist (about 3 meters)
        double ambiguity = target.poseAmbiguity;
        auto currentPose = m_poseEstimator->GetEstimatedPose();

        // The distance between the current pose and the vision estimated pose
        double visionEstimatedDistance = sqrt(pow((currentPose.X().value() - visionEstimatedRobotPose.X().value()), 2.0) + pow(((currentPose.Y().value() - visionEstimatedRobotPose.Y().value())), 2.0));
        
        // ? 3 if statements oooooooOOOOoooh
        // Add offset to timestamp to set to rust vision's epoch
        if (ambiguity < VisionConstants::kMaxAmbiguity && tagDistance < VisionConstants::kMaxDistance.value()) {
            if (visionEstimatedDistance < VisionConstants::kUseVisionMeasurementThreshold){
                if (m_cameraName == VisionConstants::kPhotonTurretFrontCamera) {
                    m_poseEstimator->AddVisionMeasurement(visionEstimatedRobotPose, timestamp, distanceCompensatedStdDevs, VisionCamera::PhotonVision1);
                } else if (m_cameraName == VisionConstants::kPhotonCamera3) {
                    m_poseEstimator->AddVisionMeasurement(visionEstimatedRobotPose, timestamp, distanceCompensatedStdDevs, VisionCamera::PhotonVision2);     
                }
            
            } else {
                frc::SmartDashboard::PutBoolean(m_folderName + "Detected", false);
            }
        }
    }
}