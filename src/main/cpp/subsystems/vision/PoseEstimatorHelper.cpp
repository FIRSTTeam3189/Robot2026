

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/vision/PoseEstimatorHelper.h"

namespace subsystems::vision
{

    PoseEstimatorHelper::PoseEstimatorHelper()
    {
    }

    void PoseEstimatorHelper::SetPoseEstimator(frc::SwerveDrivePoseEstimator<4> *poseEstimator)
    {
        // set the member variable to the pose estimator variable
        m_poseEstimator = poseEstimator;
    }

    void PoseEstimatorHelper::UpdatePoseEstimator(units::second_t timestamp, wpi::array<frc::SwerveModulePosition, 4U> modulePositions, frc::Rotation2d rotation)
    {
        // update the pose estimator with the rotation found by the encoders and the module positions
        m_poseEstimator->UpdateWithTime(timestamp, rotation, modulePositions);
        UpdateRotation(rotation);
    }

    void PoseEstimatorHelper::SetActivePath(std::vector<frc::Pose2d> poses)
    {
        m_autoField.GetObject("path")->SetPoses(poses);
    }

    void PoseEstimatorHelper::SetCurrentAutoPose(frc::Pose2d pose)
    {
        m_autoField.SetRobotPose(pose);
    }

    void PoseEstimatorHelper::SetTargetAutoPose(frc::Pose2d pose)
    {
        m_autoField.GetObject("target pose")->SetPose(pose);
    }

    frc::Pose2d PoseEstimatorHelper::GetEstimatedPose()
    {
        return m_poseEstimator->GetEstimatedPosition();
    }

    void PoseEstimatorHelper::UpdateRotation(frc::Rotation2d rotation)
    {
        m_rotation = rotation;
    }

    void PoseEstimatorHelper::ResetPose(frc::Rotation2d rotation, wpi::array<frc::SwerveModulePosition, 4> modulePositions, frc::Pose2d pose)
    {
        // reset the positions based on newest values
        m_poseEstimator->ResetPosition(rotation, modulePositions, pose);
    }



    void PoseEstimatorHelper::AddVisionMeasurement(frc::Pose2d pose, units::second_t timestamp, wpi::array<double, 3> visionMeasurementStdDevs, VisionCamera camera)
    {
            m_poseEstimator->SetVisionMeasurementStdDevs(visionMeasurementStdDevs);

        // Currently setting pose rotation to estimated rotation rather than vision-deduced one
        // Gyro is already fairly accurate and prevents noisy rotation data for driving/auto
        // Sets pose angle to 0
        pose = pose.TransformBy(frc::Transform2d(0.0_m, 0.0_m, -pose.Rotation()));
        // Then sets pose angle to estimated pose angle
        pose = pose.TransformBy(frc::Transform2d(0.0_m, 0.0_m, m_poseEstimator->SampleAt(timestamp).value_or(GetEstimatedPose()).Rotation()));
        // basically this makes it so that it uses what the rotation of the pose was at the timestamp rather than what it currently is
        // so it wont accidentally set the rotation of a pose 100 ms ago to the rotation of the current pose (also returns the current pose if sampleat() returns nothing)

        // Add vision estimated poses to corresponding fields
        switch (camera)
        {
        case (VisionCamera::FrontCamera):
            m_photonVisionFront.SetRobotPose(pose);
            frc::SmartDashboard::PutData("Vision pose/Front Camera", &m_photonVisionFrontPose);
            break;
        case (VisionCamera::RightCamera):
            m_photonVisionRight.SetRobotPose(pose);
            frc::SmartDashboard::PutData("Vision pose/Right Camera", &m_photonVisionRightPose);
            break;
        case (VisionCamera::LeftCamera):
            m_photonVisionLeft.SetRobotPose(pose);
            frc::SmartDashboard::PutData("Vision pose/Left Camera", &m_photonVisionLeftPose);
            break;
        }

        m_poseEstimator->AddVisionMeasurement(pose, timestamp);
    }

    void PoseEstimatorHelper::Periodic()
    {
        m_estimatedPose.SetRobotPose(GetEstimatedPose());

        frc::SmartDashboard::PutData("Estimated pose", &m_estimatedPose);
        frc::SmartDashboard::PutData("Auto field", &m_autoField);
    }
}
