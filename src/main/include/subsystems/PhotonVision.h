// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/PoseEstimatorHelper.h"
#include "Constants/VisionConstants.h"

#include <photon/PhotonCamera.h>
#include <photon/PhotonUtils.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <photon/targeting/PhotonTrackedTarget.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Transform2d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

class PhotonVision : public frc2::SubsystemBase {
 public:
  PhotonVision(PoseEstimatorHelper *helper, std::string camera_name, units::meter_t xOffset, units::meter_t yOffset, frc::Rotation2d yawOffset, frc::Rotation2d pitchOffset, units::meter_t heightOffset);
  void SetTimestampOffset(units::second_t offset);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  PoseEstimatorHelper *m_poseEstimator;
  photon::PhotonCamera m_camera;
  frc::AprilTagFieldLayout m_aprilTagFieldLayout;
  frc::Transform2d m_cameraToRobotTransform;
  units::second_t m_timestampOffset;

  std::string m_cameraName;
  std::string m_folderName;

  frc::Rotation2d m_yawOffset; 
  frc::Rotation2d m_pitchOffset;
  units::meter_t m_heightOffset;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};