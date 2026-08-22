// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "questnav/QuestNav.h"
#include "constants/Vision.h"
#include "subsystems/vision/PoseEstimatorHelper.h"

namespace subsystems::vision {
class Questnav : public frc2::SubsystemBase {
 public:
  explicit Questnav(PoseEstimatorHelper *poseEstimator);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SetPose(const frc::Pose3d& pose);
  bool IsConnected();
  bool IsTracking();
  void ToggleQuestnav();

 private:
  questnav::QuestNav m_questNav;
  questnav::PoseFrame m_latestFrame;
  PoseEstimatorHelper *m_poseEstimator;
  bool m_isQuest = true;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
}
