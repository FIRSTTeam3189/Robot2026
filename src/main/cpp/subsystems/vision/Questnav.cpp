// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/vision/Questnav.h"

#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Rotation3d.h>

namespace subsystems::vision {
Questnav::Questnav(PoseEstimatorHelper *poseEstimator) :
  m_poseEstimator(poseEstimator) {}

// This method will be called once per scheduler run
void Questnav::Periodic() {
    m_questNav.CommandPeriodic();

  frc::SmartDashboard::PutBoolean("QuestNav/Enabled", m_isQuest);
  frc::SmartDashboard::PutBoolean("QuestNav/Connected", m_questNav.IsConnected());
  frc::SmartDashboard::PutBoolean("QuestNav/Tracking", m_questNav.IsTracking());

  // questPose3d is the Quest headset's own field-relative pose, not the
  // robot's -- undo the robot-center-to-Quest mounting offset to recover the
  // robot's pose before fusing it into the pose estimator.
  frc::Transform3d questMountOffset{
      constants::vision::kQuestMountXOffset,
      constants::vision::kQuestMountYOffset,
      constants::vision::kQuestMountZOffset,
      frc::Rotation3d{constants::vision::kQuestMountRollOffset,
                       constants::vision::kQuestMountPitchOffset,
                       constants::vision::kQuestMountYawOffset}};

  for (const auto& frame : m_questNav.GetAllUnreadPoseFrames()) {
    m_latestFrame = frame;
    frc::Pose3d robotPose3d = m_latestFrame.questPose3d.TransformBy(questMountOffset.Inverse());
    if (m_isQuest) {
    m_poseEstimator->AddVisionMeasurement(robotPose3d.ToPose2d(), units::second_t{m_latestFrame.dataTimestamp}, constants::vision::kQuestnavTrustCoefficients, VisionCamera::Questnav);
    }
  }
  frc::SmartDashboard::PutNumber("QuestNav/Latency", m_questNav.GetLatency().value());
  frc::SmartDashboard::PutNumber("QuestNav/Battery", m_questNav.GetBatteryPercent().value_or(-1));
  frc::SmartDashboard::PutNumber("QuestNav/X", m_latestFrame.questPose3d.X().value());
  frc::SmartDashboard::PutNumber("QuestNav/Y", m_latestFrame.questPose3d.Y().value());
  frc::SmartDashboard::PutNumber("QuestNav/Timestamp", m_latestFrame.dataTimestamp);
  frc::SmartDashboard::PutNumber("QuestNav/FrameCount", m_questNav.GetFrameCount().value_or(-1));
}

void Questnav::ToggleQuestnav() {
  m_isQuest = !m_isQuest;
}

void Questnav::SetPose(const frc::Pose3d& pose) {
  m_questNav.SetPose(pose);
}

bool Questnav::IsConnected() {
  return m_questNav.IsConnected();
}

bool Questnav::IsTracking() {
  return m_questNav.IsTracking();
}
}