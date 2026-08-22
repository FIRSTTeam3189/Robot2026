/*
* QUESTNAV
  https://github.com/QuestNav
* Copyright (C) 2025 QuestNav
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the MIT License as published.
*/
#include "questnav/QuestNav.h"

#include <utility>

#include <frc/Errors.h>
#include <frc/Timer.h>

// NOTE: The Java library generates this from Gradle (BuildConfig.APP_VERSION).
// Keep this in sync with the release version, or wire it up to the build
// system via a compile definition.
#ifndef QUESTNAV_LIB_VERSION
#define QUESTNAV_LIB_VERSION "dev"
#endif

using namespace questnav;

QuestNav::QuestNav() = default;

// --- Callback registration ---

void QuestNav::OnConnected(std::function<void()> callback) {
  m_onConnectedCallback = std::move(callback);
}

void QuestNav::OnDisconnected(std::function<void()> callback) {
  m_onDisconnectedCallback = std::move(callback);
}

void QuestNav::OnTrackingAcquired(std::function<void()> callback) {
  m_onTrackingAcquiredCallback = std::move(callback);
}

void QuestNav::OnTrackingLost(std::function<void()> callback) {
  m_onTrackingLostCallback = std::move(callback);
}

void QuestNav::OnLowBattery(int thresholdPercent,
                            std::function<void(int)> callback) {
  m_lowBatteryThreshold = thresholdPercent;
  m_onLowBatteryCallback = std::move(callback);
}

void QuestNav::OnCommandSuccess(
    std::function<void(const CommandResponse&)> callback) {
  m_onSuccessCallback = std::move(callback);
}

void QuestNav::OnCommandFailure(
    std::function<void(const CommandResponse&)> callback) {
  m_onFailureCallback = std::move(callback);
}

// --- Internal helpers ---

void QuestNav::CheckVersionMatch() {
  if (!m_versionCheckEnabled || !IsConnected()) {
    return;
  }

  units::second_t currentTime = frc::Timer::GetTimestamp();
  if ((currentTime - m_lastVersionCheckTime) < kVersionCheckInterval) {
    return;
  }
  m_lastVersionCheckTime = currentTime;

  std::string libVersion = GetLibVersion();
  std::string questNavVersion = GetQuestNavVersion();

  if (questNavVersion != libVersion) {
    FRC_ReportWarning(
        "WARNING FROM QUESTNAV: QuestNavLib version ({}) on your robot does "
        "not match QuestNav app version ({}) on your headset. This may cause "
        "compatibility issues. Check the version of your vendordep and the "
        "app running on your headset.",
        libVersion, questNavVersion);
  }
}

void QuestNav::CheckStateCallbacks() {
  // --- Connection state ---
  bool connected = IsConnected();
  if (connected != m_lastConnectedState) {
    if (connected) {
      if (m_onConnectedCallback) {
        m_onConnectedCallback();
      }
    } else {
      if (m_onDisconnectedCallback) {
        m_onDisconnectedCallback();
      }
    }
    m_lastConnectedState = connected;
  }

  // --- Tracking state ---
  bool tracking = IsTracking();
  if (tracking != m_lastTrackingState) {
    if (tracking) {
      if (m_onTrackingAcquiredCallback) {
        m_onTrackingAcquiredCallback();
      }
    } else {
      if (m_onTrackingLostCallback) {
        m_onTrackingLostCallback();
      }
    }
    m_lastTrackingState = tracking;
  }

  // --- Low battery ---
  if (m_onLowBatteryCallback) {
    if (auto level = GetBatteryPercent()) {
      if (*level <= m_lowBatteryThreshold && !m_lowBatteryFired) {
        m_onLowBatteryCallback(*level);
        m_lowBatteryFired = true;
      } else if (*level > m_lowBatteryThreshold) {
        // Reset so the callback can fire again if battery dips below again
        m_lowBatteryFired = false;
      }
    }
  }
}

// --- Public API ---

void QuestNav::SetVersionCheckEnabled(bool enabled) {
  m_versionCheckEnabled = enabled;
}

void QuestNav::SetPose(const frc::Pose3d& pose) {
  Command request;
  request.type = CommandType::kPoseReset;
  request.commandId = ++m_lastSentRequestId;
  request.poseResetPayload = PoseResetPayload{pose};

  m_requestPublisher.Set(request);
}

std::optional<int32_t> QuestNav::GetBatteryPercent() {
  auto latest = m_deviceDataSubscriber.GetAtomic();
  if (latest.serverTime != 0) {
    return latest.value.batteryPercent;
  }
  return std::nullopt;
}

std::optional<int32_t> QuestNav::GetFrameCount() {
  auto latest = m_frameDataSubscriber.GetAtomic();
  if (latest.serverTime != 0) {
    return latest.value.frameCount;
  }
  return std::nullopt;
}

std::optional<int32_t> QuestNav::GetTrackingLostCounter() {
  auto latest = m_deviceDataSubscriber.GetAtomic();
  if (latest.serverTime != 0) {
    return latest.value.trackingLostCounter;
  }
  return std::nullopt;
}

bool QuestNav::IsConnected() {
  return (frc::Timer::GetTimestamp() -
          units::microsecond_t{static_cast<double>(
              m_frameDataSubscriber.GetLastChange())}) < 120_ms;
}

units::millisecond_t QuestNav::GetLatency() {
  return frc::Timer::GetTimestamp() -
         units::microsecond_t{
             static_cast<double>(m_frameDataSubscriber.GetLastChange())};
}

std::optional<double> QuestNav::GetAppTimestamp() {
  auto latest = m_frameDataSubscriber.GetAtomic();
  if (latest.serverTime != 0) {
    return latest.value.timestamp;
  }
  return std::nullopt;
}

bool QuestNav::IsTracking() {
  auto latest = m_frameDataSubscriber.GetAtomic();
  if (latest.serverTime != 0) {
    return latest.value.isTracking;
  }
  return false;
}

std::vector<PoseFrame> QuestNav::GetAllUnreadPoseFrames() {
  std::vector<nt::Timestamped<FrameData>> frameDataArray =
      m_frameDataSubscriber.ReadQueue();
  std::vector<PoseFrame> result;
  result.reserve(frameDataArray.size());
  for (auto& frameData : frameDataArray) {
    result.push_back(PoseFrame{
        frameData.value.pose3d,
        units::second_t{
            units::microsecond_t{static_cast<double>(frameData.serverTime)}}
            .value(),
        frameData.value.timestamp,
        frameData.value.frameCount,
        frameData.value.isTracking,
    });
  }
  return result;
}

void QuestNav::CommandPeriodic() {
  CheckVersionMatch();
  CheckStateCallbacks();

  std::vector<nt::Timestamped<CommandResponse>> responses =
      m_responseSubscriber.ReadQueue();

  for (const auto& tsResponse : responses) {
    const CommandResponse& response = tsResponse.value;
    if (response.success) {
      if (m_onSuccessCallback) {
        m_onSuccessCallback(response);
      }
    } else {
      FRC_ReportError(frc::err::Error, "QuestNav command failed!\n{}",
                      response.errorMessage);
      if (m_onFailureCallback) {
        m_onFailureCallback(response);
      }
    }
  }
}

std::string QuestNav::GetLibVersion() const {
  return QUESTNAV_LIB_VERSION;
}

std::string QuestNav::GetQuestNavVersion() {
  return m_versionSubscriber.Get();
}
