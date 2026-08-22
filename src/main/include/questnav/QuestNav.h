/*
* QUESTNAV
  https://github.com/QuestNav
* Copyright (C) 2025 QuestNav
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the MIT License as published.
*/
#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <frc/geometry/Pose3d.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/ProtobufTopic.h>
#include <networktables/StringTopic.h>
#include <units/time.h>

#include "questnav/PoseFrame.h"
#include "questnav/protos/Messages.h"
#include "questnav/protos/wpilib/CommandsProto.h"
#include "questnav/protos/wpilib/DataProto.h"

namespace questnav {

/**
 * The QuestNav class provides an interface for communicating with a Meta
 * Quest VR headset for robot localization and tracking in FRC.
 *
 * C++ port of gg.questnav.questnav.QuestNav. Public API behavior matches the
 * Java implementation; naming follows WPILib C++ conventions (PascalCase).
 *
 * Basic usage:
 * @code
 * questnav::QuestNav questNav;
 *
 * questNav.OnConnected([] { fmt::print("Quest connected!\n"); });
 * questNav.OnTrackingLost([] { FRC_ReportWarning("Quest tracking lost!"); });
 * questNav.OnLowBattery(20, [](int level) {
 *   FRC_ReportWarning("Quest battery low: {}%", level);
 * });
 *
 * // Set initial Quest pose (required for field-relative tracking)
 * questNav.SetPose(frc::Pose3d{1_m, 2_m, 0_m, frc::Rotation3d{}});
 *
 * // In RobotPeriodic():
 * questNav.CommandPeriodic();
 * for (const auto& frame : questNav.GetAllUnreadPoseFrames()) {
 *   // Use frame.questPose3d and frame.dataTimestamp with a pose estimator
 * }
 * @endcode
 *
 * This class is not thread safe; use it from the main robot loop (the same
 * constraint applies in practice to the Java version's cached objects).
 */
class QuestNav {
 public:
  /**
   * Creates a new QuestNav instance and sets up all NetworkTables
   * publishers/subscribers on the "QuestNav" table.
   */
  QuestNav();

  QuestNav(const QuestNav&) = delete;
  QuestNav& operator=(const QuestNav&) = delete;

  // --- Callback registration ---

  /**
   * Registers a callback that fires once when the Quest transitions from
   * disconnected to connected. Evaluated each CommandPeriodic() call.
   */
  void OnConnected(std::function<void()> callback);

  /**
   * Registers a callback that fires once when the Quest transitions from
   * connected to disconnected. Evaluated each CommandPeriodic() call.
   */
  void OnDisconnected(std::function<void()> callback);

  /**
   * Registers a callback that fires once when the Quest transitions from
   * not-tracking to actively tracking. Evaluated each CommandPeriodic() call.
   */
  void OnTrackingAcquired(std::function<void()> callback);

  /**
   * Registers a callback that fires once when the Quest transitions from
   * tracking to not-tracking. Evaluated each CommandPeriodic() call.
   */
  void OnTrackingLost(std::function<void()> callback);

  /**
   * Registers a callback that fires once when the Quest battery level drops
   * at or below the given threshold. Will not fire again until the battery
   * rises above the threshold and drops back down.
   *
   * @param thresholdPercent Battery percentage (0-100) at or below which the
   *                         callback fires
   * @param callback Receives the current battery percentage
   */
  void OnLowBattery(int thresholdPercent, std::function<void(int)> callback);

  /**
   * Registers a callback that fires for each command response where the
   * command succeeded. Called from CommandPeriodic().
   */
  void OnCommandSuccess(std::function<void(const CommandResponse&)> callback);

  /**
   * Registers a callback that fires for each command response where the
   * command failed. Called from CommandPeriodic(). The failure is also
   * reported to the DriverStation regardless of whether a callback is
   * registered.
   */
  void OnCommandFailure(std::function<void(const CommandResponse&)> callback);

  // --- Public API ---

  /**
   * Turns the version check on or off. When on, a warning will be reported
   * to the DriverStation if the QuestNavLib and QuestNav app versions do not
   * match. Default is enabled.
   */
  void SetVersionCheckEnabled(bool enabled);

  /**
   * Sets the field-relative pose of the Quest headset by commanding it to
   * reset its tracking.
   *
   * Important: This should be the Quest's pose, not the robot's pose. If you
   * know the robot's pose, apply the mounting offset to get the Quest's pose
   * before calling this method.
   *
   * The command is sent asynchronously. Success/failure is reported via
   * OnCommandSuccess()/OnCommandFailure(); failures are also logged to the
   * DriverStation.
   *
   * @param pose The Quest's current field-relative pose (WPILib coordinates)
   */
  void SetPose(const frc::Pose3d& pose);

  /**
   * Returns the Quest's battery level as a percentage (0-100), or empty if
   * no device data is available.
   */
  std::optional<int32_t> GetBatteryPercent();

  /**
   * Gets the current frame count from the Quest headset, or empty if no
   * frame data is available.
   */
  std::optional<int32_t> GetFrameCount();

  /**
   * Gets the number of tracking lost events since app startup, or empty if
   * no device data is available.
   */
  std::optional<int32_t> GetTrackingLostCounter();

  /**
   * Determines if the Quest is currently connected to the robot. Connection
   * is determined by how stale the last received frame from the Quest is
   * (threshold: 120 ms).
   */
  bool IsConnected();

  /**
   * Gets the latency of the Quest -> Robot connection: the time between now
   * and the last frame data update.
   */
  units::millisecond_t GetLatency();

  /**
   * Returns the Quest app's uptime timestamp (seconds since app startup) for
   * debugging and diagnostics, or empty if no frame data is available.
   *
   * For integration with a pose estimator, use PoseFrame::dataTimestamp
   * instead.
   */
  std::optional<double> GetAppTimestamp();

  /**
   * Gets the current tracking state of the Quest headset. Returns false if
   * tracking is lost or no frame data is available.
   */
  bool IsTracking();

  /**
   * Retrieves all new pose frames received from the Quest since the last
   * call to this method. This consumes the frame queue, so each frame is
   * only returned once; call regularly (every robot loop).
   */
  std::vector<PoseFrame> GetAllUnreadPoseFrames();

  /**
   * Processes command responses from the Quest, fires command
   * success/failure callbacks, and evaluates connection, tracking, and
   * battery state callbacks. Call regularly (typically in RobotPeriodic()).
   */
  void CommandPeriodic();

  /** Retrieves the QuestNav-lib version number. */
  std::string GetLibVersion() const;

  /**
   * Retrieves the QuestNav app version running on the Quest headset, or
   * "unknown" if unable to retrieve.
   */
  std::string GetQuestNavVersion();

 private:
  /** Interval at which to check/log a lib-vs-app version mismatch. */
  static constexpr units::second_t kVersionCheckInterval = 5_s;

  void CheckVersionMatch();
  void CheckStateCallbacks();

  /** Battery percentage at or below which the low-battery callback fires. */
  int m_lowBatteryThreshold = 20;

  nt::NetworkTableInstance m_ntInstance = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> m_questNavTable =
      m_ntInstance.GetTable("QuestNav");

  nt::ProtobufSubscriber<CommandResponse> m_responseSubscriber =
      m_questNavTable->GetProtobufTopic<CommandResponse>("response").Subscribe(
          CommandResponse{},
          {.pollStorage = 20, .periodic = 0.05, .sendAll = true});

  nt::ProtobufSubscriber<FrameData> m_frameDataSubscriber =
      m_questNavTable->GetProtobufTopic<FrameData>("frameData").Subscribe(
          FrameData{},
          {.pollStorage = 20, .periodic = 0.01, .sendAll = true});

  nt::ProtobufSubscriber<DeviceData> m_deviceDataSubscriber =
      m_questNavTable->GetProtobufTopic<DeviceData>("deviceData").Subscribe(
          DeviceData{});

  nt::StringSubscriber m_versionSubscriber =
      m_questNavTable->GetStringTopic("version").Subscribe("unknown");

  nt::ProtobufPublisher<Command> m_requestPublisher =
      m_questNavTable->GetProtobufTopic<Command>("request").Publish();

  /** Last sent request id. */
  uint32_t m_lastSentRequestId = 0;

  /** True to check for lib/app version match at an interval. */
  bool m_versionCheckEnabled = true;

  /** The last time the versions were checked for a match. */
  units::second_t m_lastVersionCheckTime = 0_s;

  // Callback state tracking
  bool m_lastConnectedState = false;
  bool m_lastTrackingState = false;
  bool m_lowBatteryFired = false;

  // Callbacks
  std::function<void()> m_onConnectedCallback;
  std::function<void()> m_onDisconnectedCallback;
  std::function<void()> m_onTrackingAcquiredCallback;
  std::function<void()> m_onTrackingLostCallback;
  std::function<void(int)> m_onLowBatteryCallback;
  std::function<void(const CommandResponse&)> m_onSuccessCallback;
  std::function<void(const CommandResponse&)> m_onFailureCallback;
};

}  // namespace questnav
