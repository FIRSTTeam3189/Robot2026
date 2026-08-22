/*
* QUESTNAV
  https://github.com/QuestNav
* Copyright (C) 2025 QuestNav
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the MIT License as published.
*/
#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <frc/geometry/Pose3d.h>

namespace questnav {

/**
 * Command types that can be sent to the QuestNav app.
 * Values match questnav.protos.commands.QuestNavCommandType.
 */
enum class CommandType : int {
  kUnspecified = 0,
  kPoseReset = 1,
};

/**
 * C++ value type for questnav.protos.commands.ProtobufQuestNavPoseResetPayload.
 */
struct PoseResetPayload {
  /// The target pose to reset Quest tracking to (WPILib field coordinates).
  frc::Pose3d targetPose;
};

/**
 * C++ value type for questnav.protos.commands.ProtobufQuestNavCommand.
 */
struct Command {
  /// The type of command being sent.
  CommandType type = CommandType::kUnspecified;
  /// Unique identifier for this command instance.
  uint32_t commandId = 0;
  /// Payload for POSE_RESET commands (empty for commands with no payload).
  std::optional<PoseResetPayload> poseResetPayload;
};

/**
 * C++ value type for questnav.protos.commands.ProtobufQuestNavCommandResponse.
 */
struct CommandResponse {
  /// Command ID matching the original command.
  uint32_t commandId = 0;
  /// Whether the command was executed successfully.
  bool success = false;
  /// Human-readable error message when success == false.
  std::string errorMessage;
};

/**
 * C++ value type for questnav.protos.data.ProtobufQuestNavFrameData.
 */
struct FrameData {
  /// Sequential frame counter for this tracking session.
  int32_t frameCount = 0;
  /// Timestamp in seconds since QuestNav app startup.
  double timestamp = 0.0;
  /// Quest's 3D pose on the field in WPILib coordinates.
  frc::Pose3d pose3d;
  /// Tracking status of the Quest when this frame was captured.
  bool isTracking = false;
};

/**
 * C++ value type for questnav.protos.data.ProtobufQuestNavDeviceData.
 */
struct DeviceData {
  /// Count of tracking loss events since app startup.
  int32_t trackingLostCounter = 0;
  /// Quest device battery level as a percentage (0-100).
  int32_t batteryPercent = 0;
};

}  // namespace questnav
