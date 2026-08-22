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

#include <frc/geometry/Pose3d.h>

namespace questnav {

/**
 * Represents a single frame of pose tracking data received from the Quest
 * headset.
 *
 * C++ port of the Java PoseFrame record. Field semantics are identical:
 *
 * - questPose3d: The Quest's pose on the field (WPILib field coordinates;
 *   meters / radians). Only meaningful field-relative after
 *   QuestNav::SetPose() has been called.
 * - dataTimestamp: NetworkTables reception timestamp in seconds (robot time
 *   base). Use this timestamp when adding vision measurements to a pose
 *   estimator.
 * - appTimestamp: Quest app's internal timestamp in seconds since app
 *   startup. Debugging/diagnostics only.
 * - frameCount: Sequential frame number from the Quest tracking system.
 * - isTracking: Whether the Quest was tracking when this frame was captured.
 */
struct PoseFrame {
  /// The Quest's pose on the field as measured by Quest tracking.
  frc::Pose3d questPose3d;
  /// NetworkTables timestamp when this frame was received (seconds).
  double dataTimestamp = 0.0;
  /// Quest app internal timestamp (seconds since app startup).
  double appTimestamp = 0.0;
  /// Sequential frame number from the Quest tracking system.
  int32_t frameCount = 0;
  /// Whether the Quest was tracking when this frame was captured.
  bool isTracking = false;
};

}  // namespace questnav
