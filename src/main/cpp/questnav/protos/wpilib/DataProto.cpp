/*
* QUESTNAV
  https://github.com/QuestNav
* Copyright (C) 2025 QuestNav
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the MIT License as published.
*/
#include "questnav/protos/wpilib/DataProto.h"

#include <frc/geometry/proto/Pose3dProto.h>
#include <wpi/protobuf/ProtobufCallbacks.h>

std::optional<questnav::FrameData> wpi::Protobuf<questnav::FrameData>::Unpack(
    InputStream& stream) {
  wpi::UnpackCallback<frc::Pose3d> pose;
  questnav_protos_data_ProtobufQuestNavFrameData msg{
      .frame_count = 0,
      .timestamp = 0,
      .pose3d = pose.Callback(),
      .isTracking = false,
  };
  if (!stream.Decode(msg)) {
    return {};
  }

  auto ipose = pose.Items();

  return questnav::FrameData{
      msg.frame_count,
      msg.timestamp,
      // proto3 submessages are optional on the wire; match the Java behavior
      // (quickbuf returns an empty message -> identity pose) if absent.
      ipose.empty() ? frc::Pose3d{} : ipose[0],
      msg.isTracking,
  };
}

bool wpi::Protobuf<questnav::FrameData>::Pack(OutputStream& stream,
                                              const questnav::FrameData& value) {
  wpi::PackCallback pose{&value.pose3d};
  questnav_protos_data_ProtobufQuestNavFrameData msg{
      .frame_count = value.frameCount,
      .timestamp = value.timestamp,
      .pose3d = pose.Callback(),
      .isTracking = value.isTracking,
  };
  return stream.Encode(msg);
}

std::optional<questnav::DeviceData> wpi::Protobuf<questnav::DeviceData>::Unpack(
    InputStream& stream) {
  questnav_protos_data_ProtobufQuestNavDeviceData msg{
      .tracking_lost_counter = 0,
      .battery_percent = 0,
  };
  if (!stream.Decode(msg)) {
    return {};
  }
  return questnav::DeviceData{msg.tracking_lost_counter, msg.battery_percent};
}

bool wpi::Protobuf<questnav::DeviceData>::Pack(OutputStream& stream,
                                               const questnav::DeviceData& value) {
  questnav_protos_data_ProtobufQuestNavDeviceData msg{
      .tracking_lost_counter = value.trackingLostCounter,
      .battery_percent = value.batteryPercent,
  };
  return stream.Encode(msg);
}
