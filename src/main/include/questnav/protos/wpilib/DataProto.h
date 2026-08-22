/*
* QUESTNAV
  https://github.com/QuestNav
* Copyright (C) 2025 QuestNav
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the MIT License as published.
*/
#pragma once

#include <optional>

#include <wpi/protobuf/Protobuf.h>

#include "questnav/protos/Messages.h"
#include "questnav/protos/data.npb.h"

/** WPILib Protobuf layer for FrameData (questnav.protos.data.ProtobufQuestNavFrameData). */
template <>
struct wpi::Protobuf<questnav::FrameData> {
  using MessageStruct = questnav_protos_data_ProtobufQuestNavFrameData;
  using InputStream = wpi::ProtoInputStream<questnav::FrameData>;
  using OutputStream = wpi::ProtoOutputStream<questnav::FrameData>;
  static std::optional<questnav::FrameData> Unpack(InputStream& stream);
  static bool Pack(OutputStream& stream, const questnav::FrameData& value);
};

/** WPILib Protobuf layer for DeviceData (questnav.protos.data.ProtobufQuestNavDeviceData). */
template <>
struct wpi::Protobuf<questnav::DeviceData> {
  using MessageStruct = questnav_protos_data_ProtobufQuestNavDeviceData;
  using InputStream = wpi::ProtoInputStream<questnav::DeviceData>;
  using OutputStream = wpi::ProtoOutputStream<questnav::DeviceData>;
  static std::optional<questnav::DeviceData> Unpack(InputStream& stream);
  static bool Pack(OutputStream& stream, const questnav::DeviceData& value);
};
