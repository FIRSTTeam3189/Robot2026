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
#include "questnav/protos/commands.npb.h"

/**
 * WPILib Protobuf layer for PoseResetPayload
 * (questnav.protos.commands.ProtobufQuestNavPoseResetPayload).
 */
template <>
struct wpi::Protobuf<questnav::PoseResetPayload> {
  using MessageStruct = questnav_protos_commands_ProtobufQuestNavPoseResetPayload;
  using InputStream = wpi::ProtoInputStream<questnav::PoseResetPayload>;
  using OutputStream = wpi::ProtoOutputStream<questnav::PoseResetPayload>;
  static std::optional<questnav::PoseResetPayload> Unpack(InputStream& stream);
  static bool Pack(OutputStream& stream, const questnav::PoseResetPayload& value);
};

/** WPILib Protobuf layer for Command (questnav.protos.commands.ProtobufQuestNavCommand). */
template <>
struct wpi::Protobuf<questnav::Command> {
  using MessageStruct = questnav_protos_commands_ProtobufQuestNavCommand;
  using InputStream = wpi::ProtoInputStream<questnav::Command>;
  using OutputStream = wpi::ProtoOutputStream<questnav::Command>;
  static std::optional<questnav::Command> Unpack(InputStream& stream);
  static bool Pack(OutputStream& stream, const questnav::Command& value);
};

/**
 * WPILib Protobuf layer for CommandResponse
 * (questnav.protos.commands.ProtobufQuestNavCommandResponse).
 */
template <>
struct wpi::Protobuf<questnav::CommandResponse> {
  using MessageStruct = questnav_protos_commands_ProtobufQuestNavCommandResponse;
  using InputStream = wpi::ProtoInputStream<questnav::CommandResponse>;
  using OutputStream = wpi::ProtoOutputStream<questnav::CommandResponse>;
  static std::optional<questnav::CommandResponse> Unpack(InputStream& stream);
  static bool Pack(OutputStream& stream, const questnav::CommandResponse& value);
};
