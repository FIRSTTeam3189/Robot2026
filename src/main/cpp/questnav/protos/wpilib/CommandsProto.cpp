/*
* QUESTNAV
  https://github.com/QuestNav
* Copyright (C) 2025 QuestNav
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the MIT License as published.
*/
#include "questnav/protos/wpilib/CommandsProto.h"

#include <string>
#include <utility>

#include <frc/geometry/proto/Pose3dProto.h>
#include <wpi/protobuf/ProtobufCallbacks.h>

// --- PoseResetPayload ---

std::optional<questnav::PoseResetPayload>
wpi::Protobuf<questnav::PoseResetPayload>::Unpack(InputStream& stream) {
  wpi::UnpackCallback<frc::Pose3d> pose;
  questnav_protos_commands_ProtobufQuestNavPoseResetPayload msg{
      .target_pose = pose.Callback(),
  };
  if (!stream.Decode(msg)) {
    return {};
  }
  auto ipose = pose.Items();
  return questnav::PoseResetPayload{ipose.empty() ? frc::Pose3d{} : ipose[0]};
}

bool wpi::Protobuf<questnav::PoseResetPayload>::Pack(
    OutputStream& stream, const questnav::PoseResetPayload& value) {
  wpi::PackCallback pose{&value.targetPose};
  questnav_protos_commands_ProtobufQuestNavPoseResetPayload msg{
      .target_pose = pose.Callback(),
  };
  return stream.Encode(msg);
}

// --- Command ---

std::optional<questnav::Command> wpi::Protobuf<questnav::Command>::Unpack(
    InputStream& stream) {
  wpi::UnpackCallback<questnav::PoseResetPayload> payload;
  questnav_protos_commands_ProtobufQuestNavCommand msg;
  msg.type = questnav_protos_commands_QuestNavCommandType_COMMAND_TYPE_UNSPECIFIED;
  msg.command_id = 0;
  msg.which_payload = 0;
  msg.payload.pose_reset_payload = payload.Callback();
  if (!stream.Decode(msg)) {
    return {};
  }

  questnav::Command result;
  result.type = static_cast<questnav::CommandType>(msg.type);
  result.commandId = msg.command_id;
  // NOTE: nanopb's decode path for callback-type oneof members
  // (decode_callback_field) invokes the pre-set pb_callback_t but does NOT
  // update which_payload, so presence is detected via the captured items
  // rather than the oneof discriminator.
  auto ipayload = payload.Items();
  if (!ipayload.empty()) {
    result.poseResetPayload = ipayload[0];
  }
  return result;
}

bool wpi::Protobuf<questnav::Command>::Pack(OutputStream& stream,
                                            const questnav::Command& value) {
  questnav_protos_commands_ProtobufQuestNavCommand msg;
  msg.type = static_cast<questnav_protos_commands_QuestNavCommandType>(value.type);
  msg.command_id = value.commandId;
  msg.which_payload = 0;
  msg.payload.pose_reset_payload = pb_callback_t{{nullptr}, nullptr};

  if (value.poseResetPayload) {
    // The PackCallback must stay alive for the duration of Encode().
    wpi::PackCallback payload{&*value.poseResetPayload};
    msg.which_payload =
        questnav_protos_commands_ProtobufQuestNavCommand_pose_reset_payload_tag;
    msg.payload.pose_reset_payload = payload.Callback();
    return stream.Encode(msg);
  }
  return stream.Encode(msg);
}

// --- CommandResponse ---

std::optional<questnav::CommandResponse>
wpi::Protobuf<questnav::CommandResponse>::Unpack(InputStream& stream) {
  wpi::UnpackCallback<std::string> error;
  questnav_protos_commands_ProtobufQuestNavCommandResponse msg{
      .command_id = 0,
      .success = false,
      .error_message = error.Callback(),
  };
  if (!stream.Decode(msg)) {
    return {};
  }
  auto ierror = error.Items();
  return questnav::CommandResponse{
      msg.command_id,
      msg.success,
      ierror.empty() ? std::string{} : std::move(ierror[0]),
  };
}

bool wpi::Protobuf<questnav::CommandResponse>::Pack(
    OutputStream& stream, const questnav::CommandResponse& value) {
  wpi::PackCallback<std::string> error{&value.errorMessage};
  questnav_protos_commands_ProtobufQuestNavCommandResponse msg{
      .command_id = value.commandId,
      .success = value.success,
      .error_message = error.Callback(),
  };
  return stream.Encode(msg);
}
