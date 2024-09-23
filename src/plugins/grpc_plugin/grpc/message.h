// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

namespace aimrt::plugins::grpc_plugin::grpc {

constexpr size_t kGrpcMessagePrefixSize = 5;

struct GrpcMessagePrefix {
  uint8_t compression_flag;
  uint32_t message_length;
};

std::optional<GrpcMessagePrefix> ParseGrpcMessagePrefix(std::string_view message);

std::string EncodeGrpcMessagePrefix(const GrpcMessagePrefix& prefix);

}  // namespace aimrt::plugins::grpc_plugin::grpc
