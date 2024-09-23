// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "grpc_plugin/grpc/message.h"

#include <string>

namespace aimrt::plugins::grpc_plugin::grpc {

std::optional<GrpcMessagePrefix> ParseGrpcMessagePrefix(std::string_view message) {
  if (message.size() < kGrpcMessagePrefixSize) {
    return std::nullopt;
  }
  auto compression_flag = message[0];
  auto message_length = (static_cast<uint32_t>(static_cast<unsigned char>(message[1])) << 24) |
                        (static_cast<uint32_t>(static_cast<unsigned char>(message[2])) << 16) |
                        (static_cast<uint32_t>(static_cast<unsigned char>(message[3])) << 8) |
                        static_cast<uint32_t>(static_cast<unsigned char>(message[4]));

  return GrpcMessagePrefix{
      .compression_flag = static_cast<uint8_t>(compression_flag),
      .message_length = static_cast<uint32_t>(message_length),
  };
}

std::string EncodeGrpcMessagePrefix(const GrpcMessagePrefix& prefix) {
  auto result = std::string(kGrpcMessagePrefixSize, 0);
  result[0] = static_cast<char>(prefix.compression_flag);
  result[1] = static_cast<char>((prefix.message_length >> 24) & 0xFF);
  result[2] = static_cast<char>((prefix.message_length >> 16) & 0xFF);
  result[3] = static_cast<char>((prefix.message_length >> 8) & 0xFF);
  result[4] = static_cast<char>(prefix.message_length & 0xFF);
  return result;
}

}  // namespace aimrt::plugins::grpc_plugin::grpc
