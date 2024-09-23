// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string_view>

#include <nghttp2/nghttp2.h>

namespace aimrt::plugins::grpc_plugin::http2 {

constexpr std::string_view kHeaderNameHost = ":host";
constexpr std::string_view kHeaderNameMethod = ":method";
constexpr std::string_view kHeaderNamePath = ":path";
constexpr std::string_view kHeaderNameScheme = ":scheme";
constexpr std::string_view kHeaderNameAuthority = ":authority";
constexpr std::string_view kHeaderNameStatus = ":status";

inline nghttp2_nv CreatePairWithoutCopy(std::string_view name, std::string_view value) {
  return {.name = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(name.data())),
          .value = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(value.data())),
          .namelen = name.size(),
          .valuelen = value.size(),
          .flags = NGHTTP2_NV_FLAG_NO_COPY_NAME | NGHTTP2_NV_FLAG_NO_COPY_VALUE};
}

}  // namespace aimrt::plugins::grpc_plugin::http2
