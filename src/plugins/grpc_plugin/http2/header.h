// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>
#include <unordered_map>

namespace aimrt::plugins::grpc_plugin::http2 {

using Headers = std::unordered_map<std::string, std::string>;
using Trailers = std::unordered_map<std::string, std::string>;

}  // namespace aimrt::plugins::grpc_plugin::http2
