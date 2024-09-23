// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <chrono>
#include <optional>
#include <string>

namespace aimrt::plugins::grpc_plugin::grpc {

constexpr std::chrono::nanoseconds kDefaultTimeout = std::chrono::nanoseconds::max();

std::optional<std::chrono::nanoseconds> ParseTimeout(const std::string& timeout_str);

std::string FormatTimeout(std::chrono::nanoseconds timeout);

}  // namespace aimrt::plugins::grpc_plugin::grpc