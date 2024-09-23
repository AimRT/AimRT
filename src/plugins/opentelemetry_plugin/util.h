// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string_view>

#include "opentelemetry/nostd/string_view.h"

namespace aimrt::plugins::opentelemetry_plugin {

static constexpr std::string_view kCtxKeyPrefix = "aimrt_otp-";

static constexpr std::string_view kCtxKeyStartNewTrace = "aimrt_otp-start_new_trace";
static constexpr std::string_view kCtxKeyTraceParent = "aimrt_otp-traceparent";
static constexpr std::string_view kCtxKeyTraceState = "aimrt_otp-tracestate";

inline opentelemetry::nostd::string_view ToNoStdStringView(std::string_view s) {
  return opentelemetry::nostd::string_view(s.data(), s.size());
}

inline std::string_view ToStdStringView(opentelemetry::nostd::string_view s) {
  return std::string_view(s.data(), s.size());
}

}  // namespace aimrt::plugins::opentelemetry_plugin