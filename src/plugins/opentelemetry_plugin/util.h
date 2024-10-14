// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string_view>

namespace aimrt::plugins::opentelemetry_plugin {

static constexpr std::string_view kCtxKeyPrefix = "aimrt_otp-";

static constexpr std::string_view kCtxKeyStartNewTrace = "aimrt_otp-start_new_trace";
static constexpr std::string_view kCtxKeyTraceParent = "aimrt_otp-traceparent";
static constexpr std::string_view kCtxKeyTraceState = "aimrt_otp-tracestate";

}  // namespace aimrt::plugins::opentelemetry_plugin