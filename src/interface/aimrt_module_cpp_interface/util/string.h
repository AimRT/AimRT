// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstring>
#include <string>

#include "aimrt_module_c_interface/util/string.h"

namespace aimrt::util {

inline aimrt_string_view_t ToAimRTStringView(const std::string& s) {
  return aimrt_string_view_t{s.c_str(), s.size()};
}

inline aimrt_string_view_t ToAimRTStringView(std::string_view s) {
  return aimrt_string_view_t{s.data(), s.size()};
}

inline aimrt_string_view_t ToAimRTStringView(const char* s) {
  return aimrt_string_view_t{s, strlen(s)};
}

inline std::string_view ToStdStringView(aimrt_string_view_t s) {
  return std::string_view(s.str, s.len);
}

inline std::string ToStdString(aimrt_string_view_t s) {
  return std::string(s.str, s.len);
}

}  // namespace aimrt::util
