// Copyright (c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#pragma once

#include <cstdint>

// This needs to be updated when the code generator is updated
// and backward compability is broken.
#define AIMRT_MIN_GENCODE_VERSION "0.10.0"

constexpr uint32_t VersionToInt(const char* version) {
  uint32_t major = 0;
  uint32_t minor = 0;
  uint32_t patch = 0;
  const char* p = version;

  while (*p >= '0' && *p <= '9') {
    major = major * 10 + (*p - '0');
    ++p;
  }
  if (*p != '.') throw "Invalid version format";
  ++p;

  while (*p >= '0' && *p <= '9') {
    minor = minor * 10 + (*p - '0');
    ++p;
  }
  if (*p != '.') throw "Invalid version format";
  ++p;

  while (*p >= '0' && *p <= '9') {
    patch = patch * 10 + (*p - '0');
    ++p;
  }

  return (major * 1000000) + (minor * 1000) + patch;
}

constexpr uint32_t AIMRT_RUNTIME_VERSION_INT = VersionToInt(AIMRT_RUNTIME_VERSION);
constexpr uint32_t AIMRT_MIN_GENCODE_VERSION_INT = VersionToInt(AIMRT_MIN_GENCODE_VERSION);
