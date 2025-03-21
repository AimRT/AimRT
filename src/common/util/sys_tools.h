// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#ifdef _WIN32
  #include <windows.h>
#else
  #include <limits.h>
  #include <unistd.h>
#endif

namespace aimrt::common::util {

/**
 * @brief get current executable path
 *
 * @return std::string current executable path
 */
inline static std::string GetExecutablePath() {
  std::string path;
#ifdef _WIN32
  char buffer[MAX_PATH];
  GetModuleFileNameA(NULL, buffer, MAX_PATH);
  path = buffer;
#else
  char buffer[PATH_MAX];
  ssize_t len = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
  if (len != -1) {
    buffer[len] = '\0';
    path = buffer;
  }
#endif
  return path;
};

/**
 * @brief get current process pid
 *
 * @return std::string current process pid
 */
inline static std::string GetCurrentProcessPid() {
#ifdef _WIN32
  return std::to_string(GetCurrentProcessId());
#else
  return std::to_string(getpid());
#endif
};

}  // namespace aimrt::common::util
