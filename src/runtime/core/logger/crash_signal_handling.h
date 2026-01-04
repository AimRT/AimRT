// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <csignal>
#include <memory>
#include <vector>
#include "util/log_util.h"

namespace aimrt::runtime::core::logger {

class CrashSignalHandling {
 public:
  CrashSignalHandling();
  ~CrashSignalHandling() = default;

  CrashSignalHandling(const CrashSignalHandling&) = delete;
  CrashSignalHandling& operator=(const CrashSignalHandling&) = delete;

  static void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  static const aimrt::common::util::LoggerWrapper& GetLogger() { return *logger_ptr_; }

 private:
  struct FreeDeleter {
    void operator()(char* p) const noexcept;
  };

  static void HandleSignal(int signo, siginfo_t* info, void* _ctx);
  static void SigHandler(int signo, siginfo_t* info, void* _ctx);
  void EnsureAltStack();
  void InstallHandlers(const std::vector<int>& signals);

 private:
  inline static std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;
  std::unique_ptr<char, FreeDeleter> alt_stack_{nullptr};
  bool loaded_{false};
};

}  // namespace aimrt::runtime::core::logger
