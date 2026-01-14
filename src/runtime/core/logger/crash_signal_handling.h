// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <csignal>
#include <memory>
#include <vector>

#if defined(_WIN32)
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <windows.h>
#else
  #include <signal.h>  // siginfo_t
#endif

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
#if defined(_WIN32)
  static void PrintStacktrace(CONTEXT* ctx, int skip_frames);
  static LONG WINAPI UnhandledExceptionFilter(EXCEPTION_POINTERS* info);
  static void AbortHandler(int);
#else
  struct FreeDeleter {
    void operator()(char* p) const noexcept;
  };

  static void HandleSignal(int signo, siginfo_t* info, void* _ctx);
  static void SigHandler(int signo, siginfo_t* info, void* _ctx);
  void EnsureAltStack();
  void InstallHandlers(const std::vector<int>& signals);
#endif

 private:
  inline static std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_ =
      std::make_shared<aimrt::common::util::LoggerWrapper>();
#if !defined(_WIN32)
  std::unique_ptr<char, FreeDeleter> alt_stack_{nullptr};
  bool loaded_{false};
#endif
};

}  // namespace aimrt::runtime::core::logger
