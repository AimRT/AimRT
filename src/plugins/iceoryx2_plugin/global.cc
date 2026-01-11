// Copyright (c) 2024, SmartCar Project
// Iceoryx2 Plugin for AimRT

#include "iceoryx2_plugin/global.h"

#include <atomic>

namespace aimrt::plugins::iceoryx2_plugin {

// Thread-safe global logger (set once during initialization)
static std::atomic<aimrt::logger::LoggerRef> global_logger{};

void SetLogger(aimrt::logger::LoggerRef logger) {
  global_logger.store(logger, std::memory_order_release);
}

aimrt::logger::LoggerRef GetLogger() {
  auto logger = global_logger.load(std::memory_order_acquire);
  return logger ? logger : aimrt::logger::GetSimpleLoggerRef();
}

}  // namespace aimrt::plugins::iceoryx2_plugin
