// Copyright (c) 2024, SmartCar Project
// Iceoryx2 Plugin for AimRT - Zero-copy IPC using Iceoryx2

#pragma once

#include <atomic>

#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"
#include "runtime/core/aimrt_core.h"

namespace aimrt::plugins::iceoryx2_plugin {

// Forward declaration
class Iceoryx2ChannelBackend;

class Iceoryx2Plugin : public AimRTCorePluginBase {
 public:
  struct Options {
    // Shared memory configuration
    uint64_t shm_init_size = 16 * 1024 * 1024;  // 16MB default
    uint64_t max_slice_len = 4 * 1024 * 1024;   // 4MB default per message

    // Allocation strategy: "static", "dynamic", "power_of_two"
    std::string allocation_strategy = "dynamic";

    // Listener thread configuration
    std::string listener_thread_name;
  };

 public:
  Iceoryx2Plugin() = default;
  ~Iceoryx2Plugin() override = default;

  std::string_view Name() const noexcept override { return "iceoryx2_plugin"; }

  bool Initialize(runtime::core::AimRTCore* core_ptr) noexcept override;
  void Shutdown() noexcept override;

 private:
  void SetPluginLogger();
  void RegisterIceoryx2ChannelBackend();

 private:
  runtime::core::AimRTCore* core_ptr_ = nullptr;

  Options options_;

  bool init_flag_ = false;

  std::atomic_bool stop_flag_ = false;
};

}  // namespace aimrt::plugins::iceoryx2_plugin
