#pragma once

#include <atomic>
#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"
#include "core/aimrt_core.h"

namespace aimrt::plugins::topic_logger_plugin {
class TopicLoggerPlugin : public AimRTCorePluginBase {
 public:
  struct Options {
  };

 public:
  TopicLoggerPlugin() = default;
  ~TopicLoggerPlugin() override = default;

  std::string_view Name() const noexcept override { return "topic_logger_plugin"; }

  bool Initialize(runtime::core::AimRTCore* core_ptr) noexcept override;
  void Shutdown() noexcept override;

 private:
  runtime::core::AimRTCore* core_ptr_ = nullptr;

  std::unordered_map<runtime::core::AimRTCore::State, std::function<void()>> hook_task_map_;

  Options options_;

  bool init_flag_ = false;

  std::atomic_bool stop_flag_ = false;
};

}  // namespace aimrt::plugins::topic_logger_plugin