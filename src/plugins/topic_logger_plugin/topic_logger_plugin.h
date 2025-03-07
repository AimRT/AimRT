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
  void RegisterTopicLoggerBackend();

 private:
  runtime::core::AimRTCore* core_ptr_ = nullptr;

  std::vector<std::function<void()>> post_init_channel_hook_task_vec_;
  std::vector<std::function<void()>> pre_start_channel_hook_task_vec_;
  std::vector<std::function<void()>> post_shutdown_channel_hook_task_vec_;

  Options options_;

  bool init_flag_ = false;

  std::atomic_bool stop_flag_ = false;
};

}  // namespace aimrt::plugins::topic_logger_plugin