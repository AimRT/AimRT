#include "topic_logger_plugin/topic_logger_plugin.h"
#include "topic_logger_plugin/topic_logger_backend.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::topic_logger_plugin::TopicLoggerPlugin::Options> {
  using Options = aimrt::plugins::topic_logger_plugin::TopicLoggerPlugin::Options;

  static Node encode(const Options& rhs) {
    Node node;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::topic_logger_plugin {
bool TopicLoggerPlugin::Initialize(runtime::core::AimRTCore* core_ptr) noexcept {
  try {
    core_ptr_ = core_ptr;

    YAML::Node plugin_options_node = core_ptr_->GetPluginManager().GetPluginOptionsNode(Name());

    if (plugin_options_node && !plugin_options_node.IsNull()) {
      options_ = plugin_options_node.as<Options>();
    }

    init_flag_ = true;

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitLog,
                                [this] {
                                  core_ptr_->GetLoggerManager()
                                      .RegisterLoggerBackendGenFunc(
                                          "topic_logger",
                                          [this]() -> std::unique_ptr<runtime::core::logger::LoggerBackendBase> {
                                            auto topic_logger_backend_ptr = std::make_unique<TopicLoggerBackend>();

                                            topic_logger_backend_ptr->RegisterCorePtr(core_ptr_);

                                            topic_logger_backend_ptr->RegisterGetExecutorFunc(
                                                [this](std::string_view executor_name) -> aimrt::executor::ExecutorRef {
                                                  return core_ptr_->GetExecutorManager().GetExecutor(executor_name);
                                                }

                                            );

                                            hook_task_map_.emplace(runtime::core::AimRTCore::State::kPostInitChannel,
                                                                   [topic_logger_backend = topic_logger_backend_ptr.get()]() {
                                                                     topic_logger_backend->RegisterLogPublisher();
                                                                   });

                                            hook_task_map_.emplace(runtime::core::AimRTCore::State::kPreStartChannel,
                                                                   [topic_logger_backend = topic_logger_backend_ptr.get()]() {
                                                                     topic_logger_backend->StartupPulisher();
                                                                   });

                                            hook_task_map_.emplace(runtime::core::AimRTCore::State::kPostShutdownChannel,
                                                                   [topic_logger_backend = topic_logger_backend_ptr.get()]() {
                                                                     //  topic_logger_backend->StopPulisher();
                                                                   });
                                            return topic_logger_backend_ptr;
                                          });
                                });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostInitChannel,
                                [this] {
                                  hook_task_map_[runtime::core::AimRTCore::State::kPostInitChannel]();
                                });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreStartChannel,
                                [this] {
                                  hook_task_map_[runtime::core::AimRTCore::State::kPreStartChannel]();
                                });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostShutdownChannel,
                                [this] {
                                  hook_task_map_[runtime::core::AimRTCore::State::kPostShutdownChannel]();
                                });

    plugin_options_node = options_;
    core_ptr_->GetPluginManager().UpdatePluginOptionsNode(Name(), plugin_options_node);

    return true;

  } catch (const std::exception& e) {
    (void)fprintf(stderr, "ERROR: TopicLoggerPlugin initialize failed\n");
    return false;
  }
}

void TopicLoggerPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;

    stop_flag_ = true;

  } catch (const std::exception& e) {
    (void)fprintf(stderr, "ERROR: TopicLoggerPlugin shutdown failed\n");
  }
}

}  // namespace aimrt::plugins::topic_logger_plugin