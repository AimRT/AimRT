// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "topic_logger_plugin/topic_logger_backend.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::topic_logger_plugin::TopicLoggerBackend::Options> {
  using Options = aimrt::plugins::topic_logger_plugin::TopicLoggerBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["module_filter"] = rhs.module_filter;
    node["interval_ms"] = rhs.interval_ms;
    node["timer_executor_name"] = rhs.timer_executor_name;
    node["topic_name"] = rhs.topic_name;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["interval_ms"])
      rhs.interval_ms = node["interval_ms"].as<uint32_t>();
    if (node["module_filter"])
      rhs.module_filter = node["module_filter"].as<std::string>();

    rhs.topic_name = node["topic_name"].as<std::string>();
    rhs.timer_executor_name = node["timer_executor_name"].as<std::string>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::topic_logger_plugin {
void TopicLoggerBackend::Initialize(YAML::Node options_node) {
  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  // register timer
  AIMRT_ASSERT(!options_.timer_executor_name.empty(), "Timer executor name is empty.");

  timer_executor_ = get_executor_func_(options_.timer_executor_name);
  AIMRT_ASSERT(timer_executor_, "Invalid timer executor name: {}", options_.timer_executor_name);
  AIMRT_ASSERT(timer_executor_.SupportTimerSchedule(),
               "Timer executor {} must support timer schedule.", options_.timer_executor_name);

  auto timer_task = [this]() {
    std::queue<aimrt::protocols::topic_logger::SingleLogData> tmp_queue;
    {
      std::unique_lock<std::mutex> lck(mutex_);
      // if queue is empty, then stop timer to avoid unnecessary work
      if (queue_.empty()) [[unlikely]] {
        publish_flag_ = false;
        timer_ptr->Cancel();
        return;
      }
      queue_.swap(tmp_queue);
    }

    aimrt::protocols::topic_logger::LogData log_data;
    while (!tmp_queue.empty()) {
      auto& single_log_data = tmp_queue.front();
      log_data.mutable_logs()->Add(std::move(single_log_data));
      tmp_queue.pop();
    }

    aimrt::channel::Publish(log_publisher_, log_data);
  };
  timer_ptr = executor::CreateTimer(timer_executor_,
                                    std::chrono::milliseconds(options_.interval_ms),
                                    std::move(timer_task),
                                    false);
  options_node = options_;

  run_flag_.store(true);
}

void TopicLoggerBackend::Log(const runtime::core::logger::LogDataWrapper& log_data_wrapper) noexcept {
  try {
    if (!run_flag_.load()) [[unlikely]] {
      return;
    }

    if (!CheckLog(log_data_wrapper)) [[unlikely]] {
      return;
    }

    aimrt::protocols::topic_logger::SingleLogData single_log_data;
    single_log_data.set_module_name(log_data_wrapper.module_name.data(), log_data_wrapper.module_name.size());
    single_log_data.set_thread_id(log_data_wrapper.thread_id);
    single_log_data.set_time_point(log_data_wrapper.t.time_since_epoch().count());
    single_log_data.set_level(static_cast<::aimrt::protocols::topic_logger::LogLevel>(log_data_wrapper.lvl));
    single_log_data.set_line(log_data_wrapper.line);
    single_log_data.set_column(log_data_wrapper.column);
    single_log_data.set_file_name(log_data_wrapper.file_name);
    single_log_data.set_function_name(log_data_wrapper.function_name);
    single_log_data.set_message(log_data_wrapper.log_data, log_data_wrapper.log_data_size);

    {
      std::unique_lock<std::mutex> lck(mutex_);

      queue_.emplace(std::move(single_log_data));

      // if timer is stop, then reset it
      if (!publish_flag_) [[unlikely]] {
        publish_flag_ = true;
        timer_ptr->Reset();
      }
    }

  } catch (const std::exception& e) {
    (void)fprintf(stderr, "Log get exception: %s\n", e.what());
  }
}
bool TopicLoggerBackend::CheckLog(const runtime::core::logger::LogDataWrapper& log_data_wrapper) {
  {
    std::shared_lock lock(module_filter_map_mutex_);
    auto find_itr = module_filter_map_.find(log_data_wrapper.module_name);
    if (find_itr != module_filter_map_.end()) {
      return find_itr->second;
    }
  }

  bool if_log = false;

  try {
    if (std::regex_match(
            log_data_wrapper.module_name.begin(),
            log_data_wrapper.module_name.end(),
            std::regex(options_.module_filter, std::regex::ECMAScript))) {
      if_log = true;
    }
  } catch (const std::exception& e) {
    (void)fprintf(stderr, "Regex get exception, expr: %s, string: %s, exception info: %s\n",
                  options_.module_filter.c_str(), log_data_wrapper.module_name.data(), e.what());
  }

  std::unique_lock lock(module_filter_map_mutex_);
  module_filter_map_.emplace(log_data_wrapper.module_name, if_log);

  return if_log;
}

void TopicLoggerBackend::RegisterLogPublisher() {
  log_publisher_ = get_publisher_ref_func_(options_.topic_name);
  AIMRT_ASSERT(log_publisher_, "Failed to get publisher for topic: {}", options_.topic_name);

  bool ret = aimrt::channel::RegisterPublishType<aimrt::protocols::topic_logger::LogData>(log_publisher_);
  AIMRT_ASSERT(ret, "Register publish type failed.");
}

}  // namespace aimrt::plugins::topic_logger_plugin