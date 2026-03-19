// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>
#include <unordered_map>

#include "MQTTAsync.h"
#include "aimrt_module_cpp_interface/executor/executor.h"
#include "util/string_util.h"

#include "mqtt_plugin/global.h"

namespace aimrt::plugins::mqtt_plugin {

class MsgHandleRegistry {
 public:
  using MsgHandleFunc = std::function<void(MQTTAsync_message* message)>;

  MsgHandleRegistry() = default;
  ~MsgHandleRegistry() = default;

  MsgHandleRegistry(const MsgHandleRegistry&) = delete;
  MsgHandleRegistry& operator=(const MsgHandleRegistry&) = delete;

  template <typename... Args>
    requires std::constructible_from<MsgHandleFunc, Args...>
  void RegisterMsgHandle(std::string_view topic, Args&&... args) {
    msg_handle_map_.emplace(topic, std::forward<Args>(args)...);
  }

  void HandleServerMsg(std::string_view topic, MQTTAsync_message* message) const {
    if (shutdown_flag_.load()) [[unlikely]]
      return;

    AIMRT_TRACE("Mqtt recv msg, topic: {}", topic);

    try {
      auto find_topic_itr = msg_handle_map_.find(topic);
      if (find_topic_itr == msg_handle_map_.end()) [[unlikely]] {
        AIMRT_WARN("Unregisted topic: {}", topic);
        return;
      }

      if (executor_) {
        auto& handler = find_topic_itr->second;

        active_tasks_++;

        // Capture the handler by value [handler]
        executor_.Execute([this, handler, message, topic]() mutable {
          handler(message);
          MQTTAsync_freeMessage(&message);
          MQTTAsync_free(const_cast<char*>(topic.data()));

          if (--active_tasks_ == 0) {
            std::lock_guard<std::mutex> lock(wait_mtx_);
            wait_cv_.notify_all();
          }
        });
        return;
      }
      find_topic_itr->second(message);

    } catch (const std::exception& e) {
      AIMRT_ERROR("Handle msg failed, topic: {}, exception info: {}", topic, e.what());
      return;
    }
  }

  void Shutdown() {
    if (std::atomic_exchange(&shutdown_flag_, true)) return;
  }

  void RegisterGetExecutorFunc(
      const std::function<executor::ExecutorRef(std::string_view)>& get_executor_func) {
    get_executor_func_ = get_executor_func;
  }

  void InitExecutor(std::string_view executor_name) {
    executor_ = get_executor_func_(executor_name);
    AIMRT_CHECK_ERROR_THROW(executor_, "Can not get executor {}.", executor_name);
  }

  void WaitForAllTasks() const {
    if (active_tasks_.load() == 0) return;

    std::unique_lock<std::mutex> lock(wait_mtx_);
    wait_cv_.wait(lock, [this]() {
      return active_tasks_.load() == 0;
    });
  }

 private:
  std::atomic_bool shutdown_flag_ = false;

  mutable std::atomic_int active_tasks_ = 0;
  mutable std::mutex wait_mtx_;
  mutable std::condition_variable wait_cv_;

  using UriMsgHandleMap = std::unordered_map<std::string, MsgHandleFunc, aimrt::common::util::StringHash, std::equal_to<>>;
  UriMsgHandleMap msg_handle_map_;

  std::function<executor::ExecutorRef(std::string_view)> get_executor_func_;
  mutable aimrt::executor::ExecutorRef executor_;
};

}  // namespace aimrt::plugins::mqtt_plugin
