// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>
#include <unordered_map>

#include "util/string_util.h"

#include "MQTTAsync.h"

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

      find_topic_itr->second(message);

    } catch (const std::exception& e) {
      AIMRT_ERROR("Handle msg failed, topic: {}, exception info: {}", topic, e.what());
      return;
    }
  }

  void Shutdown() {
    if (std::atomic_exchange(&shutdown_flag_, true)) return;
  }

 private:
  std::atomic_bool shutdown_flag_ = false;

  using UriMsgHandleMap = std::unordered_map<std::string, MsgHandleFunc, aimrt::common::util::StringHash, std::equal_to<>>;
  UriMsgHandleMap msg_handle_map_;
};

}  // namespace aimrt::plugins::mqtt_plugin
