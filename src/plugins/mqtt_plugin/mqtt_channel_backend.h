// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <set>
#include <utility>

#include "core/channel/channel_backend_base.h"
#include "core/channel/channel_backend_tools.h"
#include "mqtt_plugin/msg_handle_registry.h"
#include "util/light_signal.h"

namespace aimrt::plugins::mqtt_plugin {

class MqttChannelBackend : public runtime::core::channel::ChannelBackendBase {
 public:
  struct Options {
    struct PubTopicOptions {
      std::string topic_name;
      int qos = 2;
    };

    std::vector<PubTopicOptions> pub_topics_options;

    struct SubTopicOptions {
      std::string topic_name;
      int qos = 2;
    };

    std::vector<SubTopicOptions> sub_topics_options;
  };

 public:
  MqttChannelBackend(
      MQTTAsync& client,
      uint32_t max_pkg_size,
      const std::shared_ptr<MsgHandleRegistry>& msg_handle_registry_ptr)
      : client_(client),
        max_pkg_size_(max_pkg_size),
        msg_handle_registry_ptr_(msg_handle_registry_ptr) {}

  ~MqttChannelBackend() override = default;

  std::string_view Name() const noexcept override { return "mqtt"; }

  void Initialize(YAML::Node options_node) override;
  void Start() override;
  void Shutdown() override;

  void SetChannelRegistry(const runtime::core::channel::ChannelRegistry* channel_registry_ptr) noexcept override {
    channel_registry_ptr_ = channel_registry_ptr;
  }

  bool RegisterPublishType(
      const runtime::core::channel::PublishTypeWrapper& publish_type_wrapper) noexcept override;
  bool Subscribe(const runtime::core::channel::SubscribeWrapper& subscribe_wrapper) noexcept override;
  void Publish(runtime::core::channel::MsgWrapper& msg_wrapper) noexcept override;

  void SubscribeMqttTopic();
  void UnSubscribeMqttTopic();

 private:
  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

  Options options_;
  std::atomic<State> state_ = State::kPreInit;

  const runtime::core::channel::ChannelRegistry* channel_registry_ptr_ = nullptr;

  MQTTAsync& client_;
  uint32_t max_pkg_size_;
  std::shared_ptr<MsgHandleRegistry> msg_handle_registry_ptr_;

  struct MqttSubInfo {
    std::string topic;
    int qos;
  };
  std::vector<MqttSubInfo> sub_info_vec_;

  std::unordered_map<
      std::string,
      std::unique_ptr<aimrt::runtime::core::channel::SubscribeTool>>
      subscribe_wrapper_map_;

  struct PubCfgInfo {
    int qos;
  };
  std::unordered_map<std::string_view, PubCfgInfo> pub_cfg_info_map_;
};

}  // namespace aimrt::plugins::mqtt_plugin