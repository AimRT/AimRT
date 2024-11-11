// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "core/channel/channel_backend_base.h"
#include "core/channel/channel_backend_tools.h"
#include "util/buffer_util.h"
#include "util/url_encode.h"
#include "zenoh.h"
#include "zenoh_plugin/zenoh_manager.h"

namespace aimrt::plugins::zenoh_plugin {

class ZenohChannelBackend : public runtime::core::channel::ChannelBackendBase {
 public:
  struct Options {
    struct PubTopicOptions {
      std::string topic_name;
      bool shm_enabled = false;
    };
    std::vector<PubTopicOptions> pub_topics_options;
  };

 public:
  ZenohChannelBackend(
      const std::shared_ptr<ZenohManager>& zenoh_util_ptr, const std::string& limit_domain)
      : zenoh_manager_ptr_(zenoh_util_ptr),
        limit_domain_(limit_domain) {}

  ~ZenohChannelBackend() override = default;

  std::string_view Name() const noexcept override { return "zenoh"; }

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

  std::shared_ptr<ZenohManager> zenoh_manager_ptr_;
  std::string limit_domain_;

  std::unordered_map<
      std::string,
      std::unique_ptr<aimrt::runtime::core::channel::SubscribeTool>>
      subscribe_wrapper_map_;
};

}  // namespace aimrt::plugins::zenoh_plugin