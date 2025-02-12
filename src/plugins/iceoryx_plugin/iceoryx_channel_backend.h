// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "core/channel/channel_backend_base.h"
#include "core/channel/channel_backend_tools.h"
#include "iceoryx_plugin/iceoryx_buffer_array_allocator.h"
#include "iceoryx_plugin/iceoryx_manager.h"
#include "util/buffer_util.h"
#include "util/url_encode.h"

#include <shared_mutex>

namespace aimrt::plugins::iceoryx_plugin {

class IceoryxChannelBackend : public runtime::core::channel::ChannelBackendBase {
 public:
  struct Options {
  };

 public:
  IceoryxChannelBackend(
      std::shared_ptr<IceoryxManager>& iceoryx_manager_ptr, uint64_t iox_shm_init_size)
      : iceoryx_manager_ptr_(iceoryx_manager_ptr),
        iox_shm_init_size_(iox_shm_init_size) {}

  ~IceoryxChannelBackend() override {}

  std::string_view Name() const noexcept override { return "iceoryx"; }

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

  std::shared_mutex shared_mtx_;

  std::unordered_map<
      std::string,
      std::unique_ptr<aimrt::runtime::core::channel::SubscribeTool>>
      subscribe_wrapper_map_;
  std::unordered_map<std::string, uint64_t> iox_pub_shm_size_map_;

  std::shared_ptr<IceoryxManager> iceoryx_manager_ptr_;
  uint64_t iox_shm_init_size_;
};

}  // namespace aimrt::plugins::iceoryx_plugin