// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "core/channel/channel_registry.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::runtime::core::channel {

class ChannelBackendBase {
 public:
  ChannelBackendBase() = default;
  virtual ~ChannelBackendBase() = default;

  ChannelBackendBase(const ChannelBackendBase&) = delete;
  ChannelBackendBase& operator=(const ChannelBackendBase&) = delete;

  virtual std::string_view Name() const noexcept = 0;  // It should always return the same value

  virtual void Initialize(YAML::Node options_node) = 0;
  virtual void Start() = 0;
  virtual void Shutdown() = 0;

  virtual std::list<std::pair<std::string, std::string>> GenInitializationReport() const noexcept { return {}; }

  /**
   * @brief Set the Channel Registry to backend
   * @note
   * 1. This method will only be called once before 'Initialize'.
   *
   * @param channel_registry_ptr
   */
  virtual void SetChannelRegistry(const ChannelRegistry* channel_registry_ptr) noexcept {}

  /**
   * @brief Register publish type
   * @note
   * 1. This method will only be called after 'Initialize' and before 'Start'.
   *
   * @param publish_type_wrapper
   * @return Register result
   */
  virtual bool RegisterPublishType(
      const PublishTypeWrapper& publish_type_wrapper) noexcept = 0;

  /**
   * @brief Subscribe
   * @note
   * 1. This method will only be called after 'Initialize' and before 'Start'.
   *
   * @param subscribe_wrapper
   * @return Subscribe result
   */
  virtual bool Subscribe(const SubscribeWrapper& subscribe_wrapper) noexcept = 0;

  /**
   * @brief Publish
   * @note
   * 1. This method will only be called after 'Start' and before 'Shutdown'.
   *
   * @param publish_wrapper
   */
  virtual void Publish(MsgWrapper& msg_wrapper) noexcept = 0;
};

}  // namespace aimrt::runtime::core::channel
