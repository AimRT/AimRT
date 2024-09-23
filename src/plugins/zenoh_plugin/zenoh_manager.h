// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "zenoh.h"
#include "zenoh_plugin/global.h"

namespace aimrt::plugins::zenoh_plugin {
class ZenohManager {
 public:
  using MsgHandleFunc = std::function<void(const z_loaned_sample_t* message)>;

  ZenohManager() = default;
  ~ZenohManager() = default;

  ZenohManager(const ZenohManager&) = delete;
  ZenohManager& operator=(const ZenohManager&) = delete;

  void Initialize();
  void Shutdown();

  void RegisterSubscriber(const std::string& url, MsgHandleFunc handle);
  void RegisterPublisher(const std::string& url);

  void Publish(const std::string& url, char* serialized_data_ptr, uint64_t serialized_data_len);

 private:
  std::unordered_map<std::string, z_owned_publisher_t> z_pub_registry_;
  std::unordered_map<std::string, z_owned_subscriber_t> z_sub_registry_;
  std::vector<std::shared_ptr<MsgHandleFunc>> msg_handle_vec_;

  z_publisher_put_options_t z_pub_options_;
  z_owned_session_t z_session_;
  z_owned_config_t z_config_;
};

}  // namespace aimrt::plugins::zenoh_plugin