// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "json/json.h"
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

  void Initialize(const std::string& native_cfg_path, size_t shm_pool_size);
  void Shutdown();

  void RegisterSubscriber(const std::string& keyexpr, MsgHandleFunc handle);
  void RegisterPublisher(const std::string& keyexpr, bool shm_enabled);

  void RegisterRpcNode(const std::string& keyexpr, MsgHandleFunc handle, const std::string& role, bool shm_enabled);

  void Publish(const std::string& topic, char* serialized_data_ptr, uint64_t serialized_data_len);

  std::shared_ptr<std::unordered_map<std::string, std::pair<z_owned_publisher_t, bool>>> GetPublisherRegisterMap() {
    return std::make_shared<std::unordered_map<std::string, std::pair<z_owned_publisher_t, bool>>>(z_pub_registry_);
  }

 public:
  z_owned_shm_provider_t shm_provider_;
  z_alloc_alignment_t alignment_ = {0};
  z_publisher_put_options_t z_pub_options_;

 private:
  static void PrintZenohCgf(z_owned_config_t z_config) {
    z_owned_string_t out_config_string;
    zc_config_to_string(z_loan(z_config), &out_config_string);

    Json::CharReaderBuilder reader_builder;
    Json::Value json_data;

    std::istringstream s(z_string_data(z_loan(out_config_string)));
    Json::parseFromStream(reader_builder, s, &json_data, nullptr);
    Json::StreamWriterBuilder writer_builder;
    writer_builder["indentation"] = "    ";
    std::string pretty_json = Json::writeString(writer_builder, json_data);

    AIMRT_INFO("Using custom zenoh native configuration: {}", pretty_json);

    z_drop(z_move(out_config_string));
  }

  std::unordered_map<std::string, std::pair<z_owned_publisher_t, bool>> z_pub_registry_;
  std::unordered_map<std::string, z_owned_subscriber_t> z_sub_registry_;

  std::vector<std::shared_ptr<MsgHandleFunc>> msg_handle_vec_;

  z_owned_session_t z_session_;
  z_owned_config_t z_config_;

  // shm related
  size_t shm_pool_size_;
  z_owned_memory_layout_t shm_layout_;

  std::atomic_bool shm_initialized_ = false;

  std::mutex z_mutex_;
};

}  // namespace aimrt::plugins::zenoh_plugin