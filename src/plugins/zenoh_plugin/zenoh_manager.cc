// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "zenoh_plugin/zenoh_manager.h"

#include <utility>

namespace aimrt::plugins::zenoh_plugin {

void ZenohManager::Initialize(const std::string &native_cfg_path, size_t shm_pool_size) {
  if (!native_cfg_path.empty() && native_cfg_path.c_str() != nullptr) {
    if (zc_config_from_file(&z_config_, native_cfg_path.c_str()) != Z_OK) {
      AIMRT_ERROR("Unable to load configuration file: {}", native_cfg_path);
      return;
    }
    PrintZenohCgf(z_config_);
  } else {
    z_config_default(&z_config_);
  }

  if (z_open(&z_session_, z_move(z_config_), NULL) < 0) {
    AIMRT_ERROR("Unable to open zenoh session!");
    return;
  }

  shm_pool_size_ = shm_pool_size;
}

void ZenohManager::Shutdown() {
  for (const auto &ptr : z_pub_registry_) {
    auto z_pub = ptr.second->z_pub;
    z_drop(z_move(z_pub));
  }

  for (auto ptr : z_sub_registry_) {
    z_drop(z_move(ptr.second));
  }

  msg_handle_vec_.clear();
  z_pub_registry_.clear();
  z_sub_registry_.clear();

  z_drop(z_move(z_session_));
  z_drop(z_move(shm_provider_));
  z_drop(z_move(shm_layout_));
}

void ZenohManager::RegisterPublisher(const std::string &keyexpr, bool shm_enabled) {
  z_view_keyexpr_t key;
  z_view_keyexpr_from_str(&key, keyexpr.c_str());

  z_owned_publisher_t z_pub;
  z_publisher_put_options_default(&z_pub_options_);

  if (z_declare_publisher(z_loan(z_session_), &z_pub, z_loan(key), nullptr) < 0) {
    AIMRT_ERROR("Unable to declare Publisher!");
    return;
  }

  std::lock_guard<std::mutex> lock(z_registry_mutex_);

  z_pub_registry_.emplace(keyexpr, std::make_shared<ZenohPubCtx>(ZenohPubCtx{z_pub, shm_enabled}));
  AIMRT_TRACE("Publisher with keyexpr: {} registered successfully.", keyexpr.c_str());

  // Create shared memory provider
  if (!shm_initialized_.load() && shm_enabled) {
    z_memory_layout_new(&shm_layout_, shm_pool_size_, alignment_);
    z_posix_shm_provider_new(&shm_provider_, z_loan(shm_layout_));
    shm_initialized_.store(true);
  }
}

void ZenohManager::RegisterSubscriber(const std::string &keyexpr, MsgHandleFunc handle) {
  z_view_keyexpr_t key;
  z_view_keyexpr_from_str(&key, keyexpr.c_str());
  z_owned_closure_sample_t z_callback;
  auto function_ptr = std::make_shared<MsgHandleFunc>(std::move(handle));
  z_closure(
      &z_callback,
      [](z_loaned_sample_t *sample, void *arg) { (*reinterpret_cast<MsgHandleFunc *>(arg))(sample); },

      nullptr,

      function_ptr.get());

  z_owned_subscriber_t z_sub;

  if (z_declare_subscriber(z_loan(z_session_), &z_sub, z_loan(key), z_move(z_callback), nullptr) < 0) {
    AIMRT_ERROR("Unable to declare Subscriber!");
    return;
  }

  std::lock_guard<std::mutex> lock(z_registry_mutex_);

  z_sub_registry_.emplace(keyexpr, z_sub);
  msg_handle_vec_.emplace_back(std::move(function_ptr));

  AIMRT_TRACE("Subscriber with keyexpr: {} registered successfully.", keyexpr.c_str());
}

void ZenohManager::RegisterRpcNode(const std::string &keyexpr, MsgHandleFunc handle, const std::string &role, bool shm_enabled) {
  std::string pub_keyexpr;
  std::string sub_keyexpr;

  if (role == "client") {
    pub_keyexpr = "req/" + keyexpr;
    sub_keyexpr = "rsp/" + keyexpr;
  } else if (role == "server") {
    pub_keyexpr = "rsp/" + keyexpr;
    sub_keyexpr = "req/" + keyexpr;
  } else {
    AIMRT_ERROR("Invalid role: {}", role);
    return;
  }

  RegisterPublisher(pub_keyexpr, shm_enabled);
  RegisterSubscriber(sub_keyexpr, std::move(handle));
  AIMRT_INFO("{} with keyexpr: {} ,role: {} , shm_enabled: {}  is registered successfully.", role, keyexpr.c_str(), role, shm_enabled);
}

void ZenohManager::Publish(const std::string &topic, char *serialized_data_ptr, uint64_t serialized_data_len) {
  auto z_pub_iter = z_pub_registry_.find(topic);
  if (z_pub_iter == z_pub_registry_.end()) [[unlikely]] {
    AIMRT_ERROR("Url: {} is not registered!", topic);
    return;
  }

  z_owned_bytes_t z_payload;

  z_bytes_from_buf(&z_payload, reinterpret_cast<uint8_t *>(serialized_data_ptr), serialized_data_len, nullptr, nullptr);
  z_publisher_put(z_loan(z_pub_iter->second->z_pub), z_move(z_payload), &z_pub_options_);
}

}  // namespace aimrt::plugins::zenoh_plugin