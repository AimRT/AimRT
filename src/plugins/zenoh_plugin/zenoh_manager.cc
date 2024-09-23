// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "zenoh_plugin/zenoh_manager.h"

namespace aimrt::plugins::zenoh_plugin {

void ZenohManager::Initialize() {
  z_config_default(&z_config_);

  if (z_open(&z_session_, z_move(z_config_)) < 0) {
    AIMRT_ERROR("Unable to open zenoh session!");
    return;
  }
}

void ZenohManager::Shutdown() {
  for (auto ptr : z_pub_registry_) {
    z_undeclare_publisher(z_move(ptr.second));
  }

  for (auto ptr : z_sub_registry_) {
    z_undeclare_subscriber(z_move(ptr.second));
  }

  msg_handle_vec_.clear();
  z_pub_registry_.clear();
  z_sub_registry_.clear();

  z_close(z_move(z_session_));
}

void ZenohManager::RegisterPublisher(const std::string &keyexpr) {
  z_view_keyexpr_t key;
  z_view_keyexpr_from_str(&key, keyexpr.c_str());

  z_owned_publisher_t z_pub;
  z_publisher_put_options_default(&z_pub_options_);

  if (z_declare_publisher(&z_pub, z_loan(z_session_), z_loan(key), NULL) < 0) {
    AIMRT_ERROR("Unable to declare Publisher!");
    return;
  }

  z_pub_registry_.emplace(keyexpr, z_pub);
  AIMRT_TRACE("Publisher with keyexpr: {} registered successfully.", keyexpr.c_str());

  return;
}

void ZenohManager::RegisterSubscriber(const std::string &keyexpr, MsgHandleFunc handle) {
  z_view_keyexpr_t key;
  z_view_keyexpr_from_str(&key, keyexpr.c_str());
  z_owned_closure_sample_t z_callback;
  auto function_ptr = std::make_shared<MsgHandleFunc>(std::move(handle));

  z_closure(
      &z_callback,
      [](const z_loaned_sample_t *sample, void *arg) { (*reinterpret_cast<MsgHandleFunc *>(arg))(sample); },

      nullptr,

      function_ptr.get());

  msg_handle_vec_.emplace_back(std::move(function_ptr));

  z_owned_subscriber_t z_sub;

  if (z_declare_subscriber(&z_sub, z_loan(z_session_), z_loan(key), z_move(z_callback), NULL) < 0) {
    AIMRT_ERROR("Unable to declare Subscriber!");
    return;
  }

  z_sub_registry_.emplace(keyexpr, z_sub);
  AIMRT_TRACE("Subscriber with keyexpr: {} registered successfully.", keyexpr.c_str());

  return;
}

void ZenohManager::Publish(const std::string &topic, char *serialized_data_ptr, uint64_t serialized_data_len) {
  auto z_pub_iter = z_pub_registry_.find(topic);
  if (z_pub_iter == z_pub_registry_.end()) {
    AIMRT_ERROR("Url: {} is not registered!", topic);
    return;
  }

  z_owned_bytes_t z_payload;

  z_bytes_from_buf(&z_payload, reinterpret_cast<uint8_t *>(serialized_data_ptr), serialized_data_len, NULL, NULL);
  z_publisher_put(z_loan(z_pub_iter->second), z_move(z_payload), &z_pub_options_);

  return;
}

}  // namespace aimrt::plugins::zenoh_plugin