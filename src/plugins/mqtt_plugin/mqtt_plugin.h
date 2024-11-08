// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <memory>

#include "MQTTAsync.h"

#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"
#include "mqtt_plugin/mqtt_channel_backend.h"
#include "mqtt_plugin/mqtt_rpc_backend.h"
#include "mqtt_plugin/msg_handle_registry.h"

namespace aimrt::plugins::mqtt_plugin {

class MqttPlugin : public AimRTCorePluginBase {
 public:
  struct Options {
    std::string broker_addr;
    std::string client_id;
    uint32_t max_pkg_size_k = 1024;
    uint32_t reconnect_interval_ms = 1000;
    std::string truststore;
    std::string client_cert;
    std::string client_key;
    std::string client_key_password;
  };

 public:
  MqttPlugin() = default;
  ~MqttPlugin() override = default;

  std::string_view Name() const noexcept override { return "mqtt_plugin"; }

  bool Initialize(runtime::core::AimRTCore *core_ptr) noexcept override;
  void Shutdown() noexcept override;

 private:
  void SetPluginLogger();
  void SetSSL(MQTTAsync_connectOptions &conn_opts, MQTTAsync_SSLOptions &ssl_opts) const;
  void RegisterMqttRpcBackend();
  void RegisterMqttChannelBackend();

  void AsyncConnect();
  void OnConnectLost(const char *cause);
  int OnMsgRecv(char *topic, int topic_len, MQTTAsync_message *message);

 private:
  runtime::core::AimRTCore *core_ptr_ = nullptr;

  Options options_;

  bool init_flag_ = false;

  std::atomic_bool stop_flag_ = false;

  MQTTAsync client_;
  std::shared_ptr<MsgHandleRegistry> msg_handle_registry_ptr_;

  std::vector<std::function<void()>> reconnect_hook_;

  std::condition_variable cv_;
  std::mutex cv_mutex_;
  std::atomic_bool notified_ = false;
};

}  // namespace aimrt::plugins::mqtt_plugin
