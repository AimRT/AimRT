// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mqtt_plugin/mqtt_plugin.h"

#include <future>

#include "core/aimrt_core.h"
#include "mqtt_plugin/global.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::mqtt_plugin::MqttPlugin::Options> {
  using Options = aimrt::plugins::mqtt_plugin::MqttPlugin::Options;

  static Node encode(const Options &rhs) {
    Node node;

    node["broker_addr"] = rhs.broker_addr;
    node["client_id"] = rhs.client_id;
    node["max_pkg_size_k"] = rhs.max_pkg_size_k;

    return node;
  }

  static bool decode(const Node &node, Options &rhs) {
    if (!node.IsMap()) return false;

    rhs.broker_addr = node["broker_addr"].as<std::string>();
    rhs.client_id = node["client_id"].as<std::string>();

    if (node["max_pkg_size_k"])
      rhs.max_pkg_size_k = node["max_pkg_size_k"].as<uint32_t>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::mqtt_plugin {

bool MqttPlugin::Initialize(runtime::core::AimRTCore *core_ptr) noexcept {
  try {
    core_ptr_ = core_ptr;

    YAML::Node plugin_options_node = core_ptr_->GetPluginManager().GetPluginOptionsNode(Name());

    if (plugin_options_node && !plugin_options_node.IsNull()) {
      options_ = plugin_options_node.as<Options>();
    }

    init_flag_ = true;

    // 初始化mqtt
    MQTTAsync_create(
        &client_, options_.broker_addr.c_str(), options_.client_id.c_str(), MQTTCLIENT_PERSISTENCE_NONE, NULL);

    MQTTAsync_setCallbacks(
        client_,
        this,
        [](void *context, char *cause) {
          static_cast<MqttPlugin *>(context)->OnConnectLost(cause);
        },
        [](void *context, char *topicName, int topicLen, MQTTAsync_message *message) -> int {
          return static_cast<MqttPlugin *>(context)->OnMsgRecv(topicName, topicLen, message);
        },
        NULL);

    // connect to broker
    std::promise<bool> connect_ret_promise;

    MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    conn_opts.onSuccess = [](void *context, MQTTAsync_successData *response) {
      static_cast<std::promise<bool> *>(context)->set_value(true);
    };
    conn_opts.onFailure = [](void *context, MQTTAsync_failureData *response) {
      AIMRT_ERROR("Failed to connect mqtt broker, code: {}, msg: {}",
                  response->code, response->message);
      static_cast<std::promise<bool> *>(context)->set_value(false);
    };
    conn_opts.context = &connect_ret_promise;
    int rc = MQTTAsync_connect(client_, &conn_opts);
    AIMRT_CHECK_ERROR_THROW(rc == MQTTASYNC_SUCCESS, "Failed to connect mqtt broker, return code: {}", rc);

    bool connect_ret = connect_ret_promise.get_future().get();
    AIMRT_CHECK_ERROR_THROW(connect_ret, "Failed to connect mqtt broker");

    msg_handle_registry_ptr_ = std::make_shared<MsgHandleRegistry>();

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostInitLog,
                                [this] { SetPluginLogger(); });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitRpc,
                                [this] { RegisterMqttRpcBackend(); });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitChannel,
                                [this] { RegisterMqttChannelBackend(); });

    plugin_options_node = options_;
    return true;
  } catch (const std::exception &e) {
    AIMRT_ERROR("Initialize failed, {}", e.what());
  }

  return false;
}

void MqttPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;

    stop_flag_ = true;

    msg_handle_registry_ptr_->Shutdown();

    MQTTAsync_disconnect(client_, NULL);

    MQTTAsync_destroy(&client_);

  } catch (const std::exception &e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
  }
}

void MqttPlugin::SetPluginLogger() {
  SetLogger(aimrt::logger::LoggerRef(
      core_ptr_->GetLoggerManager().GetLoggerProxy().NativeHandle()));
}

void MqttPlugin::RegisterMqttChannelBackend() {
  std::unique_ptr<runtime::core::channel::ChannelBackendBase> mqtt_channel_backend_ptr =
      std::make_unique<MqttChannelBackend>(
          client_,
          options_.max_pkg_size_k * 1024,
          msg_handle_registry_ptr_);

  reconnect_hook_.emplace_back(
      [ptr = static_cast<MqttChannelBackend *>(mqtt_channel_backend_ptr.get())]() {
        ptr->SubscribeMqttTopic();
      });

  core_ptr_->GetChannelManager().RegisterChannelBackend(std::move(mqtt_channel_backend_ptr));
}

void MqttPlugin::RegisterMqttRpcBackend() {
  std::unique_ptr<runtime::core::rpc::RpcBackendBase> mqtt_rpc_backend_ptr =
      std::make_unique<MqttRpcBackend>(
          options_.client_id, client_,
          options_.max_pkg_size_k * 1024,
          msg_handle_registry_ptr_);

  static_cast<MqttRpcBackend *>(mqtt_rpc_backend_ptr.get())
      ->RegisterGetExecutorFunc(
          [this](std::string_view executor_name) -> aimrt::executor::ExecutorRef {
            return core_ptr_->GetExecutorManager().GetExecutor(executor_name);
          });

  reconnect_hook_.emplace_back(
      [ptr = static_cast<MqttRpcBackend *>(mqtt_rpc_backend_ptr.get())]() {
        ptr->SubscribeMqttTopic();
      });

  core_ptr_->GetRpcManager().RegisterRpcBackend(std::move(mqtt_rpc_backend_ptr));
}

void MqttPlugin::OnConnectLost(const char *cause) {
  AIMRT_WARN("Lost connect to mqtt broker, cause {}", (cause == nullptr) ? "nil" : cause);

  if (stop_flag_.load()) return;

  MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
  conn_opts.keepAliveInterval = 20;
  conn_opts.cleansession = 1;
  conn_opts.onSuccess = [](void *context, MQTTAsync_successData *response) {
    AIMRT_INFO("Reconnect to mqtt broker success.");

    auto *mqtt_plugin_ptr = static_cast<MqttPlugin *>(context);

    for (const auto &f : mqtt_plugin_ptr->reconnect_hook_)
      f();
  };
  conn_opts.onFailure = [](void *context, MQTTAsync_failureData *response) {
    static_cast<MqttPlugin *>(context)->OnConnectLost("Reconnect failed");
  };
  conn_opts.context = this;
  int rc = MQTTAsync_connect(client_, &conn_opts);

  if (rc != MQTTASYNC_SUCCESS) {
    AIMRT_ERROR("Failed to connect mqtt broker, return code: {}", rc);
    OnConnectLost("Reconnect failed");  // TODO: 得防止爆栈
  }
}

int MqttPlugin::OnMsgRecv(char *topic, int topic_len, MQTTAsync_message *message) {
  std::string_view topic_str = topic_len ? std::string_view(topic, topic_len) : std::string_view(topic);
  msg_handle_registry_ptr_->HandleServerMsg(topic_str, message);
  MQTTAsync_freeMessage(&message);
  MQTTAsync_free(topic);
  return 1;
}

}  // namespace aimrt::plugins::mqtt_plugin
