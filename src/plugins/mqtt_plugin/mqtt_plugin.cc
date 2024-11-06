// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mqtt_plugin/mqtt_plugin.h"

#include <future>

#include "core/aimrt_core.h"
#include "mqtt_plugin/global.h"
#include "util/url_parser.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::mqtt_plugin::MqttPlugin::Options> {
  using Options = aimrt::plugins::mqtt_plugin::MqttPlugin::Options;

  static Node encode(const Options &rhs) {
    Node node;

    node["broker_addr"] = rhs.broker_addr;
    node["client_id"] = rhs.client_id;
    node["max_pkg_size_k"] = rhs.max_pkg_size_k;
    node["truststore"] = rhs.truststore;
    node["client_cert"] = rhs.client_cert;
    node["client_key"] = rhs.client_key;

    return node;
  }

  static bool decode(const Node &node, Options &rhs) {
    if (!node.IsMap()) return false;

    rhs.broker_addr = node["broker_addr"].as<std::string>();
    rhs.client_id = node["client_id"].as<std::string>();

    if (node["max_pkg_size_k"])
      rhs.max_pkg_size_k = node["max_pkg_size_k"].as<uint32_t>();

    if (node["truststore"])
      rhs.truststore = node["truststore"].as<std::string>();

    if (node["client_cert"])
      rhs.client_cert = node["client_cert"].as<std::string>();

    if (node["client_key"])
      rhs.client_key = node["client_key"].as<std::string>();

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

    // initialize mqtt
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

    // connect to broker, which is an async operation
    Connect();

    msg_handle_registry_ptr_ = std::make_shared<MsgHandleRegistry>();

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostInitLog,
                                [this] { SetPluginLogger(); });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitRpc,
                                [this] { RegisterMqttRpcBackend(); });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitChannel,
                                [this] { RegisterMqttChannelBackend(); });

    plugin_options_node = options_;
    core_ptr_->GetPluginManager().UpdatePluginOptionsNode(Name(), plugin_options_node);

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

void MqttPlugin::Connect() {
  if (stop_flag_.load()) return;

  // connect to broker
  MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
  MQTTAsync_SSLOptions ssl_opts = MQTTAsync_SSLOptions_initializer;

  conn_opts.keepAliveInterval = 20;
  conn_opts.cleansession = 1;

  // check broker_add protocol is ssl/mqtts or not
  auto ret = common::util::ParseUrl(options_.broker_addr);
  AIMRT_CHECK_ERROR_THROW(ret != std::nullopt, "Parse broker_addr failed");

  // check if need to set ssl options
  if (ret->protocol == "ssl" || ret->protocol == "mqtts") {
    SetSSL(options_, conn_opts, ssl_opts);
  } else {
    AIMRT_CHECK_WARN(options_.truststore.empty(), "Broker protocol is not ssl/mqtts, the truststore you set will be ignored.");
    AIMRT_CHECK_WARN(options_.client_cert.empty(), "Broker protocol is not ssl/mqtts, the client_cert you set will be ignored.");
    AIMRT_CHECK_WARN(options_.client_key.empty(), "Broker protocol is not ssl/mqtts, the client_key you set will be ignored.");
  }

  // if connect success, call all registered hook functions to subscribe mqtt topic
  conn_opts.onSuccess = [](void *context, MQTTAsync_successData *response) {
    AIMRT_INFO("Connect to mqtt broker success.");
    auto *mqtt_plugin_ptr = static_cast<MqttPlugin *>(context);

    for (const auto &f : mqtt_plugin_ptr->reconnect_hook_)
      f();
  };

  // if connect failed, call connect again
  conn_opts.onFailure = [](void *context, MQTTAsync_failureData *response) {
    AIMRT_WARN("Failed to connect mqtt broker, code: {}, msg: {}",
               response->code, response->message);
    static_cast<MqttPlugin *>(context)->Connect();
  };
  conn_opts.context = this;
  int rc = MQTTAsync_connect(client_, &conn_opts);

  if (rc != MQTTASYNC_SUCCESS) {
    AIMRT_ERROR("Failed to start connection, rc: {}", rc);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    Connect();  // todo: avoid stack overflow
  }
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
    AIMRT_ERROR("Failed to reconnect mqtt broker, return code: {}", rc);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

void MqttPlugin::SetSSL(const Options &options, MQTTAsync_connectOptions &conn_opts, MQTTAsync_SSLOptions &ssl_opts) const {
  // check if set ca file
  AIMRT_CHECK_ERROR_THROW(!options_.truststore.empty(), "Use ssl/mqtts must set truststore");
  ssl_opts.trustStore = options_.truststore.c_str();

  bool has_cert = !options_.client_cert.empty();
  bool has_key = !options_.client_key.empty();

  // client_cert and client_key must be set together ,which means use double authentication
  if (has_cert || has_key) {
    AIMRT_CHECK_ERROR_THROW(has_cert && has_key,
                            "When using client certificate authentication, both cert_path and key_path must be set");

    // set client certificate and key
    ssl_opts.keyStore = options_.client_cert.c_str();
    ssl_opts.privateKey = options_.client_key.c_str();
  }

  conn_opts.ssl = &ssl_opts;
}

}  // namespace aimrt::plugins::mqtt_plugin
