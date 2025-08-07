// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mqtt_plugin/mqtt_rpc_backend.h"

#include <regex>

#include "aimrt_module_cpp_interface/rpc/rpc_handle.h"
#include "aimrt_module_cpp_interface/rpc/rpc_status.h"
#include "aimrt_module_cpp_interface/util/type_support.h"
#include "core/rpc/rpc_backend_tools.h"
#include "mqtt_plugin/global.h"
#include "util/buffer_util.h"
#include "util/url_encode.h"
#include "util/url_parser.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::mqtt_plugin::MqttRpcBackend::Options> {
  using Options = aimrt::plugins::mqtt_plugin::MqttRpcBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["timeout_executor"] = rhs.timeout_executor;

    node["clients_options"] = YAML::Node();
    for (const auto& client_options : rhs.clients_options) {
      Node client_options_node;
      client_options_node["func_name"] = client_options.func_name;
      client_options_node["server_mqtt_id"] = client_options.server_mqtt_id;
      client_options_node["qos"] = client_options.qos;
      node["clients_options"].push_back(client_options_node);
    }

    node["servers_options"] = YAML::Node();
    for (const auto& server_options : rhs.servers_options) {
      Node server_options_node;
      server_options_node["func_name"] = server_options.func_name;
      server_options_node["allow_share"] = server_options.allow_share;
      server_options_node["qos"] = server_options.qos;
      node["servers_options"].push_back(server_options_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (node["timeout_executor"])
      rhs.timeout_executor = node["timeout_executor"].as<std::string>();

    if (node["clients_options"] && node["clients_options"].IsSequence()) {
      for (const auto& client_options_node : node["clients_options"]) {
        auto client_options = Options::ClientOptions{
            .func_name = client_options_node["func_name"].as<std::string>()};

        if (client_options_node["server_mqtt_id"])
          client_options.server_mqtt_id = client_options_node["server_mqtt_id"].as<std::string>();

        int qos = 2;
        if (client_options_node["qos"]) qos = client_options_node["qos"].as<int>();
        AIMRT_ASSERT(qos >= 0 && qos <= 2, "Invalid Mqtt qos: {}", qos);
        client_options.qos = qos;

        rhs.clients_options.emplace_back(std::move(client_options));
      }
    }

    if (node["servers_options"] && node["servers_options"].IsSequence()) {
      for (const auto& server_options_node : node["servers_options"]) {
        auto server_options = Options::ServerOptions{
            .func_name = server_options_node["func_name"].as<std::string>()};

        if (server_options_node["allow_share"])
          server_options.allow_share = server_options_node["allow_share"].as<bool>();

        int qos = 2;
        if (server_options_node["qos"]) qos = server_options_node["qos"].as<int>();
        AIMRT_ASSERT(qos >= 0 && qos <= 2, "Invalid Mqtt qos: {}", qos);

        server_options.qos = qos;

        rhs.servers_options.emplace_back(std::move(server_options));
      }
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::mqtt_plugin {

void MqttRpcBackend::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Mqtt Rpc backend can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  client_tool_ptr_ =
      std::make_unique<runtime::core::util::RpcClientTool<std::shared_ptr<runtime::core::rpc::InvokeWrapper>>>();

  if (!options_.timeout_executor.empty()) {
    AIMRT_CHECK_ERROR_THROW(
        get_executor_func_,
        "Get executor function is not set before initialize.");

    auto timeout_executor = get_executor_func_(options_.timeout_executor);

    AIMRT_CHECK_ERROR_THROW(
        timeout_executor,
        "Get timeout executor '{}' failed.", options_.timeout_executor);

    client_tool_ptr_->RegisterTimeoutExecutor(timeout_executor);
    client_tool_ptr_->RegisterTimeoutHandle(
        [](auto&& client_invoke_wrapper_ptr) {
          client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_TIMEOUT));
        });

    AIMRT_TRACE("Mqtt rpc backend enable the timeout function, use '{}' as timeout executor.",
                options_.timeout_executor);
  } else {
    AIMRT_TRACE("Mqtt rpc backend does not enable the timeout function.");
  }

  options_node = options_;
}

void MqttRpcBackend::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");

  // Wait a moment  for the connection to be established
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}
void MqttRpcBackend::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  UnSubscribeMqttTopic();

  client_tool_ptr_.reset();

  msg_handle_registry_ptr_->Shutdown();
}

bool MqttRpcBackend::RegisterServiceFunc(
    const runtime::core::rpc::ServiceFuncWrapper& service_func_wrapper) noexcept {
  try {
    if (state_.load() != State::kInit) {
      AIMRT_ERROR("Service func can only be registered when state is 'Init'.");
      return false;
    }

    namespace util = aimrt::common::util;

    const auto& info = service_func_wrapper.info;

    bool allow_share = true;
    int qos = 2;

    auto find_option_itr = std::find_if(
        options_.servers_options.begin(), options_.servers_options.end(),
        [func_name = info.func_name](const Options::ServerOptions& server_option) {
          try {
            auto real_func_name = std::string(rpc::GetFuncNameWithoutPrefix(func_name));
            return std::regex_match(func_name.begin(), func_name.end(),
                                    std::regex(server_option.func_name, std::regex::ECMAScript)) ||
                   std::regex_match(real_func_name.begin(), real_func_name.end(),
                                    std::regex(server_option.func_name, std::regex::ECMAScript));
          } catch (const std::exception& e) {
            AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                       server_option.func_name, func_name, e.what());
            return false;
          }
        });

    if (find_option_itr != options_.servers_options.end()) {
      allow_share = find_option_itr->allow_share;
      qos = find_option_itr->qos;
    }

    auto handle = [this, qos, &service_func_wrapper](MQTTAsync_message* message) {
      try {
        // Create a service invoke wrapper
        auto service_invoke_wrapper_ptr = std::make_shared<runtime::core::rpc::InvokeWrapper>(
            runtime::core::rpc::InvokeWrapper{.info = service_func_wrapper.info});
        const auto& info = service_invoke_wrapper_ptr->info;

        // Create service ctx
        auto ctx_ptr = std::make_shared<aimrt::rpc::Context>(aimrt_rpc_context_type_t::AIMRT_RPC_SERVER_CONTEXT);
        service_invoke_wrapper_ptr->ctx_ref = ctx_ptr;

        // Get fields
        util::ConstBufferOperator buf_oper(static_cast<const char*>(message->payload), message->payloadlen);

        std::string serialization_type(buf_oper.GetString(util::BufferLenType::kUInt8));
        ctx_ptr->SetMetaValue(AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE, serialization_type);

        std::string mqtt_pub_topic(buf_oper.GetString(util::BufferLenType::kUInt8));

        char req_id_buf[4];
        buf_oper.GetBuffer(req_id_buf, 4);

        // Get context
        size_t ctx_num = buf_oper.GetUint8();
        for (size_t ii = 0; ii < ctx_num; ++ii) {
          auto key = buf_oper.GetString(util::BufferLenType::kUInt16);
          auto val = buf_oper.GetString(util::BufferLenType::kUInt16);
          ctx_ptr->SetMetaValue(key, val);
        }

        ctx_ptr->SetFunctionName(info.func_name);
        ctx_ptr->SetMetaValue(AIMRT_RPC_CONTEXT_KEY_BACKEND, Name());

        // service req deserialization
        auto remaining_buf = buf_oper.GetRemainingBuffer();
        aimrt_buffer_view_t buffer_view{
            .data = remaining_buf.data(),
            .len = remaining_buf.size()};

        aimrt_buffer_array_view_t buffer_array_view{
            .data = &buffer_view,
            .len = 1};

        std::shared_ptr<void> service_req_ptr = info.req_type_support_ref.CreateSharedPtr();
        service_invoke_wrapper_ptr->req_ptr = service_req_ptr.get();

        bool deserialize_ret = info.req_type_support_ref.Deserialize(
            serialization_type, buffer_array_view, service_req_ptr.get());

        if (!deserialize_ret) [[unlikely]] {
          AIMRT_ERROR("Mqtt req deserialize failed.");

          ReturnRspWithStatusCode(
              mqtt_pub_topic, qos, serialization_type, req_id_buf, AIMRT_RPC_STATUS_SVR_DESERIALIZATION_FAILED);

          return;
        }

        // Create service rsp
        std::shared_ptr<void> service_rsp_ptr = info.rsp_type_support_ref.CreateSharedPtr();
        service_invoke_wrapper_ptr->rsp_ptr = service_rsp_ptr.get();

        // Set callback
        service_invoke_wrapper_ptr->callback =
            [this,
             service_invoke_wrapper_ptr,
             qos,
             ctx_ptr,
             service_req_ptr,
             service_rsp_ptr,
             serialization_type{std::move(serialization_type)},
             mqtt_pub_topic{std::move(mqtt_pub_topic)},
             req_id_buf](aimrt::rpc::Status status) {
              if (!status.OK()) [[unlikely]] {
                // If the code is not suc, then deserialization is not necessary
                ReturnRspWithStatusCode(
                    mqtt_pub_topic, qos, serialization_type, req_id_buf, status.Code());

                return;
              }

              // service rsp serialization
              auto buffer_array_view_ptr = aimrt::runtime::core::rpc::TrySerializeRspWithCache(*service_invoke_wrapper_ptr, serialization_type);
              if (!buffer_array_view_ptr) [[unlikely]] {
                ReturnRspWithStatusCode(
                    mqtt_pub_topic, qos, serialization_type, req_id_buf, AIMRT_RPC_STATUS_SVR_SERIALIZATION_FAILED);

                return;
              }

              const auto* buffer_array_data = buffer_array_view_ptr->Data();
              const size_t buffer_array_len = buffer_array_view_ptr->Size();
              size_t rsp_size = buffer_array_view_ptr->BufferSize();

              size_t mqtt_pkg_size = 1 + serialization_type.size() + 4 + 4 + rsp_size;

              if (mqtt_pkg_size > max_pkg_size_) [[unlikely]] {
                AIMRT_WARN("Mqtt publish failed, pkg is too large, limit {}k, actual {}k",
                           max_pkg_size_ / 1024, mqtt_pkg_size / 1024);

                ReturnRspWithStatusCode(
                    mqtt_pub_topic, qos, serialization_type, req_id_buf, AIMRT_RPC_STATUS_SVR_UNKNOWN);
                return;
              }

              std::vector<char> msg_buf_vec(mqtt_pkg_size);
              util::BufferOperator buf_oper(msg_buf_vec.data(), msg_buf_vec.size());

              buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);
              buf_oper.SetBuffer(req_id_buf, sizeof(req_id_buf));
              buf_oper.SetUint32(0);

              for (size_t ii = 0; ii < buffer_array_len; ++ii) {
                buf_oper.SetBuffer(
                    static_cast<const char*>(buffer_array_data[ii].data),
                    buffer_array_data[ii].len);
              }

              MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
              pubmsg.payload = msg_buf_vec.data();
              pubmsg.payloadlen = msg_buf_vec.size();
              pubmsg.qos = qos;
              pubmsg.retained = 0;

              AIMRT_TRACE("Mqtt publish to '{}'", mqtt_pub_topic);
              int rc = MQTTAsync_sendMessage(client_, mqtt_pub_topic.data(), &pubmsg, NULL);
              AIMRT_CHECK_WARN(rc == MQTTASYNC_SUCCESS,
                               "publish mqtt msg failed, topic: {}, code: {}",
                               mqtt_pub_topic, rc);
            };

        // Call service rpc
        service_func_wrapper.service_func(service_invoke_wrapper_ptr);

      } catch (const std::exception& e) {
        AIMRT_WARN("Handle mqtt rpc msg failed, exception info: {}", e.what());
      }
    };

    // check allow_share
    if (allow_share) {
      std::string mqtt_sub_topic = "aimrt_rpc_req/" + util::UrlEncode(rpc::GetFuncNameWithoutPrefix(info.func_name));
      std::string share_mqtt_sub_topic = "$share/aimrt/" + mqtt_sub_topic;
      sub_info_vec_.emplace_back(MqttSubInfo{share_mqtt_sub_topic, qos});

      msg_handle_registry_ptr_->RegisterMsgHandle(mqtt_sub_topic, handle);
    }

    std::string mqtt_sub_topic_2 =
        "aimrt_rpc_req/" +
        util::UrlEncode(client_id_) + "/" +
        util::UrlEncode(rpc::GetFuncNameWithoutPrefix(info.func_name));
    sub_info_vec_.emplace_back(MqttSubInfo{mqtt_sub_topic_2, qos});

    msg_handle_registry_ptr_->RegisterMsgHandle(mqtt_sub_topic_2, handle);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

bool MqttRpcBackend::RegisterClientFunc(
    const runtime::core::rpc::ClientFuncWrapper& client_func_wrapper) noexcept {
  try {
    if (state_.load() != State::kInit) {
      AIMRT_ERROR("Client func can only be registered when state is 'Init'.");
      return false;
    }

    namespace util = aimrt::common::util;

    const auto& info = client_func_wrapper.info;

    std::string server_mqtt_id;
    int qos = 2;

    auto find_option_itr = std::find_if(
        options_.clients_options.begin(), options_.clients_options.end(),
        [func_name = info.func_name](const Options::ClientOptions& client_option) {
          try {
            auto real_func_name = std::string(rpc::GetFuncNameWithoutPrefix(func_name));
            return std::regex_match(func_name.begin(), func_name.end(),
                                    std::regex(client_option.func_name, std::regex::ECMAScript)) ||
                   std::regex_match(real_func_name.begin(), real_func_name.end(),
                                    std::regex(client_option.func_name, std::regex::ECMAScript));
          } catch (const std::exception& e) {
            AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                       client_option.func_name, func_name, e.what());
            return false;
          }
        });

    if (find_option_itr != options_.clients_options.end()) {
      server_mqtt_id = find_option_itr->server_mqtt_id;
      qos = find_option_itr->qos;
    }

    client_cfg_info_map_.emplace(
        rpc::GetFuncNameWithoutPrefix(info.func_name),
        ClientCfgInfo{
            .server_mqtt_id = server_mqtt_id,
            .qos = qos});

    std::string mqtt_sub_topic =
        "aimrt_rpc_rsp/" +
        util::UrlEncode(client_id_) + "/" +
        util::UrlEncode(rpc::GetFuncNameWithoutPrefix(info.func_name));

    if (mqtt_sub_topic.size() > 255) {
      AIMRT_ERROR("Too long mqtt topic name: {}", mqtt_sub_topic);
      return false;
    }

    sub_info_vec_.emplace_back(MqttSubInfo{mqtt_sub_topic, qos});

    msg_handle_registry_ptr_->RegisterMsgHandle(
        mqtt_sub_topic,
        [this](MQTTAsync_message* message) {
          std::shared_ptr<runtime::core::rpc::InvokeWrapper> client_invoke_wrapper_ptr;

          try {
            util::ConstBufferOperator buf_oper(static_cast<const char*>(message->payload), message->payloadlen);

            std::string serialization_type(buf_oper.GetString(util::BufferLenType::kUInt8));
            uint32_t req_id = buf_oper.GetUint32();
            uint32_t code = buf_oper.GetUint32();

            auto msg_recorder = client_tool_ptr_->GetRecord(req_id);
            if (!msg_recorder) [[unlikely]] {
              // No record is found, which means that the call has timed out. The record has been deleted after the timeout process is gone.
              AIMRT_TRACE("Can not get req id {} from recorder.", req_id);
              return;
            }

            // Get the record
            client_invoke_wrapper_ptr = std::move(*msg_recorder);

            if (code) [[unlikely]] {
              client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(code));
              return;
            }

            const auto& info = client_invoke_wrapper_ptr->info;

            // client rsp deserialization
            auto remaining_buf = buf_oper.GetRemainingBuffer();
            aimrt_buffer_view_t buffer_view{
                .data = remaining_buf.data(),
                .len = remaining_buf.size()};

            aimrt_buffer_array_view_t buffer_array_view{
                .data = &buffer_view,
                .len = 1};

            bool deserialize_ret = info.rsp_type_support_ref.Deserialize(
                serialization_type, buffer_array_view, client_invoke_wrapper_ptr->rsp_ptr);

            if (!deserialize_ret) {
              // Deserialization failed
              client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_DESERIALIZATION_FAILED));
              return;
            }

            client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_OK));
            return;

          } catch (const std::exception& e) {
            AIMRT_WARN("Handle mqtt rpc msg failed, exception info: {}", e.what());
          }

          if (client_invoke_wrapper_ptr)
            client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
        });

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

void MqttRpcBackend::Invoke(
    const std::shared_ptr<runtime::core::rpc::InvokeWrapper>& client_invoke_wrapper_ptr) noexcept {
  try {
    if (state_.load() != State::kStart) [[unlikely]] {
      AIMRT_WARN("Method can only be called when state is 'Start'.");
      InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    namespace util = aimrt::common::util;

    const auto& info = client_invoke_wrapper_ptr->info;

    auto real_func_name = rpc::GetFuncNameWithoutPrefix(info.func_name);

    std::string server_mqtt_id;
    int qos = 2;

    auto find_itr = client_cfg_info_map_.find(real_func_name);
    if (find_itr != client_cfg_info_map_.end()) {
      server_mqtt_id = find_itr->second.server_mqtt_id;
      qos = find_itr->second.qos;
    }

    // url: mqtt://server_mqtt_id
    auto to_addr = client_invoke_wrapper_ptr->ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_TO_ADDR);
    if (!to_addr.empty()) {
      auto url = util::ParseUrl<std::string_view>(to_addr);
      if (url) {
        if (url->protocol != Name()) [[unlikely]] {
          AIMRT_WARN("Invalid addr: {}", to_addr);
          InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
          return;
        }
        server_mqtt_id = url->host;
      }
    }

    uint32_t cur_req_id = req_id_++;

    auto serialization_type =
        client_invoke_wrapper_ptr->ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE);

    if (serialization_type.size() > 255) [[unlikely]] {
      InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_UNKNOWN));
      return;
    }

    // aimrt_rpc_rsp/client_id/uri
    std::string mqtt_sub_topic =
        "aimrt_rpc_rsp/" +
        util::UrlEncode(client_id_) + "/" +
        util::UrlEncode(real_func_name);

    if (mqtt_sub_topic.size() > 255) [[unlikely]] {
      InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_UNKNOWN));
      return;
    }

    // client req serialization
    auto buffer_array_view_ptr = aimrt::runtime::core::rpc::TrySerializeReqWithCache(*client_invoke_wrapper_ptr, serialization_type);
    if (!buffer_array_view_ptr) [[unlikely]] {
      // Serialization failed
      InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_SERIALIZATION_FAILED));
      return;
    }

    const auto* buffer_array_data = buffer_array_view_ptr->Data();
    const size_t buffer_array_len = buffer_array_view_ptr->Size();
    size_t req_size = buffer_array_view_ptr->BufferSize();

    // context
    client_invoke_wrapper_ptr->ctx_ref.SetMetaValue("aimrt-from_mqtt_client", client_id_);
    auto [meta_key_vals_array, meta_key_vals_array_len] = client_invoke_wrapper_ptr->ctx_ref.GetMetaKeyValsArray();
    if (meta_key_vals_array_len / 2 > 255) [[unlikely]] {
      AIMRT_WARN("Too much context meta, require less than 255, but actually {}.", meta_key_vals_array_len / 2);
      InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    size_t context_meta_kv_size = 1;
    for (size_t ii = 0; ii < meta_key_vals_array_len; ++ii) {
      context_meta_kv_size += (2 + meta_key_vals_array[ii].len);
    }

    // Fill mqtt package
    size_t mqtt_pkg_size = 1 + serialization_type.size() +
                           1 + mqtt_sub_topic.size() +
                           4 +
                           context_meta_kv_size +
                           req_size;

    if (mqtt_pkg_size > max_pkg_size_) [[unlikely]] {
      AIMRT_WARN("Mqtt publish failed, pkg is too large, limit {}k, actual {}k",
                 max_pkg_size_ / 1024, mqtt_pkg_size / 1024);

      InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_SEND_REQ_FAILED));
      return;
    }

    // Record the callbacks and other contents
    auto timeout = client_invoke_wrapper_ptr->ctx_ref.Timeout();
    auto record_ptr = client_invoke_wrapper_ptr;

    bool ret = client_tool_ptr_->Record(cur_req_id, timeout, std::move(record_ptr));

    if (!ret) [[unlikely]] {
      AIMRT_ERROR("Failed to record msg.");
      InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    std::vector<char> msg_buf_vec(mqtt_pkg_size);

    util::BufferOperator buf_oper(msg_buf_vec.data(), msg_buf_vec.size());

    buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);
    buf_oper.SetString(mqtt_sub_topic, util::BufferLenType::kUInt8);
    buf_oper.SetUint32(cur_req_id);

    buf_oper.SetUint8(static_cast<uint8_t>(meta_key_vals_array_len / 2));
    for (size_t ii = 0; ii < meta_key_vals_array_len; ++ii) {
      buf_oper.SetString(aimrt::util::ToStdStringView(meta_key_vals_array[ii]), util::BufferLenType::kUInt16);
    }

    // data
    for (size_t ii = 0; ii < buffer_array_len; ++ii) {
      buf_oper.SetBuffer(
          static_cast<const char*>(buffer_array_data[ii].data),
          buffer_array_data[ii].len);
    }

    // send
    MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
    pubmsg.payload = msg_buf_vec.data();
    pubmsg.payloadlen = msg_buf_vec.size();
    pubmsg.qos = qos;
    pubmsg.retained = 0;

    // topic: aimrt_rpc_req/uri
    std::string mqtt_pub_topic;
    if (server_mqtt_id.empty()) {
      mqtt_pub_topic = "aimrt_rpc_req/" + util::UrlEncode(real_func_name);
    } else {
      mqtt_pub_topic =
          "aimrt_rpc_req/" +
          util::UrlEncode(server_mqtt_id) + "/" +
          util::UrlEncode(real_func_name);
    }

    AIMRT_TRACE("Mqtt publish to '{}'", mqtt_pub_topic);
    int rc = MQTTAsync_sendMessage(client_, mqtt_pub_topic.data(), &pubmsg, NULL);
    AIMRT_CHECK_WARN(rc == MQTTASYNC_SUCCESS,
                     "publish mqtt msg failed, topic: {}, code: {}",
                     mqtt_pub_topic, rc);
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

void MqttRpcBackend::RegisterGetExecutorFunc(
    const std::function<aimrt::executor::ExecutorRef(std::string_view)>& get_executor_func) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  get_executor_func_ = get_executor_func;
}

void MqttRpcBackend::SubscribeMqttTopic() {
  for (auto sub_info : sub_info_vec_) {
    // todo:Replace with MQTTClient_subscribeMany
    int rc = MQTTAsync_subscribe(client_, sub_info.topic.data(), sub_info.qos, NULL);
    if (rc != MQTTASYNC_SUCCESS) {
      AIMRT_ERROR("Failed to subscribe mqtt, topic: {} return code: {}", sub_info.topic, rc);
    }
  }
}

void MqttRpcBackend::UnSubscribeMqttTopic() {
  // todo:Replace with MQTTClient_unsubscribeMany
  for (auto sub_info : sub_info_vec_) {
    MQTTAsync_unsubscribe(client_, sub_info.topic.data(), NULL);
  }
}

void MqttRpcBackend::ReturnRspWithStatusCode(
    std::string_view mqtt_pub_topic,
    int qos,
    std::string_view serialization_type,
    const char* req_id_buf,
    uint32_t code) {
  namespace util = aimrt::common::util;

  std::vector<char> msg_buf_vec(1 + serialization_type.size() + 4 + 4);
  util::BufferOperator buf_oper(msg_buf_vec.data(), msg_buf_vec.size());

  buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);
  buf_oper.SetBuffer(req_id_buf, 4);
  buf_oper.SetUint32(code);

  MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
  pubmsg.payload = msg_buf_vec.data();
  pubmsg.payloadlen = msg_buf_vec.size();
  pubmsg.qos = qos;
  pubmsg.retained = 0;

  AIMRT_TRACE("Mqtt publish to '{}'", mqtt_pub_topic);
  int rc = MQTTAsync_sendMessage(client_, mqtt_pub_topic.data(), &pubmsg, NULL);
  AIMRT_CHECK_WARN(rc == MQTTASYNC_SUCCESS,
                   "publish mqtt msg failed, topic: {}, code: {}",
                   mqtt_pub_topic, rc);
}

}  // namespace aimrt::plugins::mqtt_plugin