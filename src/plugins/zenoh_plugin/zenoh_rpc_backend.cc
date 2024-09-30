// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "zenoh_plugin/zenoh_rpc_backend.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::zenoh_plugin::ZenohRpcBackend::Options> {
  using Options = aimrt::plugins::zenoh_plugin::ZenohRpcBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["timeout_executor"] = rhs.timeout_executor;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (node["timeout_executor"])
      rhs.timeout_executor = node["timeout_executor"].as<std::string>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::zenoh_plugin {

void ZenohRpcBackend::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::Init) == State::PreInit,
      "Zenoh Rpc backend can only be initialized once.");

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

    AIMRT_TRACE("zenoh rpc backend enable the timeout function, use '{}' as timeout executor.",
                options_.timeout_executor);
  } else {
    AIMRT_TRACE("zenoh rpc backend does not enable the timeout function.");
  }
}

void ZenohRpcBackend::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::Start) == State::Init,
      "Method can only be called when state is 'Init'.");
}

void ZenohRpcBackend::Shutdown() {
  if (std::atomic_exchange(&state_, State::Shutdown) == State::Shutdown)
    return;

  client_tool_ptr_.reset();
}

bool ZenohRpcBackend::RegisterServiceFunc(
    const runtime::core::rpc::ServiceFuncWrapper& service_func_wrapper) noexcept {
  try {
    if (state_.load() != State::Init) {
      AIMRT_ERROR("Service func can only be registered when state is 'Init'.");
      return false;
    }

    namespace util = aimrt::common::util;
    const auto& info = service_func_wrapper.info;

    std::string pattern = std::string("aimrt_rpc/") +
                          util::UrlEncode(GetRealFuncName(info.func_name)) +
                          limit_domain_;

    auto handle = [this, &service_func_wrapper, pattern](const z_loaned_sample_t* message) {
      try {
        // create service invoke wrapper
        auto service_invoke_wrapper_ptr = std::make_shared<runtime::core::rpc::InvokeWrapper>(
            runtime::core::rpc::InvokeWrapper{.info = service_func_wrapper.info});
        const auto& info = service_invoke_wrapper_ptr->info;

        // create ctx
        auto ctx_ptr = std::make_shared<aimrt::rpc::Context>(aimrt_rpc_context_type_t::AIMRT_RPC_SERVER_CONTEXT);
        service_invoke_wrapper_ptr->ctx_ref = ctx_ptr;

        // read data
        const z_loaned_bytes_t* payload = z_sample_payload(message);
        size_t serialized_size = z_bytes_len(payload);
        z_bytes_reader_t reader = z_bytes_get_reader(payload);
        std::vector<char> serialized_data(serialized_size);

        if (z_bytes_reader_read(&reader, reinterpret_cast<uint8_t*>(serialized_data.data()), serialized_size) >= 0) {
          util::ConstBufferOperator buf_oper(serialized_data.data(), serialized_size);

          // deserialize type
          std::string serialization_type(buf_oper.GetString(util::BufferLenType::kUInt8));
          ctx_ptr->SetMetaValue(AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE, serialization_type);

          std::string pattern(buf_oper.GetString(util::BufferLenType::kUInt8));
          char req_id_buf[4];
          buf_oper.GetBuffer(req_id_buf, 4);

          size_t ctx_num = buf_oper.GetUint8();
          for (size_t ii = 0; ii < ctx_num; ++ii) {
            auto key = buf_oper.GetString(util::BufferLenType::kUInt16);
            auto val = buf_oper.GetString(util::BufferLenType::kUInt16);
            ctx_ptr->SetMetaValue(key, val);
          }
          ctx_ptr->SetFunctionName(info.func_name);
          ctx_ptr->SetMetaValue(AIMRT_RPC_CONTEXT_KEY_BACKEND, Name());

          // deserialize msg
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

          AIMRT_CHECK_ERROR_THROW(deserialize_ret, "Zenoh req deserialize failed.");

          // create service rsp
          std::shared_ptr<void> service_rsp_ptr = info.rsp_type_support_ref.CreateSharedPtr();
          service_invoke_wrapper_ptr->rsp_ptr = service_rsp_ptr.get();

          // set callback
          service_invoke_wrapper_ptr->callback =
              [this,
               service_invoke_wrapper_ptr,
               ctx_ptr,
               service_req_ptr,
               service_rsp_ptr,
               serialization_type{std::move(serialization_type)},
               pattern{std::move(pattern)},
               req_id_buf](aimrt::rpc::Status status) {
                if (!status.OK()) [[unlikely]] {
                  ReturnRspWithStatusCode(
                      pattern, serialization_type, req_id_buf, status.Code());
                  return;
                }

                // serivice rsp serialize
                auto buffer_array_view_ptr = aimrt::runtime::core::rpc::TrySerializeRspWithCache(
                    *service_invoke_wrapper_ptr, serialization_type);
                if (!buffer_array_view_ptr) [[unlikely]] {
                  ReturnRspWithStatusCode(
                      pattern, serialization_type, req_id_buf, AIMRT_RPC_STATUS_SVR_SERIALIZATION_FAILED);

                  return;
                }
                const auto* buffer_array_data = buffer_array_view_ptr->Data();
                const size_t buffer_array_len = buffer_array_view_ptr->Size();
                size_t rsp_size = buffer_array_view_ptr->BufferSize();

                size_t pkg_size = 1 + serialization_type.size() + 4 + 4 + rsp_size;

                std::vector<char> msg_buf_vec(pkg_size);
                util::BufferOperator buf_oper(msg_buf_vec.data(), msg_buf_vec.size());
                buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);
                buf_oper.SetBuffer(req_id_buf, sizeof(req_id_buf));
                buf_oper.SetUint32(0);

                for (size_t ii = 0; ii < buffer_array_len; ++ii) {
                  buf_oper.SetBuffer(
                      static_cast<const char*>(buffer_array_data[ii].data),
                      buffer_array_data[ii].len);
                }

                zenoh_manager_ptr_->Publish("rsp/" + pattern, msg_buf_vec.data(), pkg_size);
              };
          // call service
          service_func_wrapper.service_func(service_invoke_wrapper_ptr);

        } else {
          AIMRT_ERROR("Zenoh Plugin Read payload failed!");
        }
      } catch (const std::exception& e) {
        AIMRT_WARN("Handle zenoh rpc msg failed, exception info: {}", e.what());
      }
    };
    zenoh_manager_ptr_->RegisterRpcNode(pattern, std::move(handle), "server");
    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

bool ZenohRpcBackend::RegisterClientFunc(
    const runtime::core::rpc::ClientFuncWrapper& client_func_wrapper) noexcept {
  try {
    if (state_.load() != State::Init) {
      AIMRT_ERROR("Client func can only be registered when state is 'Init'.");
      return false;
    }

    namespace util = aimrt::common::util;

    const auto& info = client_func_wrapper.info;

    std::string pattern = std::string("aimrt_rpc/") +
                          util::UrlEncode(GetRealFuncName(info.func_name)) +
                          limit_domain_;
    auto handle = [this](const z_loaned_sample_t* message) {
      std::shared_ptr<runtime::core::rpc::InvokeWrapper> client_invoke_wrapper_ptr;
      try {
        // read data
        const z_loaned_bytes_t* payload = z_sample_payload(message);
        size_t serialized_size = z_bytes_len(payload);
        z_bytes_reader_t reader = z_bytes_get_reader(payload);
        std::vector<char> serialized_data(serialized_size);

        auto read_ret = z_bytes_reader_read(&reader, reinterpret_cast<uint8_t*>(serialized_data.data()), serialized_size);
        if (read_ret < 0) {
          AIMRT_ERROR("Zenoh Plugin Read payload failed!");
          return;
        }

        util::ConstBufferOperator buf_oper(serialized_data.data(), serialized_size);

        std::string serialization_type(buf_oper.GetString(util::BufferLenType::kUInt8));
        uint32_t req_id = buf_oper.GetUint32();
        uint32_t code = buf_oper.GetUint32();

        auto msg_recorder = client_tool_ptr_->GetRecord(req_id);
        if (!msg_recorder) [[unlikely]] {
          // can't find recorder, which means timeout
          AIMRT_TRACE("Can not get req id {} from recorder.", req_id);
          return;
        }

        // find record
        client_invoke_wrapper_ptr = std::move(*msg_recorder);

        if (code) [[unlikely]] {
          client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(code));
          return;
        }

        const auto& info = client_invoke_wrapper_ptr->info;

        // client rsp deserialize
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
          // deserialize failed
          client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_DESERIALIZATION_FAILED));
          return;
        }

        client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_OK));
        return;
      } catch (const std::exception& e) {
        AIMRT_WARN("Handle zenoh rpc msg failed, exception info: {}", e.what());
      }

      if (client_invoke_wrapper_ptr)
        client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
    };

    zenoh_manager_ptr_->RegisterRpcNode(pattern, std::move(handle), "client");
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
  return true;
}

void ZenohRpcBackend::Invoke(
    const std::shared_ptr<runtime::core::rpc::InvokeWrapper>& client_invoke_wrapper_ptr) noexcept {
  try {
    if (state_.load() != State::Start) [[unlikely]] {
      AIMRT_WARN("Method can only be called when state is 'Start'.");
      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    namespace util = aimrt::common::util;

    const auto& info = client_invoke_wrapper_ptr->info;

    std::string pattern = std::string("aimrt_rpc/") +
                          util::UrlEncode(GetRealFuncName(info.func_name)) +
                          limit_domain_;

    uint32_t cur_req_id = req_id_++;

    auto serialization_type =
        client_invoke_wrapper_ptr->ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE);

    if (serialization_type.size() > 255) [[unlikely]] {
      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_UNKNOWN));
      return;
    }

    // client req serialize
    auto buffer_array_view_ptr = aimrt::runtime::core::rpc::TrySerializeReqWithCache(
        *client_invoke_wrapper_ptr, serialization_type);
    if (!buffer_array_view_ptr) [[unlikely]] {
      // serialize failed
      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_SERIALIZATION_FAILED));
      return;
    }

    const auto* buffer_array_data = buffer_array_view_ptr->Data();
    const size_t buffer_array_len = buffer_array_view_ptr->Size();
    size_t req_size = buffer_array_view_ptr->BufferSize();

    // context
    const auto& keys = client_invoke_wrapper_ptr->ctx_ref.GetMetaKeys();
    if (keys.size() > 255) [[unlikely]] {
      AIMRT_WARN("Too much context meta, require less than 255, but actually {}.", keys.size());
      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    std::vector<std::string_view> context_meta_kv;
    size_t context_meta_kv_size = 1;
    for (const auto& key : keys) {
      context_meta_kv_size += (2 + key.size());
      context_meta_kv.emplace_back(key);

      auto val = client_invoke_wrapper_ptr->ctx_ref.GetMetaValue(key);
      context_meta_kv_size += (2 + val.size());
      context_meta_kv.emplace_back(val);
    }
    // padding zenoh pkg
    size_t z_pkg_size = 1 + serialization_type.size() +
                        1 + pattern.size() +
                        4 +
                        context_meta_kv_size +
                        req_size;

    auto timeout = client_invoke_wrapper_ptr->ctx_ref.Timeout();
    auto record_ptr = client_invoke_wrapper_ptr;

    bool ret = client_tool_ptr_->Record(cur_req_id, timeout, std::move(record_ptr));

    if (!ret) [[unlikely]] {
      AIMRT_ERROR("Failed to record msg.");
      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    std::vector<char> msg_buf_vec(z_pkg_size);
    util::BufferOperator buf_oper(msg_buf_vec.data(), msg_buf_vec.size());

    buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);
    buf_oper.SetString(pattern, util::BufferLenType::kUInt8);
    buf_oper.SetUint32(cur_req_id);

    buf_oper.SetUint8(static_cast<uint8_t>(keys.size()));
    for (const auto& s : context_meta_kv) {
      buf_oper.SetString(s, util::BufferLenType::kUInt16);
    }

    // data
    for (size_t ii = 0; ii < buffer_array_len; ++ii) {
      buf_oper.SetBuffer(
          static_cast<const char*>(buffer_array_data[ii].data),
          buffer_array_data[ii].len);
    }

    // send data
    zenoh_manager_ptr_->Publish("req/" + pattern, msg_buf_vec.data(), z_pkg_size);

  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

void ZenohRpcBackend::RegisterGetExecutorFunc(
    const std::function<aimrt::executor::ExecutorRef(std::string_view)>& get_executor_func) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::PreInit,
      "Method can only be called when state is 'PreInit'.");
  get_executor_func_ = get_executor_func;
}

void ZenohRpcBackend::ReturnRspWithStatusCode(
    const std::string& pattern,
    std::string_view serialization_type,
    const char* req_id_buf,
    uint32_t code) {
  namespace util = aimrt::common::util;

  int pkg_size = 1 + serialization_type.size() + 4 + 4;
  std::vector<char> msg_buf_vec(pkg_size);
  util::BufferOperator buf_oper(msg_buf_vec.data(), msg_buf_vec.size());

  buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);
  buf_oper.SetBuffer(req_id_buf, 4);
  buf_oper.SetUint32(code);

  zenoh_manager_ptr_->Publish("rsp/" + pattern, msg_buf_vec.data(), pkg_size);
}

}  // namespace aimrt::plugins::zenoh_plugin