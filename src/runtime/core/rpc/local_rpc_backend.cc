// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/rpc/local_rpc_backend.h"
#include "core/rpc/rpc_backend_tools.h"
#include "util/string_util.h"
#include "util/url_parser.h"

namespace YAML {
template <>
struct convert<aimrt::runtime::core::rpc::LocalRpcBackend::Options> {
  using Options = aimrt::runtime::core::rpc::LocalRpcBackend::Options;

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

namespace aimrt::runtime::core::rpc {

void LocalRpcBackend::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Local rpc backend can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  client_tool_ptr_ =
      std::make_unique<util::RpcClientTool<std::shared_ptr<InvokeWrapper>>>();

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
        [](std::shared_ptr<InvokeWrapper>&& client_invoke_wrapper_ptr) {
          client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_TIMEOUT));
        });

    AIMRT_TRACE("Local rpc backend enable the timeout function, use '{}' as timeout executor.",
                options_.timeout_executor);
  } else {
    AIMRT_TRACE("Local rpc backend does not enable the timeout function.");
  }

  options_node = options_;
}

void LocalRpcBackend::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");
}

void LocalRpcBackend::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  service_func_register_index_.clear();

  client_tool_ptr_.reset();

  get_executor_func_ = nullptr;
}

bool LocalRpcBackend::RegisterServiceFunc(
    const ServiceFuncWrapper& service_func_wrapper) noexcept {
  try {
    if (state_.load() != State::kInit) {
      AIMRT_ERROR("Service func can only be registered when state is 'Init'.");
      return false;
    }

    const auto& info = service_func_wrapper.info;

    service_func_register_index_[info.func_name][info.pkg_path].emplace(info.module_name);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

bool LocalRpcBackend::RegisterClientFunc(
    const ClientFuncWrapper& client_func_wrapper) noexcept {
  try {
    if (state_.load() != State::kInit) {
      AIMRT_ERROR("Client func can only be registered when state is 'Init'.");
      return false;
    }

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

void LocalRpcBackend::Invoke(
    const std::shared_ptr<InvokeWrapper>& client_invoke_wrapper_ptr) noexcept {
  try {
    if (state_.load() != State::kStart) [[unlikely]] {
      AIMRT_WARN("Method can only be called when state is 'Start'.");
      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    // Check ctx
    std::string_view service_pkg_path, service_module_name;

    auto to_addr = client_invoke_wrapper_ptr->ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_TO_ADDR);

    // url: local://rpc/func_name?pkg_path=xxxx&module_name=yyyy
    if (!to_addr.empty()) {
      namespace util = aimrt::common::util;
      auto url = util::ParseUrl<std::string_view>(to_addr);
      if (url) {
        if (url->protocol != Name()) [[unlikely]] {
          AIMRT_WARN("Invalid addr: {}", to_addr);
          client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
          return;
        }
        service_pkg_path = util::GetValueFromStrKV(url->query, "pkg_path");
        service_module_name = util::GetValueFromStrKV(url->query, "module_name");
      }
    }

    const auto& client_info = client_invoke_wrapper_ptr->info;

    // Find qualified conditions in the local service registry
    auto service_func_register_index_find_func_itr = service_func_register_index_.find(client_info.func_name);
    if (service_func_register_index_find_func_itr == service_func_register_index_.end()) [[unlikely]] {
      AIMRT_ERROR("Service func '{}' is not registered in local rpc backend.", client_info.func_name);
      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_SVR_NOT_FOUND));
      return;
    }

    if (service_pkg_path.empty()) {
      if (service_module_name.empty()) {
        // Neither pkg nor module were specified, just find the first one
        auto service_func_register_index_find_pkg_itr =
            service_func_register_index_find_func_itr->second.begin();

        service_pkg_path = service_func_register_index_find_pkg_itr->first;
        service_module_name = *(service_func_register_index_find_pkg_itr->second.begin());
      } else {
        // pkg is not specified, but module is specified. Iterate through all pkgs and find the first module that meets the criteria
        for (const auto& itr : service_func_register_index_find_func_itr->second) {
          if (itr.second.find(service_module_name) != itr.second.end()) {
            service_pkg_path = itr.first;
            break;
          }
        }
        if (service_pkg_path.empty()) {
          AIMRT_WARN("Can not find service func '{}' in module '{}'. Addr: {}",
                     client_info.func_name, service_module_name, to_addr);

          client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_INVALID_ADDR));
          return;
        }
      }

    } else {
      auto service_func_register_index_find_pkg_itr =
          service_func_register_index_find_func_itr->second.find(service_pkg_path);

      if (service_func_register_index_find_pkg_itr ==
          service_func_register_index_find_func_itr->second.end()) {
        AIMRT_WARN("Can not find service func '{}' in pkg '{}'. Addr: {}",
                   client_info.func_name, service_pkg_path, to_addr);

        client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_INVALID_ADDR));
        return;
      }

      if (service_module_name.empty()) {
        service_module_name = *(service_func_register_index_find_pkg_itr->second.begin());
      } else {
        auto service_func_register_index_find_module_itr =
            service_func_register_index_find_pkg_itr->second.find(service_module_name);

        if (service_func_register_index_find_module_itr ==
            service_func_register_index_find_pkg_itr->second.end()) {
          AIMRT_WARN("Can not find service func '{}' in pkg '{}' module '{}'. Addr: {}",
                     client_info.func_name, service_pkg_path, service_module_name, to_addr);

          client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_INVALID_ADDR));
          return;
        }
      }
    }

    AIMRT_TRACE("Invoke rpc func '{}' in pkg '{}' module '{}'.",
                client_info.func_name, service_pkg_path, service_module_name);

    // Find a registered service method
    const auto* service_func_wrapper_ptr =
        rpc_registry_ptr_->GetServiceFuncWrapperPtr(client_info.func_name, service_pkg_path, service_module_name);

    if (service_func_wrapper_ptr == nullptr) [[unlikely]] {
      AIMRT_WARN("Can not find service func '{}' in pkg '{}' module '{}'",
                 client_info.func_name, service_pkg_path, service_module_name);

      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    // Create a service invoke wrapper
    auto service_invoke_wrapper_ptr = std::make_shared<InvokeWrapper>(InvokeWrapper{
        .info = service_func_wrapper_ptr->info});
    const auto& service_info = service_invoke_wrapper_ptr->info;

    // Create service ctx
    auto ctx_ptr = std::make_shared<aimrt::rpc::Context>(aimrt_rpc_context_type_t::AIMRT_RPC_SERVER_CONTEXT);
    service_invoke_wrapper_ptr->ctx_ref = ctx_ptr;

    ctx_ptr->SetTimeout(client_invoke_wrapper_ptr->ctx_ref.Timeout());

    const auto& meta_keys = client_invoke_wrapper_ptr->ctx_ref.GetMetaKeys();
    for (const auto& item : meta_keys)
      ctx_ptr->SetMetaValue(item, client_invoke_wrapper_ptr->ctx_ref.GetMetaValue(item));

    ctx_ptr->SetFunctionName(service_info.func_name);
    ctx_ptr->SetMetaValue(AIMRT_RPC_CONTEXT_KEY_BACKEND, Name());
    ctx_ptr->SetMetaValue("aimrt-from_pkg", client_info.pkg_path);
    ctx_ptr->SetMetaValue("aimrt-from_module", client_info.module_name);

    // In the same pkg, call it directly without serialization
    if (service_pkg_path == client_info.pkg_path) {
      service_invoke_wrapper_ptr->req_ptr = client_invoke_wrapper_ptr->req_ptr;
      service_invoke_wrapper_ptr->rsp_ptr = client_invoke_wrapper_ptr->rsp_ptr;
      service_invoke_wrapper_ptr->callback =
          [ctx_ptr,
           service_invoke_wrapper_ptr,
           client_invoke_wrapper_ptr](aimrt::rpc::Status status) {
            client_invoke_wrapper_ptr->callback(status);
          };

      service_func_wrapper_ptr->service_func(service_invoke_wrapper_ptr);
      return;
    }

    // Not within a pkg, it needs to be serialized and enable timeout function

    // Record request
    auto timeout = client_invoke_wrapper_ptr->ctx_ref.Timeout();
    uint32_t cur_req_id = req_id_++;
    auto record_ptr = client_invoke_wrapper_ptr;

    bool ret = client_tool_ptr_->Record(cur_req_id, timeout, std::move(record_ptr));

    if (!ret) [[unlikely]] {
      // It is unlikely to appear generally
      AIMRT_ERROR("Failed to record msg.");
      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    // client req serialization
    std::string serialization_type(client_invoke_wrapper_ptr->ctx_ref.GetSerializationType());

    auto buffer_array_view_ptr = TrySerializeReqWithCache(*client_invoke_wrapper_ptr, serialization_type);
    if (!buffer_array_view_ptr) [[unlikely]] {
      // Serialization failed
      AIMRT_ERROR(
          "Req serialization failed in local rpc backend, serialization_type {}, pkg_path: {}, module_name: {}, func_name: {}",
          serialization_type, client_info.pkg_path, client_info.module_name, client_info.func_name);

      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_SERIALIZATION_FAILED));
      return;
    }

    // service req deserialization
    std::shared_ptr<void> service_req_ptr = service_info.req_type_support_ref.CreateSharedPtr();
    service_invoke_wrapper_ptr->req_ptr = service_req_ptr.get();

    bool deserialize_ret = service_info.req_type_support_ref.Deserialize(
        serialization_type,
        *(buffer_array_view_ptr->NativeHandle()),
        service_req_ptr.get());

    if (!deserialize_ret) [[unlikely]] {
      // Deserialization failed
      AIMRT_FATAL(
          "Rsp deserialization failed in local rpc backend, serialization_type {}, pkg_path: {}, module_name: {}, func_name: {}",
          serialization_type, service_info.pkg_path, service_info.module_name, service_info.func_name);

      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_DESERIALIZATION_FAILED));
      return;
    }

    // cache service req deserialization buf
    service_invoke_wrapper_ptr->req_serialization_cache.emplace(serialization_type, buffer_array_view_ptr);

    // Create service rsp
    std::shared_ptr<void> service_rsp_ptr = service_info.rsp_type_support_ref.CreateSharedPtr();
    service_invoke_wrapper_ptr->rsp_ptr = service_rsp_ptr.get();

    // Set callback
    service_invoke_wrapper_ptr->callback =
        [this,
         service_invoke_wrapper_ptr,
         ctx_ptr,
         cur_req_id,
         service_req_ptr,
         service_rsp_ptr,
         serialization_type{std::move(serialization_type)}](aimrt::rpc::Status status) {
          auto msg_recorder = client_tool_ptr_->GetRecord(cur_req_id);
          if (!msg_recorder) [[unlikely]] {
            // No record was found, which means that the call has timed out. The record has been deleted after the timeout process is gone.
            AIMRT_TRACE("Can not get req id {} from recorder.", cur_req_id);
            return;
          }

          // Get the record
          auto client_invoke_wrapper_ptr = std::move(*msg_recorder);
          const auto& client_info = client_invoke_wrapper_ptr->info;

          const auto& service_info = service_invoke_wrapper_ptr->info;

          // service rsp serialization
          auto buffer_array_view_ptr = TrySerializeRspWithCache(*service_invoke_wrapper_ptr, serialization_type);
          if (!buffer_array_view_ptr) [[unlikely]] {
            // Serialization failed
            AIMRT_ERROR(
                "Rsp serialization failed in local rpc backend, serialization_type {}, pkg_path: {}, module_name: {}, func_name: {}",
                serialization_type, service_info.pkg_path, service_info.module_name, service_info.func_name);

            client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_SVR_SERIALIZATION_FAILED));

            return;
          }

          // client rsp deserialization
          bool deserialize_ret = client_info.rsp_type_support_ref.Deserialize(
              serialization_type,
              *(buffer_array_view_ptr->NativeHandle()),
              client_invoke_wrapper_ptr->rsp_ptr);

          if (!deserialize_ret) {
            // Deserialization failed
            AIMRT_ERROR(
                "Req deserialization failed in local rpc backend, serialization_type {}, pkg_path: {}, module_name: {}, func_name: {}",
                serialization_type, client_info.pkg_path, client_info.module_name, client_info.func_name);

            client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_SVR_DESERIALIZATION_FAILED));

            return;
          }

          // cache client rsp deserialization buf
          client_invoke_wrapper_ptr->rsp_serialization_cache.emplace(serialization_type, buffer_array_view_ptr);

          // Calling callback
          client_invoke_wrapper_ptr->callback(status);
        };

    // Call service rpc
    service_func_wrapper_ptr->service_func(service_invoke_wrapper_ptr);
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

void LocalRpcBackend::RegisterGetExecutorFunc(
    const std::function<aimrt::executor::ExecutorRef(std::string_view)>& get_executor_func) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  get_executor_func_ = get_executor_func;
}

}  // namespace aimrt::runtime::core::rpc
