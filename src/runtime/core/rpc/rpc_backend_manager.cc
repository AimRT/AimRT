// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/rpc/rpc_backend_manager.h"

#include <regex>
#include <vector>

#include "aimrt_module_cpp_interface/rpc/rpc_handle.h"
#include "aimrt_module_cpp_interface/rpc/rpc_status.h"
#include "core/rpc/rpc_backend_tools.h"

namespace aimrt::runtime::core::rpc {

void RpcBackendManager::Initialize() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Rpc backend manager can only be initialized once.");
}

void RpcBackendManager::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");

  for (auto& backend : rpc_backend_index_vec_) {
    AIMRT_TRACE("Start rpc backend '{}'.", backend->Name());
    backend->Start();
  }
}

void RpcBackendManager::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  for (auto& backend : rpc_backend_index_vec_) {
    AIMRT_TRACE("Shutdown rpc backend '{}'.", backend->Name());
    backend->Shutdown();
  }
}

void RpcBackendManager::SetRpcRegistry(RpcRegistry* rpc_registry_ptr) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  rpc_registry_ptr_ = rpc_registry_ptr;
}

void RpcBackendManager::SetClientFrameworkAsyncRpcFilterManager(
    FrameworkAsyncRpcFilterManager* ptr) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  client_filter_manager_ptr_ = ptr;
}

void RpcBackendManager::SetServerFrameworkAsyncRpcFilterManager(
    FrameworkAsyncRpcFilterManager* ptr) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  server_filter_manager_ptr_ = ptr;
}

void RpcBackendManager::SetClientsFiltersRules(
    const std::vector<std::pair<std::string, std::vector<std::string>>>& rules) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  clients_filters_rules_ = rules;
}

void RpcBackendManager::SetServersFiltersRules(
    const std::vector<std::pair<std::string, std::vector<std::string>>>& rules) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  servers_filters_rules_ = rules;
}

void RpcBackendManager::SetClientsBackendsRules(
    const std::vector<std::pair<std::string, std::vector<std::string>>>& rules) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  clients_backends_rules_ = rules;
}

void RpcBackendManager::SetServersBackendsRules(
    const std::vector<std::pair<std::string, std::vector<std::string>>>& rules) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  servers_backends_rules_ = rules;
}

void RpcBackendManager::RegisterRpcBackend(RpcBackendBase* rpc_backend_ptr) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  rpc_backend_index_vec_.emplace_back(rpc_backend_ptr);
  rpc_backend_index_map_.emplace(rpc_backend_ptr->Name(), rpc_backend_ptr);
}

bool RpcBackendManager::RegisterServiceFunc(RegisterServiceFuncProxyInfoWrapper&& wrapper) {
  if (state_.load() != State::kInit) [[unlikely]] {
    AIMRT_ERROR("Service func can only be registered when state is 'Init'.");
    return false;
  }

  if (wrapper.req_type_support == nullptr || wrapper.rsp_type_support == nullptr) [[unlikely]] {
    AIMRT_ERROR("Msg type support is null.");
    return false;
  }

  if (wrapper.service_func == nullptr) [[unlikely]] {
    AIMRT_ERROR("Service func is null.");
    return false;
  }

  auto func_name = aimrt::util::ToStdStringView(wrapper.func_name);

  // Create func wrapper
  auto service_func_wrapper_ptr = std::make_unique<ServiceFuncWrapper>();
  service_func_wrapper_ptr->info = FuncInfo{
      .func_name = std::string(func_name),
      .pkg_path = std::string(wrapper.pkg_path),
      .module_name = std::string(wrapper.module_name),
      .custom_type_support_ptr = wrapper.custom_type_support_ptr,
      .req_type_support_ref = aimrt::util::TypeSupportRef(wrapper.req_type_support),
      .rsp_type_support_ref = aimrt::util::TypeSupportRef(wrapper.rsp_type_support)};

  // Create filter
  auto filter_name_vec = GetFilterRules(func_name, servers_filters_rules_);
  server_filter_manager_ptr_->CreateFilterCollectorIfNotExist(func_name, filter_name_vec);

  // Register func wrapper
  const auto& filter_collector = server_filter_manager_ptr_->GetFilterCollector(func_name);

  auto service_func_shared_ptr = std::make_shared<aimrt::rpc::ServiceFunc>(wrapper.service_func);

  service_func_wrapper_ptr->service_func =
      [&filter_collector, service_func_shared_ptr](
          const std::shared_ptr<InvokeWrapper>& invoke_wrapper_ptr) {
        filter_collector.InvokeRpc(
            [service_func_ptr = service_func_shared_ptr.get()](
                const std::shared_ptr<InvokeWrapper>& invoke_wrapper_ptr) {
              aimrt::rpc::ServiceCallback service_callback(
                  [callback{std::move(invoke_wrapper_ptr->callback)}](uint32_t status) {
                    callback(aimrt::rpc::Status(status));
                  });

              (*service_func_ptr)(
                  invoke_wrapper_ptr->ctx_ref.NativeHandle(),
                  invoke_wrapper_ptr->req_ptr,
                  invoke_wrapper_ptr->rsp_ptr,
                  service_callback.NativeHandle());
            },
            invoke_wrapper_ptr);
      };

  const auto& service_func_wrapper_ref = *service_func_wrapper_ptr;

  if (!rpc_registry_ptr_->RegisterServiceFunc(std::move(service_func_wrapper_ptr)))
    return false;

  auto backend_itr = servers_backend_index_map_.find(func_name);
  if (backend_itr == servers_backend_index_map_.end()) {
    auto backend_ptr_vec = GetBackendsByRules(func_name, servers_backends_rules_);
    auto emplace_ret = servers_backend_index_map_.emplace(func_name, std::move(backend_ptr_vec));
    backend_itr = emplace_ret.first;
  }

  bool ret = true;
  for (auto& itr : backend_itr->second) {
    AIMRT_TRACE("Register service func '{}' to backend '{}'.", func_name, itr->Name());
    ret &= itr->RegisterServiceFunc(service_func_wrapper_ref);
  }
  return ret;
}

bool RpcBackendManager::RegisterClientFunc(RegisterClientFuncProxyInfoWrapper&& wrapper) {
  if (state_.load() != State::kInit) [[unlikely]] {
    AIMRT_ERROR("Client func can only be registered when state is 'Init'.");
    return false;
  }

  if (wrapper.req_type_support == nullptr || wrapper.rsp_type_support == nullptr) [[unlikely]] {
    AIMRT_ERROR("Msg type support is null.");
    return false;
  }

  auto func_name = aimrt::util::ToStdStringView(wrapper.func_name);

  // Create func wrapper
  auto client_func_wrapper_ptr = std::make_unique<ClientFuncWrapper>();
  client_func_wrapper_ptr->info = FuncInfo{
      .func_name = std::string(func_name),
      .pkg_path = std::string(wrapper.pkg_path),
      .module_name = std::string(wrapper.module_name),
      .custom_type_support_ptr = wrapper.custom_type_support_ptr,
      .req_type_support_ref = aimrt::util::TypeSupportRef(wrapper.req_type_support),
      .rsp_type_support_ref = aimrt::util::TypeSupportRef(wrapper.rsp_type_support)};

  // Create filter
  auto filter_name_vec = GetFilterRules(func_name, clients_filters_rules_);
  client_filter_manager_ptr_->CreateFilterCollectorIfNotExist(func_name, filter_name_vec);

  // Register func wrapper
  const auto& client_func_wrapper_ref = *client_func_wrapper_ptr;

  if (!rpc_registry_ptr_->RegisterClientFunc(std::move(client_func_wrapper_ptr)))
    return false;

  auto backend_itr = clients_backend_index_map_.find(func_name);
  if (backend_itr == clients_backend_index_map_.end()) {
    auto backend_ptr_vec = GetBackendsByRules(func_name, clients_backends_rules_);
    auto emplace_ret = clients_backend_index_map_.emplace(func_name, std::move(backend_ptr_vec));
    backend_itr = emplace_ret.first;
  }

  bool ret = true;
  for (auto& itr : backend_itr->second) {
    AIMRT_TRACE("Register client func '{}' to backend '{}'.", func_name, itr->Name());
    ret &= itr->RegisterClientFunc(client_func_wrapper_ref);
  }
  return ret;
}

void RpcBackendManager::Invoke(InvokeProxyInfoWrapper&& wrapper) {
  AIMRT_CHECK_ERROR_THROW(state_.load() == State::kStart, "Method can only be called when state is 'Start'.");

  auto func_name = util::ToStdStringView(wrapper.func_name);
  auto client_callback_ptr = std::make_shared<aimrt::rpc::ClientCallback>(wrapper.callback);
  auto& client_callback = *client_callback_ptr;
  aimrt::rpc::ContextRef ctx_ref(wrapper.ctx_ptr);

  // Find registered func
  const auto* client_func_wrapper_ptr = rpc_registry_ptr_->GetClientFuncWrapperPtr(
      func_name, wrapper.pkg_path, wrapper.module_name);

  // If func is not registered
  if (client_func_wrapper_ptr == nullptr) [[unlikely]] {
    AIMRT_WARN("Func is not registered, func: {}, pkg: {}, module: {}",
               func_name, wrapper.pkg_path, wrapper.module_name);

    client_callback(AIMRT_RPC_STATUS_CLI_FUNC_NOT_REGISTERED);
    return;
  }

  // Check ctx
  if (ctx_ref.GetType() != aimrt_rpc_context_type_t::AIMRT_RPC_CLIENT_CONTEXT ||
      ctx_ref.CheckUsed()) {
    client_callback(AIMRT_RPC_STATUS_CLI_INVALID_CONTEXT);
    return;
  }

  ctx_ref.SetUsed();

  if (ctx_ref.GetSerializationType().empty())
    ctx_ref.SetSerializationType(client_func_wrapper_ptr->info.req_type_support_ref.DefaultSerializationType());

  // Find filter
  const auto& filter_collector = client_filter_manager_ptr_->GetFilterCollector(func_name);

  // Create a wrapper
  auto client_invoke_wrapper_ptr = std::make_shared<InvokeWrapper>(
      InvokeWrapper{
          .info = client_func_wrapper_ptr->info,
          .req_ptr = wrapper.req_ptr,
          .rsp_ptr = wrapper.rsp_ptr,
          .ctx_ref = ctx_ref});

  client_invoke_wrapper_ptr->callback =
      [client_invoke_wrapper_ptr, client_callback_ptr{std::move(client_callback_ptr)}](aimrt::rpc::Status status) mutable {
        (*client_callback_ptr)(status.Code());
      };

  // Initiate a call
  filter_collector.InvokeRpc(
      [this](const std::shared_ptr<InvokeWrapper>& client_invoke_wrapper_ptr) {
        // When timeout is not set, the default 5s timeout
        if (client_invoke_wrapper_ptr->ctx_ref.Timeout().count() == 0) {
          client_invoke_wrapper_ptr->ctx_ref.SetTimeout(std::chrono::seconds(5));
        }

        std::string_view func_name = client_invoke_wrapper_ptr->info.func_name;

        auto find_itr = clients_backend_index_map_.find(func_name);

        if (find_itr == clients_backend_index_map_.end()) [[unlikely]] {
          AIMRT_WARN("Rpc call found no backend to handle, func name '{}'.", func_name);
          InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_NO_BACKEND_TO_HANDLE));
          return;
        }

        const auto& backend_ptr_vec = find_itr->second;

        if (backend_ptr_vec.empty()) [[unlikely]] {
          AIMRT_WARN("Rpc call found no backend to handle, func name '{}'.", func_name);
          InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_NO_BACKEND_TO_HANDLE));
          return;
        }

        // If a backend is specified in ctx, the specified backend is used
        std::string_view to_addr(client_invoke_wrapper_ptr->ctx_ref.GetToAddr());
        if (!to_addr.empty()) {
          // to_addr format: backend_name://url_str
          AIMRT_TRACE("Rpc call use the specified address '{}', func name '{}'.", to_addr, func_name);
          auto pos = to_addr.find("://");
          if (pos != std::string_view::npos) {
            auto addr_backend = to_addr.substr(0, pos);

            auto backend_itr = rpc_backend_index_map_.find(addr_backend);
            if (backend_itr != rpc_backend_index_map_.end()) {
              auto* backend_ptr = backend_itr->second;

              if (std::find(backend_ptr_vec.begin(), backend_ptr_vec.end(), backend_ptr) != backend_ptr_vec.end()) {
                backend_ptr->Invoke(client_invoke_wrapper_ptr);
                return;
              }
            }
          }
          AIMRT_ERROR("Rpc call address '{}' is invalid, func name '{}'.", to_addr, func_name);
          InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_INVALID_ADDR));
          return;
        }

        // Use the first backend of the configuration
        auto& backend = *(backend_ptr_vec[0]);
        AIMRT_TRACE("Rpc call use backend '{}', func name '{}'.", backend.Name(), func_name);
        backend.Invoke(client_invoke_wrapper_ptr);
      },
      client_invoke_wrapper_ptr);
}

RpcBackendManager::FuncBackendInfoMap RpcBackendManager::GetClientsBackendInfo() const {
  std::unordered_map<std::string_view, std::vector<std::string_view>> result;
  for (const auto& itr : clients_backend_index_map_) {
    std::vector<std::string_view> backends_name;
    backends_name.reserve(itr.second.size());
    for (const auto& item : itr.second)
      backends_name.emplace_back(item->Name());

    result.emplace(itr.first, std::move(backends_name));
  }

  return result;
}

RpcBackendManager::FuncBackendInfoMap RpcBackendManager::GetServersBackendInfo() const {
  std::unordered_map<std::string_view, std::vector<std::string_view>> result;
  for (const auto& itr : servers_backend_index_map_) {
    std::vector<std::string_view> backends_name;
    for (const auto& item : itr.second)
      backends_name.emplace_back(item->Name());

    result.emplace(itr.first, std::move(backends_name));
  }

  return result;
}

std::vector<RpcBackendBase*> RpcBackendManager::GetBackendsByRules(
    std::string_view func_name,
    const std::vector<std::pair<std::string, std::vector<std::string>>>& rules) {
  for (const auto& item : rules) {
    const auto& func_regex = item.first;
    const auto& enable_backends = item.second;

    try {
      if (std::regex_match(func_name.begin(), func_name.end(), std::regex(func_regex, std::regex::ECMAScript))) {
        std::vector<RpcBackendBase*> backend_ptr_vec;

        for (const auto& backend_name : enable_backends) {
          auto itr = std::find_if(
              rpc_backend_index_vec_.begin(), rpc_backend_index_vec_.end(),
              [&backend_name](const RpcBackendBase* backend_ptr) -> bool {
                return backend_ptr->Name() == backend_name;
              });

          if (itr == rpc_backend_index_vec_.end()) [[unlikely]] {
            AIMRT_WARN("Can not find '{}' in backend list.", backend_name);
            continue;
          }

          backend_ptr_vec.emplace_back(*itr);
        }

        return backend_ptr_vec;
      }
    } catch (const std::exception& e) {
      AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                 func_regex, func_name, e.what());
    }
  }

  return {};
}

std::vector<std::string> RpcBackendManager::GetFilterRules(
    std::string_view func_name,
    const std::vector<std::pair<std::string, std::vector<std::string>>>& rules) {
  for (const auto& item : rules) {
    const auto& func_regex = item.first;
    const auto& filters = item.second;

    try {
      if (std::regex_match(func_name.begin(), func_name.end(), std::regex(func_regex, std::regex::ECMAScript))) {
        return filters;
      }
    } catch (const std::exception& e) {
      AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                 func_regex, func_name, e.what());
    }
  }

  return {};
}

}  // namespace aimrt::runtime::core::rpc