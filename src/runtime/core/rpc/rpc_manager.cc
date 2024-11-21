// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/rpc/rpc_manager.h"
#include "core/rpc/local_rpc_backend.h"
#include "core/rpc/rpc_backend_tools.h"

namespace YAML {
template <>
struct convert<aimrt::runtime::core::rpc::RpcManager::Options> {
  using Options = aimrt::runtime::core::rpc::RpcManager::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["backends"] = YAML::Node();
    for (const auto& backend : rhs.backends_options) {
      Node backend_options_node;
      backend_options_node["type"] = backend.type;
      backend_options_node["options"] = backend.options;
      node["backends"].push_back(backend_options_node);
    }

    node["clients_options"] = YAML::Node();
    for (const auto& client_options : rhs.clients_options) {
      Node client_options_node;
      client_options_node["func_name"] = client_options.func_name;
      client_options_node["enable_backends"] = client_options.enable_backends;
      client_options_node["enable_filters"] = client_options.enable_filters;
      node["clients_options"].push_back(client_options_node);
    }

    node["servers_options"] = YAML::Node();
    for (const auto& server_options : rhs.servers_options) {
      Node server_options_node;
      server_options_node["func_name"] = server_options.func_name;
      server_options_node["enable_backends"] = server_options.enable_backends;
      server_options_node["enable_filters"] = server_options.enable_filters;
      node["servers_options"].push_back(server_options_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["backends"] && node["backends"].IsSequence()) {
      for (const auto& backend_options_node : node["backends"]) {
        auto backend_options = Options::BackendOptions{
            .type = backend_options_node["type"].as<std::string>()};

        if (backend_options_node["options"])
          backend_options.options = backend_options_node["options"];
        else
          backend_options.options = YAML::Node(YAML::NodeType::Null);

        rhs.backends_options.emplace_back(std::move(backend_options));
      }
    }

    if (node["clients_options"] && node["clients_options"].IsSequence()) {
      for (const auto& client_options_node : node["clients_options"]) {
        auto client_options = Options::ClientOptions{
            .func_name = client_options_node["func_name"].as<std::string>(),
            .enable_backends = client_options_node["enable_backends"].as<std::vector<std::string>>()};

        if (client_options_node["enable_filters"])
          client_options.enable_filters = client_options_node["enable_filters"].as<std::vector<std::string>>();

        rhs.clients_options.emplace_back(std::move(client_options));
      }
    }

    if (node["servers_options"] && node["servers_options"].IsSequence()) {
      for (const auto& server_options_node : node["servers_options"]) {
        auto server_options = Options::ServerOptions{
            .func_name = server_options_node["func_name"].as<std::string>(),
            .enable_backends = server_options_node["enable_backends"].as<std::vector<std::string>>()};

        if (server_options_node["enable_filters"])
          server_options.enable_filters = server_options_node["enable_filters"].as<std::vector<std::string>>();

        rhs.servers_options.emplace_back(std::move(server_options));
      }
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt::runtime::core::rpc {

void RpcManager::Initialize(YAML::Node options_node) {
  RegisterLocalRpcBackend();
  RegisterDebugLogFilter();

  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Rpc manager can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  rpc_registry_ptr_ = std::make_unique<RpcRegistry>();
  rpc_registry_ptr_->SetLogger(logger_ptr_);

  rpc_backend_manager_.SetLogger(logger_ptr_);
  rpc_backend_manager_.SetRpcRegistry(rpc_registry_ptr_.get());
  rpc_backend_manager_.SetClientFrameworkAsyncRpcFilterManager(&client_filter_manager_);
  rpc_backend_manager_.SetServerFrameworkAsyncRpcFilterManager(&server_filter_manager_);

  std::vector<std::string> rpc_backend_name_vec;

  // 根据配置初始化指定的backend
  for (auto& backend_options : options_.backends_options) {
    auto finditr = std::find_if(
        rpc_backend_vec_.begin(), rpc_backend_vec_.end(),
        [&backend_options](const auto& ptr) {
          return ptr->Name() == backend_options.type;
        });

    AIMRT_CHECK_ERROR_THROW(finditr != rpc_backend_vec_.end(),
                            "Invalid rpc backend type '{}'",
                            backend_options.type);

    (*finditr)->SetRpcRegistry(rpc_registry_ptr_.get());
    (*finditr)->Initialize(backend_options.options);

    rpc_backend_manager_.RegisterRpcBackend(finditr->get());

    used_rpc_backend_vec_.emplace_back(finditr->get());
    rpc_backend_name_vec.emplace_back((*finditr)->Name());
  }

  // 设置rules
  std::vector<std::pair<std::string, std::vector<std::string>>> client_backends_rules;
  std::vector<std::pair<std::string, std::vector<std::string>>> client_filters_rules;
  for (const auto& item : options_.clients_options) {
    for (const auto& backend_name : item.enable_backends) {
      AIMRT_CHECK_ERROR_THROW(
          std::find(rpc_backend_name_vec.begin(), rpc_backend_name_vec.end(), backend_name) != rpc_backend_name_vec.end(),
          "Invalid rpc backend type '{}' for func '{}'",
          backend_name, item.func_name);
    }

    client_backends_rules.emplace_back(item.func_name, item.enable_backends);
    client_filters_rules.emplace_back(item.func_name, item.enable_filters);
  }
  rpc_backend_manager_.SetClientsBackendsRules(client_backends_rules);
  rpc_backend_manager_.SetClientsFiltersRules(client_filters_rules);

  std::vector<std::pair<std::string, std::vector<std::string>>> server_backends_rules;
  std::vector<std::pair<std::string, std::vector<std::string>>> server_filters_rules;
  for (const auto& item : options_.servers_options) {
    for (const auto& backend_name : item.enable_backends) {
      AIMRT_CHECK_ERROR_THROW(
          std::find(rpc_backend_name_vec.begin(), rpc_backend_name_vec.end(), backend_name) != rpc_backend_name_vec.end(),
          "Invalid rpc backend type '{}' for func '{}'",
          backend_name, item.func_name);
    }

    server_backends_rules.emplace_back(item.func_name, item.enable_backends);
    server_filters_rules.emplace_back(item.func_name, item.enable_filters);
  }
  rpc_backend_manager_.SetServersBackendsRules(server_backends_rules);
  rpc_backend_manager_.SetServersFiltersRules(server_filters_rules);

  // 初始化backend manager
  rpc_backend_manager_.Initialize();

  options_node = options_;

  AIMRT_INFO("Rpc manager init complete");
}

void RpcManager::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");

  rpc_backend_manager_.Start();

  AIMRT_INFO("Rpc manager start completed.");
}

void RpcManager::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  AIMRT_INFO("Rpc manager Shutdown.");

  rpc_handle_proxy_map_.clear();

  rpc_backend_manager_.Shutdown();

  rpc_backend_vec_.clear();

  rpc_registry_ptr_.reset();

  server_filter_manager_.Clear();
  client_filter_manager_.Clear();

  get_executor_func_ = std::function<executor::ExecutorRef(std::string_view)>();
}

std::list<std::pair<std::string, std::string>> RpcManager::GenInitializationReport() const {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");

  std::vector<std::vector<std::string>> client_info_table =
      {{"func", "module", "backends", "filters"}};

  const auto& client_backend_info = rpc_backend_manager_.GetClientsBackendInfo();
  const auto& client_index_map = rpc_registry_ptr_->GetClientIndexMap();

  for (const auto& client_index_itr : client_index_map) {
    auto func_name = client_index_itr.first;
    auto client_backend_itr = client_backend_info.find(func_name);
    AIMRT_CHECK_ERROR_THROW(client_backend_itr != client_backend_info.end(),
                            "Invalid rpc registry info.");

    auto filter_name_vec = client_filter_manager_.GetFilterNameVec(func_name);

    for (const auto& item : client_index_itr.second) {
      std::vector<std::string> cur_client_info(4);
      cur_client_info[0] = func_name;
      cur_client_info[1] = item->info.module_name;
      cur_client_info[2] = aimrt::common::util::JoinVec(client_backend_itr->second, ",");
      cur_client_info[3] = aimrt::common::util::JoinVec(filter_name_vec, ",");
      client_info_table.emplace_back(std::move(cur_client_info));
    }
  }

  std::vector<std::vector<std::string>> server_info_table =
      {{"func", "module", "backends", "filters"}};

  const auto& server_backend_info = rpc_backend_manager_.GetServersBackendInfo();
  const auto& server_index_map = rpc_registry_ptr_->GetServiceIndexMap();

  for (const auto& server_index_itr : server_index_map) {
    auto func_name = server_index_itr.first;
    auto server_backend_itr = server_backend_info.find(func_name);
    AIMRT_CHECK_ERROR_THROW(server_backend_itr != server_backend_info.end(),
                            "Invalid rpc registry info.");

    auto filter_name_vec = server_filter_manager_.GetFilterNameVec(func_name);

    for (const auto& item : server_index_itr.second) {
      std::vector<std::string> cur_server_info(4);
      cur_server_info[0] = func_name;
      cur_server_info[1] = item->info.module_name;
      cur_server_info[2] = aimrt::common::util::JoinVec(server_backend_itr->second, ",");
      cur_server_info[3] = aimrt::common::util::JoinVec(filter_name_vec, ",");
      server_info_table.emplace_back(std::move(cur_server_info));
    }
  }

  std::vector<std::string> rpc_backend_name_vec;
  rpc_backend_name_vec.reserve(rpc_backend_vec_.size());
  for (const auto& item : rpc_backend_vec_)
    rpc_backend_name_vec.emplace_back(item->Name());

  std::string rpc_backend_name_vec_str;
  if (rpc_backend_name_vec.empty()) {
    rpc_backend_name_vec_str = "<empty>";
  } else {
    rpc_backend_name_vec_str = "[ " + aimrt::common::util::JoinVec(rpc_backend_name_vec, " , ") + " ]";
  }

  auto rpc_client_filter_name_vec = client_filter_manager_.GetAllFiltersName();
  std::string rpc_client_filter_name_vec_str;
  if (rpc_client_filter_name_vec.empty()) {
    rpc_client_filter_name_vec_str = "<empty>";
  } else {
    rpc_client_filter_name_vec_str = "[ " + aimrt::common::util::JoinVec(rpc_client_filter_name_vec, " , ") + " ]";
  }

  auto rpc_server_filter_name_vec = server_filter_manager_.GetAllFiltersName();
  std::string rpc_server_filter_name_vec_str;
  if (rpc_server_filter_name_vec.empty()) {
    rpc_server_filter_name_vec_str = "<empty>";
  } else {
    rpc_server_filter_name_vec_str = "[ " + aimrt::common::util::JoinVec(rpc_server_filter_name_vec, " , ") + " ]";
  }

  std::list<std::pair<std::string, std::string>> report{
      {"Rpc Backend List", rpc_backend_name_vec_str},
      {"Rpc Client Filter List", rpc_client_filter_name_vec_str},
      {"Rpc Server Filter List", rpc_server_filter_name_vec_str},
      {"Rpc Client Info", aimrt::common::util::DrawTable(client_info_table)},
      {"Rpc Server Info", aimrt::common::util::DrawTable(server_info_table)}};

  for (const auto& backend_ptr : used_rpc_backend_vec_) {
    report.splice(report.end(), backend_ptr->GenInitializationReport());
  }

  return report;
}

void RpcManager::RegisterRpcBackend(
    std::unique_ptr<RpcBackendBase>&& rpc_backend_ptr) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  rpc_backend_vec_.emplace_back(std::move(rpc_backend_ptr));
}

void RpcManager::RegisterGetExecutorFunc(
    const std::function<executor::ExecutorRef(std::string_view)>& get_executor_func) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  get_executor_func_ = get_executor_func;
}

const RpcHandleProxy& RpcManager::GetRpcHandleProxy(const util::ModuleDetailInfo& module_info) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");

  auto itr = rpc_handle_proxy_map_.find(module_info.name);
  if (itr != rpc_handle_proxy_map_.end()) return *(itr->second);

  auto emplace_ret = rpc_handle_proxy_map_.emplace(
      module_info.name, std::make_unique<RpcHandleProxy>(
                            module_info.pkg_path,
                            module_info.name,
                            rpc_backend_manager_,
                            passed_context_meta_keys_));
  return *(emplace_ret.first->second);
}

void RpcManager::RegisterClientFilter(std::string_view name, FrameworkAsyncRpcFilter&& filter) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  client_filter_manager_.RegisterFilter(name, std::move(filter));
}

void RpcManager::RegisterServerFilter(std::string_view name, FrameworkAsyncRpcFilter&& filter) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  server_filter_manager_.RegisterFilter(name, std::move(filter));
}

void RpcManager::AddPassedContextMetaKeys(const std::unordered_set<std::string>& keys) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  passed_context_meta_keys_.insert(keys.begin(), keys.end());
}

const RpcRegistry* RpcManager::GetRpcRegistry() const {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");
  return rpc_registry_ptr_.get();
}

const std::vector<RpcBackendBase*>& RpcManager::GetUsedRpcBackend() const {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");
  return used_rpc_backend_vec_;
}

void RpcManager::RegisterLocalRpcBackend() {
  std::unique_ptr<RpcBackendBase> local_rpc_backend_ptr =
      std::make_unique<LocalRpcBackend>();

  static_cast<LocalRpcBackend*>(local_rpc_backend_ptr.get())
      ->SetLogger(logger_ptr_);

  static_cast<LocalRpcBackend*>(local_rpc_backend_ptr.get())
      ->RegisterGetExecutorFunc(get_executor_func_);

  RegisterRpcBackend(std::move(local_rpc_backend_ptr));
}

void RpcManager::RegisterDebugLogFilter() {
  RegisterClientFilter(
      "debug_log",
      [this](const std::shared_ptr<InvokeWrapper>& ptr, FrameworkAsyncRpcHandle&& h) {
        auto buf_ptr = TrySerializeReqWithCache(*ptr, "json");

        if (buf_ptr) {
          auto req_str = buf_ptr->JoinToString();

          AIMRT_INFO("RPC client start new rpc call. func name: {}, context: {}, req: {}",
                     ptr->info.func_name, ptr->ctx_ref.ToString(), req_str);
        } else {
          AIMRT_INFO("RPC client start new rpc call. func name: {}, context: {}",
                     ptr->info.func_name, ptr->ctx_ref.ToString());
        }

        ptr->callback =
            [this, ptr, callback{std::move(ptr->callback)}](aimrt::rpc::Status status) {
              if (!status.OK()) {
                AIMRT_WARN("RPC client get rpc error ret. func name: {}, status: {}",
                           ptr->info.func_name, status.ToString());
              } else {
                auto buf_ptr = TrySerializeRspWithCache(*ptr, "json");

                if (buf_ptr) {
                  auto rsp_str = buf_ptr->JoinToString();

                  AIMRT_INFO("RPC client get rpc ret. func name: {}, status: {}, rsp: {}",
                             ptr->info.func_name, status.ToString(), rsp_str);
                } else {
                  AIMRT_INFO("RPC client get rpc ret. func name: {}, status: {}",
                             ptr->info.func_name, status.ToString());
                }
              }

              callback(status);
            };

        h(ptr);
      });

  RegisterServerFilter(
      "debug_log",
      [this](const std::shared_ptr<InvokeWrapper>& ptr, FrameworkAsyncRpcHandle&& h) {
        auto buf_ptr = TrySerializeReqWithCache(*ptr, "json");

        if (buf_ptr) {
          auto req_str = buf_ptr->JoinToString();

          AIMRT_INFO("RPC server start new rpc call. func name: {}, context: {}, req: {}",
                     ptr->info.func_name, ptr->ctx_ref.ToString(), req_str);
        } else {
          AIMRT_INFO("RPC server start new rpc call. func name: {}, context: {}",
                     ptr->info.func_name, ptr->ctx_ref.ToString());
        }

        ptr->callback =
            [this, ptr, callback{std::move(ptr->callback)}](aimrt::rpc::Status status) {
              if (!status.OK()) {
                AIMRT_WARN("RPC server get rpc error ret. func name: {}, status: {}",
                           ptr->info.func_name, status.ToString());
              } else {
                auto buf_ptr = TrySerializeRspWithCache(*ptr, "json");

                if (buf_ptr) {
                  auto rsp_str = buf_ptr->JoinToString();

                  AIMRT_INFO("RPC server get rpc ret. func name: {}, status: {}, rsp: {}",
                             ptr->info.func_name, status.ToString(), rsp_str);
                } else {
                  AIMRT_INFO("RPC server get rpc ret. func name: {}, status: {}",
                             ptr->info.func_name, status.ToString());
                }
              }

              callback(status);
            };

        h(ptr);
      });
}

}  // namespace aimrt::runtime::core::rpc
