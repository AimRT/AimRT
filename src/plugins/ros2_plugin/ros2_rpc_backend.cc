// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "ros2_plugin/ros2_rpc_backend.h"

#include <regex>

#include "aimrt_module_cpp_interface/rpc/rpc_status.h"
#include "util/url_parser.h"

namespace YAML {

template <>
struct convert<aimrt::plugins::ros2_plugin::Ros2RpcBackend::Options::QosOptions> {
  using Options = aimrt::plugins::ros2_plugin::Ros2RpcBackend::Options::QosOptions;

  static Node encode(const Options& rhs) {
    Node node;

    node["history"] = rhs.history;
    node["depth"] = rhs.depth;
    node["reliability"] = rhs.reliability;
    node["durability"] = rhs.durability;
    node["lifespan"] = rhs.lifespan;
    node["deadline"] = rhs.deadline;
    node["liveliness"] = rhs.liveliness;
    node["liveliness_lease_duration"] = rhs.liveliness_lease_duration;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (node["history"])
      rhs.history = node["history"].as<std::string>();

    if (node["depth"])
      rhs.depth = node["depth"].as<int>();

    if (node["reliability"])
      rhs.reliability = node["reliability"].as<std::string>();

    if (node["durability"])
      rhs.durability = node["durability"].as<std::string>();

    if (node["lifespan"])
      rhs.lifespan = node["lifespan"].as<int>();

    if (node["deadline"])
      rhs.deadline = node["deadline"].as<int>();

    if (node["liveliness"])
      rhs.liveliness = node["liveliness"].as<std::string>();

    if (node["liveliness_lease_duration"])
      rhs.liveliness_lease_duration = node["liveliness_lease_duration"].as<int>();

    return true;
  }
};

template <>
struct convert<aimrt::plugins::ros2_plugin::Ros2RpcBackend::Options> {
  using Options = aimrt::plugins::ros2_plugin::Ros2RpcBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["timeout_executor"] = rhs.timeout_executor;

    node["clients_options"] = YAML::Node();
    for (const auto& client_options : rhs.clients_options) {
      Node client_options_node;
      client_options_node["func_name"] = client_options.func_name;
      client_options_node["qos"] = client_options.qos;
      client_options_node["remapping_rule"] = client_options.remapping_rule;

      node["clients_options"].push_back(client_options_node);
    }

    node["servers_options"] = YAML::Node();
    for (const auto& server_options : rhs.servers_options) {
      Node server_options_node;
      server_options_node["func_name"] = server_options.func_name;
      server_options_node["qos"] = server_options.qos;
      server_options_node["remapping_rule"] = server_options.remapping_rule;

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

        if (client_options_node["qos"]) {
          client_options.qos = client_options_node["qos"].as<Options::QosOptions>();
        }

        if (client_options_node["remapping_rule"]) {
          client_options.remapping_rule = client_options_node["remapping_rule"].as<std::string>();
        }
        rhs.clients_options.emplace_back(std::move(client_options));
      }
    }

    if (node["servers_options"] && node["servers_options"].IsSequence()) {
      for (const auto& server_options_node : node["servers_options"]) {
        auto server_options = Options::ServerOptions{
            .func_name = server_options_node["func_name"].as<std::string>()};

        if (server_options_node["qos"]) {
          server_options.qos = server_options_node["qos"].as<Options::QosOptions>();
        }

        if (server_options_node["remapping_rule"]) {
          server_options.remapping_rule = server_options_node["remapping_rule"].as<std::string>();
        }

        rhs.servers_options.emplace_back(std::move(server_options));
      }
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::ros2_plugin {

void Ros2RpcBackend::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Ros2 Rpc backend can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  if (!options_.timeout_executor.empty()) {
    AIMRT_CHECK_ERROR_THROW(
        get_executor_func_,
        "Get executor function is not set before initialize.");

    timeout_executor_ = get_executor_func_(options_.timeout_executor);

    AIMRT_CHECK_ERROR_THROW(
        timeout_executor_,
        "Get timeout executor '{}' failed.", options_.timeout_executor);

    AIMRT_TRACE("Ros rpc backend enable the timeout function, use '{}' as timeout executor.",
                options_.timeout_executor);
  } else {
    AIMRT_TRACE("Ros rpc backend does not enable the timeout function.");
  }

  options_node = options_;
}

void Ros2RpcBackend::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");

  for (auto& itr : ros2_adapter_client_map_)
    itr.second->Start();

  for (auto& itr : ros2_adapter_server_map_)
    itr.second->Start();

  for (auto& itr : ros2_adapter_wrapper_client_map_)
    itr.second->Start();

  for (auto& itr : ros2_adapter_wrapper_server_map_)
    itr.second->Start();
}

void Ros2RpcBackend::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  for (auto& itr : ros2_adapter_client_map_)
    itr.second->Shutdown();

  for (auto& itr : ros2_adapter_server_map_)
    itr.second->Shutdown();

  for (auto& itr : ros2_adapter_wrapper_client_map_)
    itr.second->Shutdown();

  for (auto& itr : ros2_adapter_wrapper_server_map_)
    itr.second->Shutdown();

  ros2_adapter_client_map_.clear();
  ros2_adapter_server_map_.clear();
  ros2_adapter_wrapper_client_map_.clear();
  ros2_adapter_wrapper_server_map_.clear();

  ros2_node_ptr_.reset();
}

rclcpp::QoS Ros2RpcBackend::GetQos(const Options::QosOptions& qos_option) {
  rclcpp::QoS qos(qos_option.depth);

  if (qos_option.history == "keep_last") {
    qos.keep_last(qos_option.depth);
  } else if (qos_option.history == "keep_all") {
    qos.history(rclcpp::HistoryPolicy::KeepAll);
  } else {
    qos.history(rclcpp::HistoryPolicy::SystemDefault);
  }
  if (qos_option.reliability == "reliable") {
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  } else if (qos_option.reliability == "best_effort") {
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  } else {
    qos.reliability(rclcpp::ReliabilityPolicy::SystemDefault);
  }

  if (qos_option.durability == "volatile") {
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
  } else if (qos_option.durability == "transient_local") {
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  } else {
    qos.durability(rclcpp::DurabilityPolicy::SystemDefault);
  }

  if (qos_option.liveliness == "automatic") {
    qos.liveliness(rclcpp::LivelinessPolicy::Automatic);
  } else if (qos_option.liveliness == "manual_by_topic") {
    qos.liveliness(rclcpp::LivelinessPolicy::ManualByTopic);
  } else {
    qos.liveliness(rclcpp::LivelinessPolicy::SystemDefault);
  }

  if (qos_option.deadline != -1) {
    qos.deadline(rclcpp::Duration::from_nanoseconds(qos_option.deadline * 1000000));
  }

  if (qos_option.lifespan != -1) {
    qos.lifespan(rclcpp::Duration::from_nanoseconds(qos_option.lifespan * 1000000));
  }

  if (qos_option.liveliness_lease_duration != -1) {
    qos.liveliness_lease_duration(rclcpp::Duration::from_nanoseconds(qos_option.liveliness_lease_duration * 1000000));
  }
  return qos;
}

std::string Ros2RpcBackend::GetRemappedFuncName(const std::string& input_string, const std::string& matching_rule, const std::string& remapping_rule) {
  // in this case, means do not need to remap
  if (remapping_rule.empty()) {
    return input_string;
  }

  // in this case, means need to remap but matching rule is empty
  if (!matching_rule.empty() && input_string.empty()) {
    AIMRT_WARN("You have not set matching rule for remapping, please add 'func_name' option in yaml file.");
    return input_string;
  }

  std::regex re(matching_rule);
  std::smatch match;

  // in this case, means there are no matched, return the original inoput string
  if (!std::regex_search(input_string, match, re)) {
    AIMRT_WARN("Regex match failed, expr: {}, string: {}", matching_rule, input_string);
    return input_string;
  }

  std::string replaced_func_name = remapping_rule;
  std::regex placeholder(R"(\{(\d+)\})");
  std::smatch placeholder_match;

  std::string::const_iterator search_start(replaced_func_name.cbegin());
  while (std::regex_search(search_start, replaced_func_name.cend(), placeholder_match, placeholder)) {
    int index = std::stoi(placeholder_match[1].str());
    if (index < match.size()) {
      replaced_func_name.replace(placeholder_match.position() + (search_start - replaced_func_name.cbegin()),
                                 placeholder_match.length(),
                                 match[index].str());
      search_start = replaced_func_name.cbegin() + placeholder_match.position() + match[index].length();
    } else {
      AIMRT_WARN("Regex placeholder index out of range, index: {}, match size: {}", index, match.size());
      return input_string;
    }
  }

  // in AimRT, func_name must be start with "<msg_typr>:/", if not, add it.
  auto splited_ret = SplitByDelimiter(std::string(input_string), "/");
  if (replaced_func_name.find(splited_ret.first) != 0) {
    replaced_func_name = splited_ret.first + replaced_func_name;
  }

  AIMRT_INFO("Ros2 func name '{}' is remapped to '{}'", input_string, replaced_func_name);

  return replaced_func_name;
}

bool Ros2RpcBackend::RegisterServiceFunc(
    const runtime::core::rpc::ServiceFuncWrapper& service_func_wrapper) noexcept {
  try {
    if (state_.load() != State::kInit) {
      AIMRT_ERROR("Service func can only be registered when state is 'Init'.");
      return false;
    }

    const auto& info = service_func_wrapper.info;

    // Read QoS and Remapping rule from configuration
    rclcpp::QoS qos = rclcpp::ServicesQoS();
    std::string remapped_func_name = info.func_name;

    auto find_option = std::find_if(
        options_.servers_options.begin(), options_.servers_options.end(),
        [&info](const Options::ServerOptions& service_option) {
          try {
            return std::regex_match(info.func_name.begin(), info.func_name.end(),
                                    std::regex(service_option.func_name, std::regex::ECMAScript));
          } catch (const std::exception& e) {
            AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                       service_option.func_name, info.func_name, e.what());
            return false;
          }
        });

    if (find_option != options_.servers_options.end()) {
      qos = GetQos(find_option->qos);
      remapped_func_name = GetRemappedFuncName(info.func_name, find_option->func_name, find_option->remapping_rule);
    }

    // 前缀是ros2类型的消息
    if (CheckRosFunc(info.func_name)) {
      if (ros2_adapter_server_map_.find(info.func_name) != ros2_adapter_server_map_.end()) {
        AIMRT_WARN(
            "Service '{}' is registered repeatedly in ros2 rpc backend, module '{}', lib path '{}'",
            info.func_name, info.module_name, info.pkg_path);
        return false;
      }

      auto ros2_func_name = GetRealRosFuncName(remapped_func_name);

      auto ros2_adapter_server_ptr = std::make_shared<Ros2AdapterServer>(
          ros2_node_ptr_->get_node_base_interface()->get_shared_rcl_node_handle(),
          service_func_wrapper,
          ros2_func_name,
          qos);
      ros2_node_ptr_->get_node_services_interface()->add_service(
          std::dynamic_pointer_cast<rclcpp::ServiceBase>(ros2_adapter_server_ptr),
          nullptr);

      ros2_adapter_server_map_.emplace(info.func_name,
                                       ros2_adapter_server_ptr);

      AIMRT_INFO("Service '{}' is registered to ros2 rpc backend, ros2 func name is '{}'",
                 info.func_name, ros2_func_name);

      return true;
    }

    // 前缀不是ros2类型的消息
    if (ros2_adapter_wrapper_server_map_.find(info.func_name) != ros2_adapter_wrapper_server_map_.end()) {
      AIMRT_WARN(
          "Service '{}' is registered repeatedly in ros2 rpc backend, module '{}', lib path '{}'",
          info.func_name, info.module_name, info.pkg_path);
      return false;
    }

    auto ros2_func_name = GetRealRosFuncName(remapped_func_name);

    auto ros2_adapter_wrapper_server_ptr = std::make_shared<Ros2AdapterWrapperServer>(
        ros2_node_ptr_->get_node_base_interface()->get_shared_rcl_node_handle(),
        service_func_wrapper,
        ros2_func_name,
        qos);
    ros2_node_ptr_->get_node_services_interface()->add_service(
        std::dynamic_pointer_cast<rclcpp::ServiceBase>(ros2_adapter_wrapper_server_ptr),
        nullptr);

    ros2_adapter_wrapper_server_map_.emplace(info.func_name,
                                             ros2_adapter_wrapper_server_ptr);

    AIMRT_INFO("Service '{}' is registered to ros2 rpc backend, ros2 func name is '{}'",
               info.func_name, ros2_func_name);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

bool Ros2RpcBackend::RegisterClientFunc(
    const runtime::core::rpc::ClientFuncWrapper& client_func_wrapper) noexcept {
  try {
    if (state_.load() != State::kInit) {
      AIMRT_ERROR("Client func can only be registered when state is 'Init'.");
      return false;
    }

    const auto& info = client_func_wrapper.info;

    // 读取QOS的配置
    rclcpp::QoS qos(rclcpp::KeepLast(100));
    qos.reliable();                          // 可靠通信
    qos.lifespan(std::chrono::seconds(30));  // 生命周期为 30 秒

    std::string remapped_func_name = info.func_name;

    auto find_option = std::find_if(
        options_.clients_options.begin(), options_.clients_options.end(),
        [&info](const Options::ClientOptions& client_option) {
          try {
            return std::regex_match(info.func_name.begin(), info.func_name.end(), std::regex(client_option.func_name, std::regex::ECMAScript));
          } catch (const std::exception& e) {
            AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                       client_option.func_name, info.func_name, e.what());
            return false;
          }
        });

    if (find_option != options_.clients_options.end()) {
      qos = GetQos(find_option->qos);
      remapped_func_name = GetRemappedFuncName(info.func_name, find_option->func_name, find_option->remapping_rule);
    }

    // 前缀是ros2类型的消息
    if (CheckRosFunc(info.func_name)) {
      if (ros2_adapter_client_map_.find(info.func_name) != ros2_adapter_client_map_.end()) {
        AIMRT_WARN(
            "Client '{}' is registered repeatedly in ros2 rpc backend, module '{}', lib path '{}'",
            info.func_name, info.module_name, info.pkg_path);
        return false;
      }

      auto ros2_func_name = GetRealRosFuncName(remapped_func_name);

      auto ros2_adapter_client_ptr = std::make_shared<Ros2AdapterClient>(
          ros2_node_ptr_->get_node_base_interface().get(),
          ros2_node_ptr_->get_node_graph_interface(),
          client_func_wrapper,
          ros2_func_name,
          qos,
          timeout_executor_);
      ros2_node_ptr_->get_node_services_interface()->add_client(
          std::dynamic_pointer_cast<rclcpp::ClientBase>(ros2_adapter_client_ptr), nullptr);

      ros2_adapter_client_map_.emplace(info.func_name, ros2_adapter_client_ptr);

      AIMRT_INFO("Client '{}' is registered to ros2 rpc backend, ros2 func name is '{}'",
                 info.func_name, ros2_func_name);

      return true;
    }

    // 前缀不是ros2类型的消息
    if (ros2_adapter_wrapper_client_map_.find(info.func_name) != ros2_adapter_wrapper_client_map_.end()) {
      AIMRT_WARN(
          "Client '{}' is registered repeatedly in ros2 rpc backend, module '{}', lib path '{}'",
          info.func_name, info.module_name, info.pkg_path);
      return false;
    }

    auto ros2_func_name = GetRealRosFuncName(remapped_func_name);

    auto ros2_adapter_wrapper_client_ptr = std::make_shared<Ros2AdapterWrapperClient>(
        ros2_node_ptr_->get_node_base_interface().get(),
        ros2_node_ptr_->get_node_graph_interface(),
        client_func_wrapper,
        ros2_func_name,
        qos,
        timeout_executor_);
    ros2_node_ptr_->get_node_services_interface()->add_client(
        std::dynamic_pointer_cast<rclcpp::ClientBase>(ros2_adapter_wrapper_client_ptr),
        nullptr);

    ros2_adapter_wrapper_client_map_.emplace(info.func_name, ros2_adapter_wrapper_client_ptr);

    AIMRT_INFO("Client '{}' is registered to ros2 rpc backend, ros2 func name is '{}'",
               info.func_name, ros2_func_name);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

void Ros2RpcBackend::Invoke(
    const std::shared_ptr<runtime::core::rpc::InvokeWrapper>& client_invoke_wrapper_ptr) noexcept {
  try {
    if (state_.load() != State::kStart) [[unlikely]] {
      AIMRT_WARN("Method can only be called when state is 'Start'.");
      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    namespace util = aimrt::common::util;

    const auto& info = client_invoke_wrapper_ptr->info;

    // 检查ctx
    auto to_addr = client_invoke_wrapper_ptr->ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_TO_ADDR);
    if (!to_addr.empty()) {
      auto url = util::ParseUrl<std::string_view>(to_addr);
      if (url) {
        if (url->protocol != Name()) [[unlikely]] {
          AIMRT_WARN("Invalid addr: {}", to_addr);
          client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
          return;
        }
      }
    }

    // 前缀是ros2类型的消息
    if (CheckRosFunc(info.func_name)) {
      auto finditr = ros2_adapter_client_map_.find(info.func_name);
      if (finditr == ros2_adapter_client_map_.end()) {
        AIMRT_WARN(
            "Client '{}' unregistered in ros2 rpc backend, module '{}', lib path '{}'",
            info.func_name, info.module_name, info.pkg_path);

        client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
        return;
      }

      if (!(finditr->second->service_is_ready())) {
        AIMRT_WARN("Ros2 service '{}' not ready, module '{}', lib path '{}'",
                   info.func_name, info.module_name, info.pkg_path);

        client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
        return;
      }

      finditr->second->Invoke(client_invoke_wrapper_ptr);
      return;
    }

    // 前缀不是ros2类型的消息
    auto finditr = ros2_adapter_wrapper_client_map_.find(info.func_name);
    if (finditr == ros2_adapter_wrapper_client_map_.end()) {
      AIMRT_WARN(
          "Client '{}' unregistered in ros2 rpc backend, module '{}', lib path '{}'",
          info.func_name, info.module_name, info.pkg_path);

      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    if (!(finditr->second->service_is_ready())) {
      AIMRT_WARN("Ros2 service '{}' not ready, module '{}', lib path '{}'",
                 info.func_name, info.module_name, info.pkg_path);

      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    finditr->second->Invoke(client_invoke_wrapper_ptr);
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

void Ros2RpcBackend::RegisterGetExecutorFunc(
    const std::function<aimrt::executor::ExecutorRef(std::string_view)>& get_executor_func) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  get_executor_func_ = get_executor_func;
}

}  // namespace aimrt::plugins::ros2_plugin