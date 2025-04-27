// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/configurator/configurator_manager.h"

#include <cstdlib>
#include <fstream>

#include "core/util/yaml_tools.h"
#include "util/string_util.h"

namespace YAML {
template <>
struct convert<aimrt::runtime::core::configurator::ConfiguratorManager::Options> {
  using Options = aimrt::runtime::core::configurator::ConfiguratorManager::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["temp_cfg_path"] = rhs.temp_cfg_path.string();

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["temp_cfg_path"])
      rhs.temp_cfg_path = node["temp_cfg_path"].as<std::string>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::runtime::core::configurator {

void ConfiguratorManager::Initialize(
    const std::filesystem::path& cfg_file_path) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Configurator manager can only be initialized once.");

  cfg_file_path_ = cfg_file_path;

  // TODO: yaml-cpp cannot be destructed normally at the end when used with the plugin. No destruction here
  ori_root_options_node_ptr_ = new YAML::Node();
  root_options_node_ptr_ = new YAML::Node();
  user_root_options_node_ptr_ = new YAML::Node();

  auto& ori_root_options_node = *ori_root_options_node_ptr_;
  auto& root_options_node = *root_options_node_ptr_;
  auto& user_root_options_node = *user_root_options_node_ptr_;

  if (!cfg_file_path_.empty()) {
    cfg_file_path_ = std::filesystem::canonical(std::filesystem::absolute(cfg_file_path_));
    AIMRT_INFO("Cfg file path: {}", cfg_file_path_.string());

    std::ifstream file_stream(cfg_file_path_);
    AIMRT_CHECK_ERROR_THROW(file_stream, "Can not open cfg file '{}'.", cfg_file_path_.string());

    std::stringstream file_data;
    file_data << file_stream.rdbuf();
    ori_root_options_node = YAML::Load(aimrt::common::util::ReplaceEnvVars(file_data.str()));
    user_root_options_node = YAML::Clone(ori_root_options_node);
  } else {
    AIMRT_INFO("AimRT start with no cfg file.");
  }

  if (!ori_root_options_node["aimrt"]) {
    ori_root_options_node["aimrt"] = YAML::Node();
  }

  root_options_node["aimrt"] = YAML::Node();

  YAML::Node configurator_options_node = GetAimRTOptionsNode("configurator");
  if (configurator_options_node && !configurator_options_node.IsNull())
    options_ = configurator_options_node.as<Options>();

  if (!(std::filesystem::exists(options_.temp_cfg_path) &&
        std::filesystem::is_directory(options_.temp_cfg_path))) {
    std::filesystem::create_directories(options_.temp_cfg_path);
  }

  configurator_options_node = options_;

  AIMRT_INFO("Configurator manager init complete");
}

void ConfiguratorManager::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");

  AIMRT_INFO("Configurator manager start completed.");
}

void ConfiguratorManager::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  AIMRT_INFO("Configurator manager shutdown.");

  cfg_proxy_map_.clear();
}

YAML::Node ConfiguratorManager::GetOriRootOptionsNode() const {
  return *ori_root_options_node_ptr_;
}

YAML::Node ConfiguratorManager::GetRootOptionsNode() const {
  return *root_options_node_ptr_;
}

YAML::Node ConfiguratorManager::GetUserRootOptionsNode() const {
  return *user_root_options_node_ptr_;
}

const ConfiguratorProxy& ConfiguratorManager::GetConfiguratorProxy(
    const util::ModuleDetailInfo& module_info) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");

  AIMRT_TRACE("Get configurator proxy for module '{}'.", module_info.name);

  auto itr = cfg_proxy_map_.find(module_info.name);
  if (itr != cfg_proxy_map_.end()) return *(itr->second);

  // If the configuration file path is specified directly, use it
  if (!module_info.cfg_file_path.empty()) {
    auto emplace_ret = cfg_proxy_map_.emplace(
        module_info.name,
        std::make_unique<ConfiguratorProxy>(module_info.cfg_file_path));
    return *(emplace_ret.first->second);
  }

  auto& ori_root_options_node = *ori_root_options_node_ptr_;
  auto& root_options_node = *root_options_node_ptr_;

  // If there is this module node in the root configuration file, the content will be generated into the temporary configuration file
  if (ori_root_options_node[module_info.name] &&
      !ori_root_options_node[module_info.name].IsNull()) {
    root_options_node[module_info.name] =
        ori_root_options_node[module_info.name];

    std::filesystem::path temp_cfg_file_path =
        options_.temp_cfg_path /
        ("temp_cfg_file_for_" + module_info.name + ".yaml");
    std::ofstream ofs;
    ofs.open(temp_cfg_file_path, std::ios::trunc);
    ofs << root_options_node[module_info.name];
    ofs.flush();
    ofs.clear();
    ofs.close();

    auto emplace_ret = cfg_proxy_map_.emplace(
        module_info.name,
        std::make_unique<ConfiguratorProxy>(temp_cfg_file_path.string()));
    return *(emplace_ret.first->second);
  }

  return default_cfg_proxy_;
}

YAML::Node ConfiguratorManager::GetAimRTOptionsNode(std::string_view key) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");

  auto& ori_root_options_node = *ori_root_options_node_ptr_;
  auto& root_options_node = *root_options_node_ptr_;

  return root_options_node["aimrt"][key] = ori_root_options_node["aimrt"][key];
}

std::list<std::pair<std::string, std::string>> ConfiguratorManager::GenInitializationReport() const {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");

  auto check_msg = util::CheckYamlNodes(
      GetRootOptionsNode()["aimrt"],
      GetUserRootOptionsNode()["aimrt"],
      "aimrt");

  std::list<std::pair<std::string, std::string>> report{
      {"AimRT Core Option", YAML::Dump((*root_options_node_ptr_)["aimrt"])}};

  if (!check_msg.empty()) {
    report.emplace_back(
        std::pair<std::string, std::string>{"Configuration Warning", check_msg});
  }

  return report;
}

}  // namespace aimrt::runtime::core::configurator