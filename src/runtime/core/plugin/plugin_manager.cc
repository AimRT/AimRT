// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/plugin/plugin_manager.h"

#include "util/string_util.h"

namespace YAML {
template <>
struct convert<aimrt::runtime::core::plugin::PluginManager::Options> {
  using Options = aimrt::runtime::core::plugin::PluginManager::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["plugins"] = YAML::Node();
    for (const auto& plugin_options : rhs.plugins_options) {
      Node plugin_options_node;
      plugin_options_node["name"] = plugin_options.name;
      plugin_options_node["path"] = plugin_options.path;
      plugin_options_node["options"] = plugin_options.options;
      node["plugins"].push_back(plugin_options_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["plugins"] && node["plugins"].IsSequence()) {
      for (const auto& plugin_options_node : node["plugins"]) {
        auto plugin_options = Options::PluginOptions{
            .name = plugin_options_node["name"].as<std::string>(),
            .path = plugin_options_node["path"].as<std::string>()};

        if (plugin_options_node["options"])
          plugin_options.options = plugin_options_node["options"];

        rhs.plugins_options.emplace_back(std::move(plugin_options));
      }
    }
    return true;
  }
};
}  // namespace YAML

namespace aimrt::runtime::core::plugin {

void PluginManager::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      core_ptr_,
      "AimRT core point is not set before PluginManager initialize.");

  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Plugin manager can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  auto tmp_registered_plugin_vec = registered_plugin_vec_;

  // 加载并初始化所有插件
  for (auto& plugin_options : options_.plugins_options) {
    // 检查重复插件名称
    auto finditr = std::find_if(
        options_.plugins_options.begin(), options_.plugins_options.end(),
        [&plugin_options](const auto& op) {
          if (&plugin_options == &op) return false;
          return op.name == plugin_options.name;
        });
    AIMRT_CHECK_ERROR_THROW(finditr == options_.plugins_options.end(),
                            "Duplicate plugin name {}", plugin_options.name);

    AimRTCorePluginBase* plugin_ptr = nullptr;

    if (!plugin_options.path.empty()) {
      // 加载插件
      auto plugin_loader_ptr = std::make_unique<PluginLoader>();
      plugin_loader_ptr->SetLogger(logger_ptr_);
      plugin_loader_ptr->LoadPlugin(plugin_options.path);

      plugin_ptr = plugin_loader_ptr->GetPlugin();

      auto plugin_name = plugin_ptr->Name();

      // 检查插件名称
      AIMRT_CHECK_ERROR_THROW(
          plugin_name == plugin_options.name,
          "Require plugin name '{}', but get plugin name '{}' in lib {}.",
          plugin_options.name, plugin_name, plugin_options.path);

      plugin_options.path = plugin_loader_ptr->GetDynamicLib().GetLibFullPath();

      plugin_loader_vec_.emplace_back(std::move(plugin_loader_ptr));

    } else {
      // 在直接注册的插件中寻找
      auto finditr = std::find_if(
          tmp_registered_plugin_vec.begin(), tmp_registered_plugin_vec.end(),
          [&plugin_options](const AimRTCorePluginBase* plugin) {
            return plugin->Name() == plugin_options.name;
          });

      AIMRT_CHECK_ERROR_THROW(finditr != tmp_registered_plugin_vec.end(),
                              "Can not find plugin {}", plugin_options.name);

      plugin_ptr = *finditr;

      tmp_registered_plugin_vec.erase(finditr);
    }

    // 初始化插件
    bool ret = plugin_ptr->Initialize(core_ptr_);
    AIMRT_CHECK_ERROR_THROW(ret, "Init plugin '{}' failed.", plugin_options.name);

    used_plugin_vec_.emplace_back(plugin_ptr);
    AIMRT_TRACE("Load plugin '{}' succeeded.", plugin_options.name);
  }

  if (!tmp_registered_plugin_vec.empty()) {
    std::vector<std::string> unconfigured_plugins;
    unconfigured_plugins.reserve(tmp_registered_plugin_vec.size());
    for (auto* itr : tmp_registered_plugin_vec)
      unconfigured_plugins.emplace_back(itr->Name());
    AIMRT_ERROR_THROW("Some plugins are not configured, {}",
                      aimrt::common::util::JoinVec(unconfigured_plugins, ","));
  }

  options_node = options_;

  AIMRT_INFO("Plugin manager init complete");
}

void PluginManager::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");

  AIMRT_INFO("Plugin manager start completed.");
}

void PluginManager::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  AIMRT_INFO("Plugin manager shutdown.");

  // 按照反顺序执行Shutdown
  for (auto itr = plugin_loader_vec_.rbegin(); itr != plugin_loader_vec_.rend(); ++itr) {
    (*itr)->GetPlugin()->Shutdown();
  }

  // plugin_loader_vec_ 不能清理掉，有很多回调是从其他dll中注册过来的

  core_ptr_ = nullptr;
}

void PluginManager::RegisterPlugin(AimRTCorePluginBase* plugin) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  AIMRT_CHECK_ERROR_THROW(plugin != nullptr, "Register invalid plugin");

  registered_plugin_vec_.emplace_back(plugin);
}

void PluginManager::RegisterCorePtr(AimRTCore* core_ptr) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  core_ptr_ = core_ptr;
}

YAML::Node PluginManager::GetPluginOptionsNode(std::string_view plugin_name) const {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");

  auto finditr = std::find_if(
      options_.plugins_options.begin(),
      options_.plugins_options.end(),
      [plugin_name](const auto& op) { return plugin_name == op.name; });

  if (finditr != options_.plugins_options.end())
    return finditr->options;

  return YAML::Node();
}

std::list<std::pair<std::string, std::string>> PluginManager::GenInitializationReport() const {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");

  std::vector<std::vector<std::string>> plugin_info_table =
      {{"name", "path"}};

  for (const auto& plugin_options : options_.plugins_options) {
    std::vector<std::string> cur_plugin_info(2);
    cur_plugin_info[0] = plugin_options.name;
    cur_plugin_info[1] = plugin_options.path;
    plugin_info_table.emplace_back(std::move(cur_plugin_info));
  }

  std::list<std::pair<std::string, std::string>> report{
      {"Plugin List", aimrt::common::util::DrawTable(plugin_info_table)}};

  for (const auto& plugin_ptr : used_plugin_vec_) {
    report.splice(report.end(), plugin_ptr->GenInitializationReport());
  }

  return report;
}

const std::vector<AimRTCorePluginBase*>& PluginManager::GetUsedPlugin() const {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");
  return used_plugin_vec_;
}

}  // namespace aimrt::runtime::core::plugin
