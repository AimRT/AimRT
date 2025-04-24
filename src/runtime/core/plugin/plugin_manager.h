// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <functional>
#include <string>

#include "core/plugin/plugin_loader.h"
#include "util/log_util.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::runtime::core::plugin {

class PluginManager {
 public:
  struct Options {
    struct PluginOptions {
      std::string name;
      std::string path;
      YAML::Node options;
    };
    std::vector<PluginOptions> plugins_options;
  };

  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

  using PluginInitFunc = std::function<bool(AimRTCorePluginBase*)>;

 public:
  PluginManager()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~PluginManager() = default;

  PluginManager(const PluginManager&) = delete;
  PluginManager& operator=(const PluginManager&) = delete;

  void Initialize(YAML::Node options_node);
  void Start();
  void Shutdown();

  void RegisterPlugin(AimRTCorePluginBase* plugin);

  void RegisterCorePtr(AimRTCore* core_ptr);

  YAML::Node GetPluginOptionsNode(std::string_view plugin_name) const;
  void UpdatePluginOptionsNode(std::string_view plugin_name, YAML::Node options);

  State GetState() const { return state_.load(); }

  std::list<std::pair<std::string, std::string>> GenInitializationReport() const;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  const std::vector<AimRTCorePluginBase*>& GetUsedPlugin() const;

 private:
  Options options_;
  std::atomic<State> state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  AimRTCore* core_ptr_ = nullptr;

  // Register plugin directly
  std::vector<AimRTCorePluginBase*> registered_plugin_vec_;

  // Load plugins through dynamic libraries
  std::vector<std::unique_ptr<PluginLoader>> plugin_loader_vec_;

  std::vector<AimRTCorePluginBase*> used_plugin_vec_;
};
}  // namespace aimrt::runtime::core::plugin
