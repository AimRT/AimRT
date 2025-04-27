// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>

#include "core/configurator/configurator_proxy.h"
#include "core/util/module_detail_info.h"
#include "util/log_util.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::runtime::core::configurator {

class ConfiguratorManager {
 public:
  struct Options {
    std::filesystem::path temp_cfg_path = "./cfg/tmp";
  };

  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

 public:
  ConfiguratorManager()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~ConfiguratorManager() = default;

  ConfiguratorManager(const ConfiguratorManager&) = delete;
  ConfiguratorManager& operator=(const ConfiguratorManager&) = delete;

  void Initialize(const std::filesystem::path& cfg_file_path);
  void Start();
  void Shutdown();

  YAML::Node GetOriRootOptionsNode() const;
  YAML::Node GetRootOptionsNode() const;
  YAML::Node GetUserRootOptionsNode() const;

  std::string GetConfigureFilePath() const { return cfg_file_path_.string(); }

  const ConfiguratorProxy& GetConfiguratorProxy(const util::ModuleDetailInfo& module_info);

  YAML::Node GetAimRTOptionsNode(std::string_view key);

  State GetState() const { return state_.load(); }

  std::list<std::pair<std::string, std::string>> GenInitializationReport() const;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

 private:
  std::filesystem::path cfg_file_path_;
  Options options_;
  std::atomic<State> state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  YAML::Node* ori_root_options_node_ptr_;   // Configuration with standardized aimrt nodes and user module nodes
  YAML::Node* root_options_node_ptr_;       // Configuration containing only standardized aimrt nodes
  YAML::Node* user_root_options_node_ptr_;  // Originally loaded configuration

  std::unordered_map<std::string, std::unique_ptr<ConfiguratorProxy>> cfg_proxy_map_;
  ConfiguratorProxy default_cfg_proxy_;
};

}  // namespace aimrt::runtime::core::configurator
