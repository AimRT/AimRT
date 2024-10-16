// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>

#include "core/parameter/parameter_handle.h"
#include "core/parameter/parameter_handle_proxy.h"
#include "core/util/module_detail_info.h"
#include "util/log_util.h"
#include "util/string_util.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::runtime::core::parameter {

class ParameterManager {
 public:
  struct Options {};

  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

 public:
  ParameterManager()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~ParameterManager() = default;

  ParameterManager(const ParameterManager&) = delete;
  ParameterManager& operator=(const ParameterManager&) = delete;

  void Initialize(YAML::Node options_node);
  void Start();
  void Shutdown();

  const ParameterHandleProxy& GetParameterHandleProxy(const util::ModuleDetailInfo& module_info);
  const ParameterHandleProxy& GetParameterHandleProxy(std::string_view module_name = "core") {
    return GetParameterHandleProxy(
        util::ModuleDetailInfo{.name = std::string(module_name), .pkg_path = "core"});
  }

  State GetState() const { return state_.load(); }

  std::list<std::pair<std::string, std::string>> GenInitializationReport() const;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  ParameterHandle* GetParameterHandle(std::string_view module_name) const;

 private:
  Options options_;
  std::atomic<State> state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  class ParameterHandleProxyWrap {
   public:
    ParameterHandleProxyWrap()
        : parameter_handle_proxy(parameter_handle) {}

    ParameterHandle parameter_handle;
    ParameterHandleProxy parameter_handle_proxy;
  };

  std::unordered_map<
      std::string,
      std::unique_ptr<ParameterHandleProxyWrap>,
      aimrt::common::util::StringHash,
      std::equal_to<>>
      parameter_handle_proxy_wrap_map_;
};

}  // namespace aimrt::runtime::core::parameter
