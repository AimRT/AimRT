// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "core/allocator/allocator_proxy.h"
#include "core/util/module_detail_info.h"
#include "util/log_util.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::runtime::core::allocator {

class AllocatorManager {
 public:
  struct Options {};

  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

 public:
  AllocatorManager()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~AllocatorManager() = default;

  AllocatorManager(const AllocatorManager&) = delete;
  AllocatorManager& operator=(const AllocatorManager&) = delete;

  void Initialize(YAML::Node options_node);
  void Start();
  void Shutdown();

  const AllocatorProxy& GetAllocatorProxy(const util::ModuleDetailInfo& module_info);
  const AllocatorProxy& GetAllocatorProxy(std::string_view module_name = "core") {
    return GetAllocatorProxy(
        util::ModuleDetailInfo{.name = std::string(module_name), .pkg_path = "core"});
  }

  State GetState() const { return state_.load(); }

  std::list<std::pair<std::string, std::string>> GenInitializationReport() const;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

 private:
  Options options_;
  std::atomic<State> state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  AllocatorProxy allocator_proxy_;
};

}  // namespace aimrt::runtime::core::allocator
