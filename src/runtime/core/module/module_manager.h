// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "aimrt_module_c_interface/logger/logger_base.h"
#include "aimrt_module_cpp_interface/util/string.h"
#include "core/module/core_proxy.h"
#include "core/module/module_loader.h"
#include "core/util/module_detail_info.h"
#include "util/log_util.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::runtime::core::module {

class ModuleManager {
 public:
  struct Options {
    struct PkgLoaderOptions {
      std::string path;
      std::vector<std::string> disable_modules;
      std::vector<std::string> enable_modules;
    };
    std::vector<PkgLoaderOptions> pkgs_options;

    struct ModuleOptions {
      std::string name;
      bool enable = true;
      aimrt_log_level_t log_lvl = aimrt_log_level_t::AIMRT_LOG_LEVEL_TRACE;
      bool use_default_log_lvl = true;
      std::string cfg_file_path;
    };
    std::vector<ModuleOptions> modules_options;
  };

  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

  using CoreProxyConfigurator = std::function<void(const util::ModuleDetailInfo&, CoreProxy&)>;

 public:
  ModuleManager()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~ModuleManager() = default;

  ModuleManager(const ModuleManager&) = delete;
  ModuleManager& operator=(const ModuleManager&) = delete;

  void Initialize(YAML::Node options_node);
  void Start();
  void Shutdown();

  void RegisterModule(std::string_view pkg, const aimrt_module_base_t* module);
  void RegisterModule(const aimrt_module_base_t* module) { RegisterModule("core", module); }

  const aimrt_core_base_t* CreateModule(std::string_view pkg, aimrt_module_info_t module_info);
  const aimrt_core_base_t* CreateModule(aimrt_module_info_t module_info) {
    return CreateModule("core", module_info);
  }
  const aimrt_core_base_t* CreateModule(std::string_view pkg, std::string_view module_name) {
    return CreateModule(pkg, aimrt_module_info_t{.name = aimrt::util::ToAimRTStringView(module_name)});
  }
  const aimrt_core_base_t* CreateModule(std::string_view module_name) {
    return CreateModule(aimrt_module_info_t{.name = aimrt::util::ToAimRTStringView(module_name)});
  }

  void RegisterCoreProxyConfigurator(CoreProxyConfigurator&& module_proxy_configurator);

  // Information query interface
  const std::vector<std::string>& GetModuleNameList() const;
  const std::vector<const util::ModuleDetailInfo*>& GetModuleDetailInfoList() const;

  State GetState() const { return state_.load(); }

  std::list<std::pair<std::string, std::string>> GenInitializationReport() const;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

 private:
  struct ModuleWrapper {
    util::ModuleDetailInfo info;                // Module configuration
    ModuleLoader* loader_ptr = nullptr;         // Pointer to the parent dynamic library
    const aimrt_module_base_t* module_ptr;      // Module pointer
    std::unique_ptr<CoreProxy> core_proxy_ptr;  // The aimrt handle provided to the module
  };

  std::optional<Options::ModuleOptions> GetModuleOptions(std::string_view module_name);
  void InitModule(ModuleWrapper* module_wrapper_ptr);

 private:
  Options options_;
  std::atomic<State> state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  CoreProxyConfigurator module_proxy_configurator_;

  // Directly registered module (pkg-module)
  std::vector<std::pair<std::string, const aimrt_module_base_t*>> registered_module_vec_;

  // Module initialization order
  std::vector<std::string> module_init_order_;

  // Dynamic library path-dynamic library
  std::unordered_map<std::string, std::unique_ptr<ModuleLoader>> module_loader_map_;

  // Module name - Module
  std::unordered_map<std::string, std::unique_ptr<ModuleWrapper>> module_wrapper_map_;

  // Information query class variables
  std::vector<const util::ModuleDetailInfo*> module_detail_info_vec_;
};

}  // namespace aimrt::runtime::core::module
