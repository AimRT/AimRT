// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "aimrt_module_c_interface/logger/logger_base.h"
#include "aimrt_module_cpp_interface/util/function.h"
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

  // 信息查询类接口
  const std::vector<std::string>& GetModuleNameList() const;
  const std::vector<const util::ModuleDetailInfo*>& GetModuleDetailInfoList() const;

  State GetState() const { return state_.load(); }

  std::list<std::pair<std::string, std::string>> GenInitializationReport() const;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

 private:
  struct ModuleWrapper {
    util::ModuleDetailInfo info;                // 模块配置
    ModuleLoader* loader_ptr = nullptr;         // 所属的动态库
    const aimrt_module_base_t* module_ptr;      // 模块指针
    std::unique_ptr<CoreProxy> core_proxy_ptr;  // 提供给模块的aimrt句柄
  };

  std::optional<Options::ModuleOptions> GetModuleOptions(std::string_view module_name);
  void InitModule(ModuleWrapper* module_wrapper_ptr);

 private:
  Options options_;
  std::atomic<State> state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  CoreProxyConfigurator module_proxy_configurator_;

  // 直接注册的模块（pkg-module）
  std::vector<std::pair<std::string, const aimrt_module_base_t*>> registered_module_vec_;

  // 模块初始化顺序
  std::vector<std::string> module_init_order_;

  // 动态库路径-动态库
  std::unordered_map<std::string, std::unique_ptr<ModuleLoader>> module_loader_map_;

  // 模块名称-模块
  std::unordered_map<std::string, std::unique_ptr<ModuleWrapper>> module_wrapper_map_;

  // 信息查询类变量
  std::vector<const util::ModuleDetailInfo*> module_detail_info_vec_;
};

}  // namespace aimrt::runtime::core::module
