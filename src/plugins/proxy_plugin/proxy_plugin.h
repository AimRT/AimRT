// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"
#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_cpp_interface/util/type_support.h"
#include "core/aimrt_core.h"
#include "core/util/topic_meta_key.h"
#include "core/util/type_support_pkg_loader.h"

#include "proxy_action.h"
#include "proxy_plugin/topic_meta.h"

namespace aimrt::plugins::proxy_plugin {

class ProxyPlugin : public AimRTCorePluginBase {
 public:
  struct Options {
    struct ProxyAction {
      std::string name;
      YAML::Node options;
    };
    std::vector<ProxyAction> proxy_actions;
    struct TypeSupportPkg {
      std::string path;
    };
    std::vector<TypeSupportPkg> type_support_pkgs;
    std::string executor;
  };

 public:
  ProxyPlugin() = default;
  ~ProxyPlugin() override = default;

  std::string_view Name() const noexcept override { return "proxy_plugin"; }

  bool Initialize(runtime::core::AimRTCore* core_ptr) noexcept override;
  void Shutdown() noexcept override;

 private:
  void InitTypeSupport(Options::TypeSupportPkg& options);

  void RegisterSubChannel();
  void RegisterPubChannel();

 private:
  runtime::core::AimRTCore* core_ptr_ = nullptr;

  aimrt::executor::ExecutorRef executor_;

  Options options_;

  bool init_flag_ = false;

  struct TypeSupportWrapper {
    const Options::TypeSupportPkg& options;
    aimrt::util::TypeSupportRef type_support_ref;
    runtime::core::util::TypeSupportPkgLoader* loader_ptr;
  };

  struct TopicPubWrapper {
    const aimrt::runtime::core::channel::PublishTypeWrapper* pub_type_wrapper_ptr;
    std::string serialization_type;
  };

  std::unordered_map<std::string_view, TypeSupportWrapper> type_support_map_;

  std::unordered_map<runtime::core::util::TopicMetaKey, TopicPubWrapper, runtime::core::util::TopicMetaKey::Hash> topic_pub_wrapper_map_;

  std::vector<std::unique_ptr<runtime::core::util::TypeSupportPkgLoader>>
      type_support_pkg_loader_vec_;

  std::unordered_map<std::string_view, std::unique_ptr<ProxyAction>> proxy_action_map_;
};

}  // namespace aimrt::plugins::proxy_plugin
