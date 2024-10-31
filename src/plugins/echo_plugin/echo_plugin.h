#pragma once

#include <string>
#include <vector>
#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"
#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_cpp_interface/util/type_support.h"
#include "core/aimrt_core.h"
#include "core/channel/channel_backend_tools.h"
#include "core/util/type_support_pkg_loader.h"

#include "echo_plugin/global.h"
#include "echo_plugin/topic_meta_key.h"
#include "yaml-cpp/yaml.h"

namespace aimrt::plugins::echo_plugin {

class EchoPlugin : public AimRTCorePluginBase {
 public:
  struct Options {
    std::string executor;

    struct TopicMeta {
      std::string topic_name;
      std::string msg_type;
      std::string serialization_type;
      std::string echo_type;
    };
    std::vector<TopicMeta> topic_meta_list;
    struct TypeSupportPkg {
      std::string path;
    };
    std::vector<TypeSupportPkg> type_support_pkgs;
  };

 public:
  EchoPlugin() = default;
  ~EchoPlugin() override = default;

  std::string_view Name() const noexcept override { return "echo_plugin"; }

  bool Initialize(runtime::core::AimRTCore* core_ptr) noexcept override;
  void Shutdown() noexcept override;

  const auto& GetTypeSupportMap() const { return type_support_map_; }

 private:
  void InitTypeSupport(Options::TypeSupportPkg& options);
  void InitExecutor();

  void RegisterEchoChannel();
  void RegisterGetTypeSupportFunc(
      const std::function<aimrt::util::TypeSupportRef(std::string_view)>& get_type_support_func);

  void RegisterGetExecutorFunc(
      const std::function<executor::ExecutorRef(std::string_view)>& get_executor_func);

  aimrt::executor::ExecutorRef executor_;

  runtime::core::AimRTCore* core_ptr_ = nullptr;

  Options options_;

  bool init_flag_ = false;

  std::vector<std::unique_ptr<runtime::core::util::TypeSupportPkgLoader>>
      type_support_pkg_loader_vec_;

  struct TypeSupportWrapper {
    const Options::TypeSupportPkg& options;
    aimrt::util::TypeSupportRef type_support_ref;
    runtime::core::util::TypeSupportPkgLoader* loader_ptr;
  };

  std::unordered_map<std::string_view, TypeSupportWrapper> type_support_map_;

  std::unordered_map<TopicMetaKey, TopicMeta, TopicMetaKey::Hash> topic_meta_map_;

  std::function<aimrt::util::TypeSupportRef(std::string_view)> get_type_support_func_;
  std::function<executor::ExecutorRef(std::string_view)> get_executor_func_;

  void Echo(runtime::core::channel::MsgWrapper& msg_wrapper, std::string_view serialization_type);
};

}  // namespace aimrt::plugins::echo_plugin
