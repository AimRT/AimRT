// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"
#include "aimrt_module_cpp_interface/util/type_support.h"
#include "core/util/type_support_pkg_loader.h"
#include "executor/executor.h"
#include "record_playback_plugin/playback_action.h"
#include "record_playback_plugin/record_action.h"
#include "record_playback_plugin/service.h"
namespace aimrt::plugins::record_playback_plugin {

class RecordPlaybackPlugin : public AimRTCorePluginBase {
 public:
  struct Options {
    std::string service_name;

    struct TypeSupportPkg {
      std::string path;
    };

    std::vector<TypeSupportPkg> type_support_pkgs;
    std::string executor;

    struct RecordActionOptions {
      std::string name;
      YAML::Node options;
    };
    std::vector<RecordActionOptions> record_actions;

    struct PlaybackActionOptions {
      std::string name;
      YAML::Node options;
    };
    std::vector<PlaybackActionOptions> playback_actions;
  };

 public:
  RecordPlaybackPlugin() = default;
  ~RecordPlaybackPlugin() override = default;

  std::string_view Name() const noexcept override { return "record_playback_plugin"; }

  bool Initialize(runtime::core::AimRTCore* core_ptr) noexcept override;
  void Shutdown() noexcept override;

 private:
  void InitTypeSupport(Options::TypeSupportPkg& options);

  void RegisterRpcService();

  void RegisterRecordChannel();
  void RegisterPlaybackChannel();

 private:
  runtime::core::AimRTCore* core_ptr_ = nullptr;

  Options options_;

  bool init_flag_ = false;

  std::unique_ptr<RecordPlaybackServiceImpl> service_ptr_;

  std::vector<std::unique_ptr<runtime::core::util::TypeSupportPkgLoader>> type_support_pkg_loader_vec_;

  aimrt::executor::ExecutorRef timer_executor_ref_;

  struct TypeSupportWrapper {
    const Options::TypeSupportPkg& options;
    aimrt::util::TypeSupportRef type_support_ref;
    runtime::core::util::TypeSupportPkgLoader* loader_ptr;
  };
  std::unordered_map<std::string_view, TypeSupportWrapper> type_support_map_;

  std::unordered_map<std::string_view, std::unique_ptr<RecordAction>> record_action_map_;
  std::unordered_map<std::string_view, std::unique_ptr<PlaybackAction>> playback_action_map_;
};

}  // namespace aimrt::plugins::record_playback_plugin
