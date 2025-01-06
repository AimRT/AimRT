// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "record_playback_plugin/record_playback_plugin.h"
#include <chrono>
#include <cstdio>
#include <utility>

#include "aimrt_module_cpp_interface/rpc/rpc_handle.h"
#include "core/aimrt_core.h"
#include "core/channel/channel_backend_tools.h"
#include "executor/timer.h"
#include "log_util.h"
#include "record_playback_plugin/global.h"
#include "util/time_util.h"
#include "util/topic_meta_key.h"

namespace YAML {

template <>
struct convert<aimrt::plugins::record_playback_plugin::RecordPlaybackPlugin::Options> {
  using Options = aimrt::plugins::record_playback_plugin::RecordPlaybackPlugin::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["service_name"] = rhs.service_name;
    node["timer_executor"] = rhs.timer_executor;

    node["type_support_pkgs"] = YAML::Node();
    for (const auto& type_support_pkg : rhs.type_support_pkgs) {
      Node type_support_pkg_node;
      type_support_pkg_node["path"] = type_support_pkg.path;
      node["type_support_pkgs"].push_back(type_support_pkg_node);
    }

    node["record_actions"] = YAML::Node();
    for (const auto& record_action : rhs.record_actions) {
      Node record_action_node;
      record_action_node["name"] = record_action.name;
      record_action_node["options"] = record_action.options;
      node["record_actions"].push_back(record_action_node);
    }

    node["playback_actions"] = YAML::Node();
    for (const auto& playback_action : rhs.playback_actions) {
      Node playback_action_node;
      playback_action_node["name"] = playback_action.name;
      playback_action_node["options"] = playback_action.options;
      node["playback_actions"].push_back(playback_action_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["service_name"])
      rhs.service_name = node["service_name"].as<std::string>();

    if (node["type_support_pkgs"] && node["type_support_pkgs"].IsSequence()) {
      for (const auto& type_support_pkg_node : node["type_support_pkgs"]) {
        auto type_support_pkg = Options::TypeSupportPkg{
            .path = type_support_pkg_node["path"].as<std::string>()};

        rhs.type_support_pkgs.emplace_back(std::move(type_support_pkg));
      }
    }

    if (node["timer_executor"])
      rhs.timer_executor = node["timer_executor"].as<std::string>();

    if (node["record_actions"] && node["record_actions"].IsSequence()) {
      for (const auto& record_action_node : node["record_actions"]) {
        auto record_action = Options::RecordActionOptions{
            .name = record_action_node["name"].as<std::string>(),
            .options = record_action_node["options"]};

        rhs.record_actions.emplace_back(std::move(record_action));
      }
    }

    if (node["playback_actions"] && node["playback_actions"].IsSequence()) {
      for (const auto& playback_action_node : node["playback_actions"]) {
        auto playback_action = Options::PlaybackActionOptions{
            .name = playback_action_node["name"].as<std::string>(),
            .options = playback_action_node["options"]};

        rhs.playback_actions.emplace_back(std::move(playback_action));
      }
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::record_playback_plugin {

bool RecordPlaybackPlugin::Initialize(runtime::core::AimRTCore* core_ptr) noexcept {
  try {
    core_ptr_ = core_ptr;

    YAML::Node plugin_options_node = core_ptr_->GetPluginManager().GetPluginOptionsNode(Name());

    if (plugin_options_node && !plugin_options_node.IsNull()) {
      options_ = plugin_options_node.as<Options>();
    }

    init_flag_ = true;

    // type support
    for (auto& type_support_pkg : options_.type_support_pkgs) {
      // 检查重复pkg
      auto finditr = std::find_if(
          options_.type_support_pkgs.begin(), options_.type_support_pkgs.end(),
          [&type_support_pkg](const auto& op) {
            if (&type_support_pkg == &op) return false;
            return op.path == type_support_pkg.path;
          });
      AIMRT_CHECK_ERROR_THROW(finditr == options_.type_support_pkgs.end(),
                              "Duplicate pkg path {}", type_support_pkg.path);

      InitTypeSupport(type_support_pkg);
    }

    AIMRT_TRACE("Load {} pkg and {} type.",
                type_support_pkg_loader_vec_.size(), type_support_map_.size());

    // record
    for (auto& record_action_options : options_.record_actions) {
      // 检查重复 record
      auto finditr = std::find_if(
          options_.record_actions.begin(), options_.record_actions.end(),
          [&record_action_options](const auto& op) {
            if (&record_action_options == &op) return false;
            return op.name == record_action_options.name;
          });
      AIMRT_CHECK_ERROR_THROW(finditr == options_.record_actions.end(),
                              "Duplicate record action {}", record_action_options.name);

      auto action_ptr = std::make_unique<RecordAction>();

      action_ptr->RegisterGetExecutorFunc(
          [this](std::string_view executor_name) -> aimrt::executor::ExecutorRef {
            return core_ptr_->GetExecutorManager().GetExecutor(executor_name);
          });
      action_ptr->RegisterGetTypeSupportFunc(
          [this](std::string_view msg_type) -> aimrt::util::TypeSupportRef {
            auto finditr = type_support_map_.find(msg_type);
            if (finditr != type_support_map_.end())
              return finditr->second.type_support_ref;
            return {};
          });
      action_ptr->Initialize(record_action_options.options);

      record_action_map_.emplace(record_action_options.name, std::move(action_ptr));
    }

    // playback
    for (auto& playback_action_options : options_.playback_actions) {
      // 检查重复 playback
      auto finditr = std::find_if(
          options_.playback_actions.begin(), options_.playback_actions.end(),
          [&playback_action_options](const auto& op) {
            if (&playback_action_options == &op) return false;
            return op.name == playback_action_options.name;
          });
      AIMRT_CHECK_ERROR_THROW(finditr == options_.playback_actions.end(),
                              "Duplicate playback action {}", playback_action_options.name);

      auto action_ptr = std::make_unique<PlaybackAction>();

      action_ptr->RegisterGetExecutorFunc(
          [this](std::string_view executor_name) -> aimrt::executor::ExecutorRef {
            return core_ptr_->GetExecutorManager().GetExecutor(executor_name);
          });
      action_ptr->RegisterGetTypeSupportFunc(
          [this](std::string_view msg_type) -> aimrt::util::TypeSupportRef {
            auto finditr = type_support_map_.find(msg_type);
            if (finditr != type_support_map_.end())
              return finditr->second.type_support_ref;
            return {};
          });
      action_ptr->Initialize(playback_action_options.options);

      playback_action_map_.emplace(playback_action_options.name, std::move(action_ptr));
    }

    core_ptr_->RegisterHookFunc(
        runtime::core::AimRTCore::State::kPostInitLog,
        [this] {
          SetLogger(aimrt::logger::LoggerRef(
              core_ptr_->GetLoggerManager().GetLoggerProxy().NativeHandle()));
        });

    core_ptr_->RegisterHookFunc(
        runtime::core::AimRTCore::State::kPreInitModules,
        [this] {
          RegisterRpcService();
          RegisterRecordChannel();
          RegisterPlaybackChannel();
        });

    core_ptr_->RegisterHookFunc(
        runtime::core::AimRTCore::State::kPostInitExecutor,
        [this] {
          if (record_action_map_.size() != 0) {
            timer_executor_ref_ = core_ptr_->GetExecutorManager().GetExecutor(options_.timer_executor);
            AIMRT_CHECK_ERROR_THROW(timer_executor_ref_,
                                    "Can not get executor {}.", options_.timer_executor);
            AIMRT_CHECK_ERROR_THROW(timer_executor_ref_.SupportTimerSchedule(),
                                    "Storage executor {} didn't support TimerSchedule!", options_.timer_executor);
          }
          for (auto& itr : record_action_map_) {
            itr.second->InitExecutor(timer_executor_ref_);
          }
          for (auto& itr : playback_action_map_) {
            itr.second->InitExecutor();
          }
        });

    core_ptr_->RegisterHookFunc(
        runtime::core::AimRTCore::State::kPostStart,
        [this] {
          for (auto& itr : record_action_map_) {
            itr.second->Start();
          }
          for (auto& itr : playback_action_map_) {
            itr.second->Start();
          }
        });

    core_ptr_->RegisterHookFunc(
        runtime::core::AimRTCore::State::kPreShutdown,
        [this] {
          for (auto& itr : playback_action_map_)
            itr.second->Shutdown();

          for (auto& itr : record_action_map_)
            itr.second->Shutdown();

          SetLogger(aimrt::logger::GetSimpleLoggerRef());
        });

    plugin_options_node = options_;
    core_ptr_->GetPluginManager().UpdatePluginOptionsNode(Name(), plugin_options_node);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("Initialize failed, {}", e.what());
  }

  return false;
}

void RecordPlaybackPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;

    for (auto& itr : playback_action_map_)
      itr.second->Shutdown();

    for (auto& itr : record_action_map_)
      itr.second->Shutdown();

  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
  }
}

void RecordPlaybackPlugin::InitTypeSupport(Options::TypeSupportPkg& options) {
  auto loader_ptr = std::make_unique<runtime::core::util::TypeSupportPkgLoader>();
  loader_ptr->LoadTypeSupportPkg(options.path);

  options.path = loader_ptr->GetDynamicLib().GetLibFullPath();

  auto type_support_array = loader_ptr->GetTypeSupportArray();

  for (const auto* item : type_support_array) {
    aimrt::util::TypeSupportRef type_support_ref(item);
    auto type_name = type_support_ref.TypeName();

    // 检查重复 type
    auto finditr = type_support_map_.find(type_name);
    if (finditr != type_support_map_.end()) {
      AIMRT_WARN("Duplicate msg type '{}' in {} and {}.",
                 type_name, options.path, finditr->second.options.path);
      continue;
    }

    type_support_map_.emplace(
        type_name,
        TypeSupportWrapper{
            .options = options,
            .type_support_ref = type_support_ref,
            .loader_ptr = loader_ptr.get()});
  }

  type_support_pkg_loader_vec_.emplace_back(std::move(loader_ptr));
}

void RecordPlaybackPlugin::RegisterRpcService() {
  service_ptr_ = std::make_unique<RecordPlaybackServiceImpl>();

  if (!options_.service_name.empty())
    service_ptr_->SetServiceName(options_.service_name);

  service_ptr_->SetRecordActionMap(&record_action_map_);
  service_ptr_->SetPlaybackActionMap(&playback_action_map_);

  auto rpc_handle_ref = aimrt::rpc::RpcHandleRef(
      core_ptr_->GetRpcManager().GetRpcHandleProxy().NativeHandle());

  bool ret = rpc_handle_ref.RegisterService(service_ptr_.get());
  AIMRT_CHECK_ERROR(ret, "Register service failed.");
}

void RecordPlaybackPlugin::RegisterRecordChannel() {
  using namespace aimrt::runtime::core::channel;
  using RecordFunc = std::function<void(uint64_t, MsgWrapper&)>;

  struct Wrapper {
    std::unordered_set<std::string> require_cache_serialization_types;
    std::vector<RecordFunc> record_func_vec;
  };

  std::unordered_map<aimrt::runtime::core::util::TopicMetaKey, Wrapper,
                     aimrt::runtime::core::util::TopicMetaKey::Hash>
      recore_func_map;

  for (auto& record_action_itr : record_action_map_) {
    auto& record_action = *(record_action_itr.second);

    const auto& topic_meta_map = record_action.GetTopicMetaMap();

    // RecordFunc
    for (const auto& topic_meta_itr : topic_meta_map) {
      const auto& topic_meta = topic_meta_itr.second;

      RecordFunc record_func =
          [&record_action, topic_id{topic_meta.id}, serialization_type{topic_meta.serialization_type}](
              uint64_t cur_timestamp, MsgWrapper& msg_wrapper) {
            auto buffer_view_ptr = aimrt::runtime::core::channel::TrySerializeMsgWithCache(msg_wrapper, serialization_type);
            if (!buffer_view_ptr) [[unlikely]] {
              AIMRT_WARN("Can not serialize msg type '{}' with serialization type '{}'.",
                         msg_wrapper.info.msg_type, serialization_type);
              return;
            }
            record_action.AddRecord(
                RecordAction::OneRecord{
                    .timestamp = cur_timestamp,
                    .topic_index = topic_id,
                    .buffer_view_ptr = buffer_view_ptr});
          };

      auto& item = recore_func_map[topic_meta_itr.first];
      item.require_cache_serialization_types.emplace(topic_meta.serialization_type);
      item.record_func_vec.emplace_back(std::move(record_func));
    }
  }

  // Subscribe
  for (auto& recore_func_itr : recore_func_map) {
    const auto& key = recore_func_itr.first;
    auto& wrapper = recore_func_itr.second;

    auto finditr = type_support_map_.find(key.msg_type);

    const auto& type_support_wrapper = finditr->second;
    auto type_support_ref = type_support_wrapper.type_support_ref;

    SubscribeWrapper sub_wrapper;
    sub_wrapper.info = TopicInfo{
        .msg_type = key.msg_type,
        .topic_name = key.topic_name,
        .pkg_path = type_support_wrapper.options.path,
        .module_name = "core",
        .msg_type_support_ref = type_support_ref};

    sub_wrapper.require_cache_serialization_types = wrapper.require_cache_serialization_types;

    // 小优化
    auto& record_func_vec = wrapper.record_func_vec;
    if (record_func_vec.size() == 1) {
      sub_wrapper.callback =
          [record_func{std::move(record_func_vec[0])}](
              MsgWrapper& msg_wrapper, std::function<void()>&& release_callback) {
            auto cur_timestamp = aimrt::common::util::GetCurTimestampNs();

            record_func(cur_timestamp, msg_wrapper);

            release_callback();
          };
    } else {
      sub_wrapper.callback =
          [record_func_vec{std::move(record_func_vec)}](
              MsgWrapper& msg_wrapper, std::function<void()>&& release_callback) {
            auto cur_timestamp = aimrt::common::util::GetCurTimestampNs();

            for (const auto& record_func : record_func_vec)
              record_func(cur_timestamp, msg_wrapper);

            release_callback();
          };
    }

    bool ret = core_ptr_->GetChannelManager().Subscribe(std::move(sub_wrapper));
    AIMRT_CHECK_ERROR_THROW(ret, "Subscribe failed!");
  }
}

void RecordPlaybackPlugin::RegisterPlaybackChannel() {
  using namespace aimrt::runtime::core::channel;

  std::unordered_set<aimrt::runtime::core::util::TopicMetaKey,
                     aimrt::runtime::core::util::TopicMetaKey::Hash>
      playback_topic_meta_set;

  // 处理 playback action
  for (auto& playback_action_itr : playback_action_map_) {
    auto& playback_action = *(playback_action_itr.second);

    const auto& topic_meta_map = playback_action.GetTopicMetaMap();

    for (const auto& topic_meta_itr : topic_meta_map) {
      playback_topic_meta_set.emplace(aimrt::runtime::core::util::TopicMetaKey{
          .topic_name = topic_meta_itr.second.topic_name,
          .msg_type = topic_meta_itr.second.msg_type});
    }
  }

  // RegisterPublishType
  for (const auto& item : playback_topic_meta_set) {
    auto finditr = type_support_map_.find(item.msg_type);

    const auto& type_support_wrapper = finditr->second;
    auto type_support_ref = type_support_wrapper.type_support_ref;

    PublishTypeWrapper pub_type_wrapper;
    pub_type_wrapper.info = TopicInfo{
        .msg_type = item.msg_type,
        .topic_name = item.topic_name,
        .pkg_path = type_support_wrapper.options.path,
        .module_name = "core",
        .msg_type_support_ref = type_support_ref};

    bool ret = core_ptr_->GetChannelManager().RegisterPublishType(std::move(pub_type_wrapper));
    AIMRT_CHECK_ERROR_THROW(ret, "Register publish type failed!");
  }

  // RegisterPubRecordFunc
  for (auto& playback_action_itr : playback_action_map_) {
    struct TopicIndexWrapper {
      const aimrt::runtime::core::channel::PublishTypeWrapper* pub_type_wrapper_ptr;
      std::string serialization_type;
    };
    std::unordered_map<uint64_t, TopicIndexWrapper> topic_id_wrapper_map;

    auto& playback_action = *(playback_action_itr.second);
    const auto& topic_meta_map = playback_action.GetTopicMetaMap();

    for (const auto& topic_meta_itr : topic_meta_map) {
      const auto& topic_meta = topic_meta_itr.second;
      auto finditr = type_support_map_.find(topic_meta.msg_type);
      const auto& type_support_wrapper = finditr->second;

      const auto* pub_type_wrapper_ptr = core_ptr_->GetChannelManager().GetChannelRegistry()->GetPublishTypeWrapperPtr(
          topic_meta.msg_type, topic_meta.topic_name, type_support_wrapper.options.path, "core");
      AIMRT_CHECK_ERROR_THROW(pub_type_wrapper_ptr, "Get publish type failed!");

      topic_id_wrapper_map.emplace(
          topic_meta.id,
          TopicIndexWrapper{
              .pub_type_wrapper_ptr = pub_type_wrapper_ptr,
              .serialization_type = topic_meta.serialization_type});
    }

    playback_action.RegisterPubRecordFunc(
        [this, topic_id_wrapper_map{std::move(topic_id_wrapper_map)}](
            const PlaybackAction::OneRecord& record) {
          auto finditr = topic_id_wrapper_map.find(record.topic_index);
          if (finditr == topic_id_wrapper_map.end()) [[unlikely]] {
            AIMRT_WARN("Invalid topic id: {}.", record.topic_index);
            return;
          }

          const auto* pub_type_wrapper_ptr = finditr->second.pub_type_wrapper_ptr;
          const auto& serialization_type = finditr->second.serialization_type;
          auto msg_type_support_ref = pub_type_wrapper_ptr->info.msg_type_support_ref;

          // TODO: 记录ctx
          aimrt::channel::Context ctx;

          MsgWrapper msg_wrapper{
              .info = pub_type_wrapper_ptr->info,
              .msg_ptr = nullptr,
              .ctx_ref = ctx};

          msg_wrapper.serialization_cache.emplace(serialization_type, record.buffer_view_ptr);

          core_ptr_->GetChannelManager().Publish(std::move(msg_wrapper));
        });
  }
}

}  // namespace aimrt::plugins::record_playback_plugin
