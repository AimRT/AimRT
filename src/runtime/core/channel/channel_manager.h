// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "core/channel/channel_backend_base.h"
#include "core/channel/channel_handle_proxy.h"
#include "core/util/module_detail_info.h"
#include "core/util/topic_meta_key.h"
#include "util/log_util.h"

namespace aimrt::runtime::core::channel {

class ChannelManager {
 public:
  struct Options {
    struct BackendOptions {
      std::string type;
      YAML::Node options;
    };
    std::vector<BackendOptions> backends_options;

    struct PubTopicOptions {
      std::string topic_name;
      std::vector<std::string> enable_backends;
      std::vector<std::string> enable_filters;
    };
    std::vector<PubTopicOptions> pub_topics_options;

    struct SubTopicOptions {
      std::string topic_name;
      std::vector<std::string> enable_backends;
      std::vector<std::string> enable_filters;
    };
    std::vector<SubTopicOptions> sub_topics_options;
  };

  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

 public:
  ChannelManager()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~ChannelManager() = default;

  ChannelManager(const ChannelManager&) = delete;
  ChannelManager& operator=(const ChannelManager&) = delete;

  void Initialize(YAML::Node options_node);
  void Start();
  void Shutdown();

  const Options& GetOptions() const { return options_; }
  State GetState() const { return state_.load(); }

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  std::list<std::pair<std::string, std::string>> GenInitializationReport() const;

  void RegisterChannelBackend(std::unique_ptr<ChannelBackendBase>&& channel_backend_ptr);

  void RegisterGetExecutorFunc(
      const std::function<aimrt::executor::ExecutorRef(std::string_view)>& get_executor_func);

  const ChannelHandleProxy& GetChannelHandleProxy(const util::ModuleDetailInfo& module_info);
  const ChannelHandleProxy& GetChannelHandleProxy(std::string_view module_name = "core") {
    return GetChannelHandleProxy(util::ModuleDetailInfo{.name = std::string(module_name), .pkg_path = "core"});
  }

  void RegisterPublishFilter(std::string_view name, FrameworkAsyncChannelFilter&& filter);
  void RegisterSubscribeFilter(std::string_view name, FrameworkAsyncChannelFilter&& filter);

  void AddPassedContextMetaKeys(const std::unordered_set<std::string>& keys);

  bool Subscribe(SubscribeWrapper&& wrapper) {
    return channel_backend_manager_.Subscribe(std::move(wrapper));
  }
  bool RegisterPublishType(PublishTypeWrapper&& wrapper) {
    return channel_backend_manager_.RegisterPublishType(std::move(wrapper));
  }
  void Publish(MsgWrapper&& wrapper) {
    channel_backend_manager_.Publish(std::move(wrapper));
  }

  const ChannelRegistry* GetChannelRegistry() const;
  const std::vector<ChannelBackendBase*>& GetUsedChannelBackend() const;

 private:
  void RegisterLocalChannelBackend();
  void RegisterDebugLogFilter();

 private:
  Options options_;
  std::atomic<State> state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  std::function<aimrt::executor::ExecutorRef(std::string_view)> get_executor_func_;

  std::unordered_set<std::string> passed_context_meta_keys_;

  FrameworkAsyncChannelFilterManager publish_filter_manager_;
  FrameworkAsyncChannelFilterManager subscribe_filter_manager_;

  std::unique_ptr<ChannelRegistry> channel_registry_ptr_;

  std::vector<std::unique_ptr<ChannelBackendBase>> channel_backend_vec_;
  std::vector<ChannelBackendBase*> used_channel_backend_vec_;

  ChannelBackendManager channel_backend_manager_;

  class ChannelHandleProxyWrap {
   public:
    ChannelHandleProxyWrap(
        std::string_view input_pkg_path,
        std::string_view input_module_name,
        aimrt::common::util::LoggerWrapper& logger,
        ChannelBackendManager& channel_backend_manager,
        const std::unordered_set<std::string>& passed_context_meta_keys,
        std::atomic_bool& channel_handle_proxy_start_flag)
        : pkg_path(input_pkg_path),
          module_name(input_module_name),
          channel_handle_proxy(
              pkg_path,
              module_name,
              logger,
              channel_backend_manager,
              passed_context_meta_keys,
              channel_handle_proxy_start_flag,
              publisher_proxy_map,
              subscriber_proxy_map) {}

    const std::string pkg_path;
    const std::string module_name;

    ChannelHandleProxy::PublisherProxyMap publisher_proxy_map;
    ChannelHandleProxy::SubscriberProxyMap subscriber_proxy_map;
    ChannelHandleProxy channel_handle_proxy;
  };

  std::atomic_bool channel_handle_proxy_start_flag_ = false;
  std::unordered_map<std::string, std::unique_ptr<ChannelHandleProxyWrap>> channel_handle_proxy_wrap_map_;
};

}  // namespace aimrt::runtime::core::channel