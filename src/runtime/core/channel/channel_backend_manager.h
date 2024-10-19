// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "aimrt_module_c_interface/util/function_base.h"
#include "core/channel/channel_backend_base.h"
#include "core/channel/channel_framework_async_filter.h"
#include "core/channel/channel_registry.h"
#include "util/log_util.h"

namespace aimrt::runtime::core::channel {

struct RegisterPublishTypeProxyInfoWrapper {
  std::string_view pkg_path;
  std::string_view module_name;
  std::string_view topic_name;

  const aimrt_type_support_base_t* msg_type_support;
};

struct PublishProxyInfoWrapper {
  std::string_view pkg_path;
  std::string_view module_name;
  std::string_view topic_name;

  aimrt_string_view_t msg_type;
  const aimrt_channel_context_base_t* ctx_ptr;
  const void* msg_ptr;
};

struct SubscribeProxyInfoWrapper {
  std::string_view pkg_path;
  std::string_view module_name;
  std::string_view topic_name;

  const aimrt_type_support_base_t* msg_type_support;
  aimrt_function_base_t* callback;
};

class ChannelBackendManager {
 public:
  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

 public:
  ChannelBackendManager()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~ChannelBackendManager() = default;

  ChannelBackendManager(const ChannelBackendManager&) = delete;
  ChannelBackendManager& operator=(const ChannelBackendManager&) = delete;

  void Initialize();
  void Start();
  void Shutdown();

  State GetState() const { return state_.load(); }

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  void SetChannelRegistry(ChannelRegistry* channel_registry_ptr);

  void SetPublishFiltersRules(
      const std::vector<std::pair<std::string, std::vector<std::string>>>& rules);
  void SetSubscribeFiltersRules(
      const std::vector<std::pair<std::string, std::vector<std::string>>>& rules);

  void SetPublishFrameworkAsyncChannelFilterManager(FrameworkAsyncChannelFilterManager* ptr);
  void SetSubscribeFrameworkAsyncChannelFilterManager(FrameworkAsyncChannelFilterManager* ptr);

  void SetPubTopicsBackendsRules(
      const std::vector<std::pair<std::string, std::vector<std::string>>>& rules);
  void SetSubTopicsBackendsRules(
      const std::vector<std::pair<std::string, std::vector<std::string>>>& rules);

  void RegisterChannelBackend(ChannelBackendBase* channel_backend_ptr);

  // for proxy
  bool Subscribe(SubscribeProxyInfoWrapper&& wrapper);
  bool RegisterPublishType(RegisterPublishTypeProxyInfoWrapper&& wrapper);
  void Publish(PublishProxyInfoWrapper&& wrapper);

  // for framework
  bool Subscribe(SubscribeWrapper&& wrapper);
  bool RegisterPublishType(PublishTypeWrapper&& wrapper);
  void Publish(MsgWrapper&& wrapper);

  using TopicBackendInfoMap = std::unordered_map<std::string_view, std::vector<std::string_view>>;
  TopicBackendInfoMap GetPubTopicBackendInfo() const;
  TopicBackendInfoMap GetSubTopicBackendInfo() const;

 private:
  std::vector<ChannelBackendBase*> GetBackendsByRules(
      std::string_view topic_name,
      const std::vector<std::pair<std::string, std::vector<std::string>>>& rules);

  std::vector<std::string> GetFilterRules(
      std::string_view topic_name,
      const std::vector<std::pair<std::string, std::vector<std::string>>>& rules);

 private:
  std::atomic<State> state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  ChannelRegistry* channel_registry_ptr_;

  // filter
  std::vector<std::pair<std::string, std::vector<std::string>>> publish_filters_rules_;
  std::vector<std::pair<std::string, std::vector<std::string>>> subscribe_filters_rules_;

  FrameworkAsyncChannelFilterManager* publish_filter_manager_ptr_ = nullptr;
  FrameworkAsyncChannelFilterManager* subscribe_filter_manager_ptr_ = nullptr;

  // backend
  std::vector<ChannelBackendBase*> channel_backend_index_vec_;

  std::vector<std::pair<std::string, std::vector<std::string>>> pub_topics_backends_rules_;
  std::vector<std::pair<std::string, std::vector<std::string>>> sub_topics_backends_rules_;

  std::unordered_map<
      std::string,
      std::vector<ChannelBackendBase*>,
      aimrt::common::util::StringHash,
      std::equal_to<>>
      pub_topics_backend_index_map_;

  std::unordered_map<
      std::string,
      std::vector<ChannelBackendBase*>,
      aimrt::common::util::StringHash,
      std::equal_to<>>
      sub_topics_backend_index_map_;
};

}  // namespace aimrt::runtime::core::channel