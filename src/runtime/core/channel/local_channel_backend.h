// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <unordered_set>

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "core/channel/channel_backend_base.h"
#include "util/log_util.h"

namespace aimrt::runtime::core::channel {

class LocalChannelBackend : public ChannelBackendBase {
 public:
  struct Options {
    bool subscriber_use_inline_executor = true;
    std::string subscriber_executor;
  };

  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

 public:
  LocalChannelBackend()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~LocalChannelBackend() override = default;

  std::string_view Name() const noexcept override { return "local"; }

  void Initialize(YAML::Node options_node) override;
  void Start() override;
  void Shutdown() override;

  const Options& GetOptions() const { return options_; }
  State GetState() const { return state_.load(); }

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  void SetChannelRegistry(const ChannelRegistry* channel_registry_ptr) noexcept override {
    channel_registry_ptr_ = channel_registry_ptr;
  }

  bool RegisterPublishType(
      const PublishTypeWrapper& publish_type_wrapper) noexcept override;
  bool Subscribe(const SubscribeWrapper& subscribe_wrapper) noexcept override;
  void Publish(MsgWrapper& msg_wrapper) noexcept override;

  void RegisterGetExecutorFunc(
      const std::function<executor::ExecutorRef(std::string_view)>& get_executor_func);

 private:
  Options options_;
  std::atomic<State> state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  const ChannelRegistry* channel_registry_ptr_ = nullptr;

  std::function<executor::ExecutorRef(std::string_view)> get_executor_func_;
  executor::ExecutorRef subscribe_executor_ref_;

  using SubscribeIndexMap =
      std::unordered_map<
          std::string_view,  // msg_type
          std::unordered_map<
              std::string_view,  // topic
              std::unordered_map<
                  std::string_view,  // lib_path
                  std::unordered_set<
                      std::string_view>>>>;  // module_name
  SubscribeIndexMap subscribe_index_map_;
};

}  // namespace aimrt::runtime::core::channel
