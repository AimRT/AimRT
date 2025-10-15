// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "aimrt_module_cpp_interface/context/context.h"
#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_cpp_interface/logger/logger.h"
#include "aimrt_module_cpp_interface/module_base.h"
#include "event.pb.h"

namespace aimrt::examples::cpp::context::channel_subscriber_module {

class ChannelSubscriberModule : public aimrt::ModuleBase {
 public:
  ChannelSubscriberModule() = default;
  ~ChannelSubscriberModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "ContextChannelSubscriberModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  aimrt::logger::LoggerRef GetLogger() const { return core_.GetLogger(); }

 private:
  aimrt::CoreRef core_;
  std::shared_ptr<aimrt::context::Context> ctx_;

  aimrt::executor::ExecutorRef work_executor_;
  aimrt::context::res::Channel<aimrt::protocols::example::ExampleEventMsg> subscriber_;

  std::string topic_name_;
};

}  // namespace aimrt::examples::cpp::context::channel_subscriber_module
