// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <string>

#include "aimrt_module_cpp_interface/context/context.h"
#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_cpp_interface/logger/logger.h"
#include "aimrt_module_cpp_interface/module_base.h"
#include "context/res/channel.h"
#include "event.pb.h"

namespace aimrt::examples::cpp::context::channel_publisher_module {

class ChannelPublisherModule : public aimrt::ModuleBase {
 public:
  ChannelPublisherModule() = default;
  ~ChannelPublisherModule() = default;
  ModuleInfo Info() const override {
    return ModuleInfo{.name = "ContextChannelPublisherModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  aimrt::logger::LoggerRef GetLogger() const { return aimrt::context::details::ExpectContext()->GetRawRef().GetLogger(); }
  void RunPublishLoopTask();

 private:
  std::shared_ptr<aimrt::context::Context> ctx_ptr_;
  aimrt::executor::ExecutorRef work_executor_;
  aimrt::context::res::Publisher<aimrt::protocols::example::ExampleEventMsg> publisher_;

  std::string topic_name_;
  double channel_frq_ = 1.0;

  std::shared_ptr<std::promise<void>> publish_loop_exit_signal_;
  std::future<void> publish_loop_exit_future_;
};

}  // namespace aimrt::examples::cpp::context::channel_publisher_module
