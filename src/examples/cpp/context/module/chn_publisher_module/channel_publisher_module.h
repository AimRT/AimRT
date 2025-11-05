// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <future>
#include <memory>
#include <string>

#include "aimrt_module_cpp_interface.h"
#include "event.pb.h"

namespace aimrt::examples::cpp::context::channel_publisher_module {

class ChnPublisherModule : public aimrt::ModuleBase {
 public:
  ChnPublisherModule() = default;
  ~ChnPublisherModule() = default;
  ModuleInfo Info() const override {
    return ModuleInfo{.name = "ContextChannelPublisherModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
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
