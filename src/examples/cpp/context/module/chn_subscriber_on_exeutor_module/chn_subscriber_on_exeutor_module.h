// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <string>

#include "aimrt_module_cpp_interface.h"
#include "aimrt_module_protobuf_interface/aimrt_module_protobuf_interface.h"

#include "event.pb.h"

namespace aimrt::examples::cpp::context::ChnSubscriberOnExecutorModule {

class ChnSubscriberOnExecutorModule : public aimrt::ModuleBase {
 public:
  ChnSubscriberOnExecutorModule() = default;
  ~ChnSubscriberOnExecutorModule() override = default;

  ModuleInfo Info() const override { return ModuleInfo{.name = "ContextSubscriberOnExecutorModule"}; }

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  std::shared_ptr<aimrt::context::Context> ctx_ptr_;
  aimrt::context::res::Subscriber<aimrt::protocols::example::ExampleEventMsg> subscriber_on_executor_;

  aimrt::executor::ExecutorRef work_executor_;

  std::string topic_name_;
};

}  // namespace aimrt::examples::cpp::context::ChnSubscriberOnExecutorModule
