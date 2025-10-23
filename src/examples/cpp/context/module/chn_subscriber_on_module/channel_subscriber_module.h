// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <string>

#include "aimrt_module_cpp_interface/context/context.h"
#include "aimrt_module_cpp_interface/logger/logger.h"
#include "aimrt_module_cpp_interface/module_base.h"

#include "event.pb.h"

namespace aimrt::examples::cpp::context::ChnSubscriberInlineModule {

class ChnSubscriberInlineModule : public aimrt::ModuleBase {
 public:
  ChnSubscriberInlineModule() = default;
  ~ChnSubscriberInlineModule() override = default;

  ModuleInfo Info() const override { return ModuleInfo{.name = "ContextSubscriberInlineModule"}; }

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  aimrt::logger::LoggerRef GetLogger() const { return ctx_ptr_->GetLogger(); }

 private:
  std::shared_ptr<aimrt::context::Context> ctx_ptr_;
  aimrt::context::res::Subscriber<aimrt::protocols::example::ExampleEventMsg> subscriber_inline_;

  std::string topic_name_;
};

}  // namespace aimrt::examples::cpp::context::ChnSubscriberInlineModule
