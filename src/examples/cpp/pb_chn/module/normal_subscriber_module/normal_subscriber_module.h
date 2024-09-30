// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/module_base.h"

#include "event.pb.h"

namespace aimrt::examples::cpp::pb_chn::normal_subscriber_module {

class NormalSubscriberModule : public aimrt::ModuleBase {
 public:
  NormalSubscriberModule() = default;
  ~NormalSubscriberModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "NormalSubscriberModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

  void EventHandle(aimrt::channel::ContextRef ctx,
                   const std::shared_ptr<const aimrt::protocols::example::ExampleEventMsg>& data);

 private:
  aimrt::CoreRef core_;

  std::string topic_name_ = "test_topic";
  aimrt::channel::SubscriberRef subscriber_;
};

}  // namespace aimrt::examples::cpp::pb_chn::normal_subscriber_module
