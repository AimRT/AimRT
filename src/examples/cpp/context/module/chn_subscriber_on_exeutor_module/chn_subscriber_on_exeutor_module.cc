// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "chn_subscriber_on_exeutor_module/chn_subscriber_on_exeutor_module.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::context::ChnSubscriberOnExecutorModule {

bool ChnSubscriberOnExecutorModule::Initialize(aimrt::CoreRef core) {
  ctx_ptr_ = aimrt::context::Context::Letme(core);
  try {
    auto cfg_path = ctx_ptr_->GetConfigFilePath();
    if (!cfg_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(cfg_path));
      topic_name_ = cfg_node["topic_name"].as<std::string>();
    }

    work_executor_ = ctx_ptr_->CreateExecutor("work_executor");

    subscriber_on_executor_ = ctx_ptr_->CreateSubscriber<aimrt::protocols::example::ExampleEventMsg>(topic_name_, work_executor_, [this](std::shared_ptr<const aimrt::protocols::example::ExampleEventMsg> msg) {
      if (aimrt::context::Running()) {
        AIMRT_INFO("Received message: {} (num={})", msg->msg(), msg->num());
      }
    });

    AIMRT_INFO("ChnSubscriberOnExecutorModule initialized on topic '{}'.", topic_name_);
  } catch (const std::exception& e) {
    AIMRT_ERROR("ChnSubscriberOnExecutorModule init failed: {}", e.what());
    return false;
  }

  return true;
}

bool ChnSubscriberOnExecutorModule::Start() {
  AIMRT_INFO("ChnSubscriberOnExecutorModule start succeeded.");
  return true;
}

void ChnSubscriberOnExecutorModule::Shutdown() {
  ctx_ptr_->StopRunning();
  AIMRT_INFO("ChnSubscriberOnExecutorModule shutdown succeeded.");
}

}  // namespace aimrt::examples::cpp::context::ChnSubscriberOnExecutorModule
