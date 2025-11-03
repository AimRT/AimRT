// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "chn_subscriber_inline_module.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::context::ChnSubscriberInlineModule {
bool ChnSubscriberInlineModule::Initialize(aimrt::CoreRef core) {
  ctx_ptr_ = aimrt::context::Context::CreateContext(core);
  ctx_ptr_->LetMe();
  try {
    auto cfg_path = ctx_ptr_->GetConfigFilePath();
    if (!cfg_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(cfg_path));
      topic_name_ = cfg_node["topic_name"].as<std::string>();
    }

    subscriber_inline_ = ctx_ptr_->CreateSubscriber<aimrt::protocols::example::ExampleEventMsg>(
        topic_name_, [this](std::shared_ptr<const aimrt::protocols::example::ExampleEventMsg> msg) {
          if (aimrt::context::Running()) {
            AIMRT_INFO("Received message: {} (num={})", msg->msg(), msg->num());
          }
        });

    AIMRT_INFO("ChnSubscriberInlineModule initialized on topic '{}'.", topic_name_);
  } catch (const std::exception& e) {
    AIMRT_ERROR("ChnSubscriberInlineModule init failed: {}", e.what());
    return false;
  }

  return true;
}

bool ChnSubscriberInlineModule::Start() {
  AIMRT_INFO("ChnSubscriberInlineModule start succeeded.");
  return true;
}

void ChnSubscriberInlineModule::Shutdown() {
  ctx_ptr_->StopRunning();
  AIMRT_INFO("ChnSubscriberInlineModule shutdown succeeded.");
}

}  // namespace aimrt::examples::cpp::context::ChnSubscriberInlineModule
