// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "channel_subscriber_module/channel_subscriber_module.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"  // NOLINT(misc-unused-include)
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"  // NOLINT(misc-unused-include)

#include "context/context.h"
#include "context/init.h"
#include "yaml-cpp/yaml.h"

#include "event.pb.h"


namespace aimrt::examples::cpp::context::channel_subscriber_module {

bool ChannelSubscriberModule::Initialize(aimrt::CoreRef core) {
  ctx_ptr_ = GetContext();
  ctx_ptr_->LetMe();

  try {
    auto cfg_path = ctx_ptr_->GetRawRef().GetConfigurator().GetConfigFilePath();
    if (!cfg_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(cfg_path));
      topic_name_ = cfg_node["topic_name"].as<std::string>();
    }

    work_executor_ = aimrt::context::init::CreateExecutor("work_executor");
    subscriber_ = ctx_ptr_->CreateSubscriber<aimrt::protocols::example::ExampleEventMsg>(topic_name_, work_executor_, [this](std::shared_ptr<const aimrt::protocols::example::ExampleEventMsg> msg) {
      if (aimrt::context::Running()) {
        AIMRT_INFO("Received message: {} (num={})", msg->msg(), msg->num());
      }
    });

    subscriber_ = aimrt::context::init::CreateSubscriber<aimrt::protocols::example::ExampleEventMsg>(topic_name_, [this](std::shared_ptr<const aimrt::protocols::example::ExampleEventMsg> msg) {
      if (aimrt::context::Running()) {
        AIMRT_INFO("Received message: {} (num={})", msg->msg(), msg->num());
      }
    });

    AIMRT_INFO("Channel subscriber initialized on topic '{}'.", topic_name_);
  } catch (const std::exception& e) {
    AIMRT_ERROR("ChannelSubscriberModule init failed: {}", e.what());
    return false;
  }

  return true;
}

bool ChannelSubscriberModule::Start() {
  AIMRT_INFO("ChannelSubscriberModule start succeeded.");
  return true;
}

void ChannelSubscriberModule::Shutdown() {
  ctx_ptr_->StopRunning();
  AIMRT_INFO("ChannelSubscriberModule shutdown succeeded.");
}

}  // namespace aimrt::examples::cpp::context::channel_subscriber_module
