// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "channel_subscriber_module/channel_subscriber_module.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"

#include "yaml-cpp/yaml.h"

#include "event.pb.h"

#include <chrono>
#include <cstdint>
#include <thread>

namespace aimrt::examples::cpp::context::channel_subscriber_module {

bool ChannelSubscriberModule::Initialize(aimrt::CoreRef core) {
  core_ = core;
  ctx_ = std::make_shared<aimrt::context::Context>(core_);

  try {
    auto cfg_path = core_.GetConfigurator().GetConfigFilePath();
    if (!cfg_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(cfg_path));
      topic_name_ = cfg_node["topic_name"].as<std::string>();
    }

    work_executor_ = ctx_->GetExecutor("work_executor");

    subscriber_ = ctx_->sub().Init<aimrt::protocols::example::ExampleEventMsg>(topic_name_);
    ctx_->sub().SubscribeInline(
      subscriber_,
      [this](std::shared_ptr<const aimrt::protocols::example::ExampleEventMsg> msg)  {
        if (ctx_->Ok()) {
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
  ctx_->RequireToShutdown();
  AIMRT_INFO("ChannelSubscriberModule shutdown succeeded.");
}

}  // namespace aimrt::examples::cpp::context::channel_subscriber_module
