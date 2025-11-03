// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <chrono>

#include "aimrt_module_protobuf_interface/aimrt_module_protobuf_interface.h"
#include "channel_publisher_module.h"
#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::context::channel_publisher_module {

bool ChnPublisherModule::Initialize(aimrt::CoreRef core) {
  ctx_ptr_ = std::make_shared<aimrt::context::Context>(core);
  ctx_ptr_->LetMe();

  try {
    auto cfg_path = ctx_ptr_->GetConfigFilePath();
    if (!cfg_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(cfg_path));
      topic_name_ = cfg_node["topic_name"].as<std::string>();
      channel_frq_ = cfg_node["channel_frq"].as<double>();
    }

    work_executor_ = ctx_ptr_->CreateExecutor("work_executor");
    publisher_ = ctx_ptr_->CreatePublisher<aimrt::protocols::example::ExampleEventMsg>(topic_name_);
    AIMRT_INFO("Channel publisher initialized on topic '{}' with frequency {} Hz.", topic_name_, channel_frq_);
  } catch (const std::exception& e) {
    AIMRT_ERROR("ChannelPublisherModule init failed: {}", e.what());
    return false;
  }

  return true;
}

bool ChnPublisherModule::Start() {
  work_executor_.Execute([this]() {
    ctx_ptr_->LetMe();
    RunPublishLoopTask();
  });

  AIMRT_INFO("ChannelPublisherModule start.");
  return true;
}

void ChnPublisherModule::Shutdown() {
  ctx_ptr_->StopRunning();
  AIMRT_INFO("ChannelPublisherModule shutdown.");
}

void ChnPublisherModule::RunPublishLoopTask() {
  using namespace std::chrono_literals;

  const auto interval_ms_double = 1000.0 / channel_frq_;

  const auto interval = std::chrono::milliseconds(static_cast<uint32_t>(interval_ms_double));

  uint32_t count = 0;

  while (aimrt::context::Running()) {
    if (interval.count() > 0) {
      std::this_thread::sleep_for(interval);
    }
    ++count;
    aimrt::protocols::example::ExampleEventMsg message;
    message.set_msg("Context channel message #" + std::to_string(count));
    message.set_num(static_cast<int32_t>(count));
    publisher_.Publish(message);

    AIMRT_INFO("Published message: {} (num={})", message.msg(), message.num());
  }

  AIMRT_INFO("ChannelPublisherModule publish loop exited.");
}

}  // namespace aimrt::examples::cpp::context::channel_publisher_module
