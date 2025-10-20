// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "channel_publisher_module/channel_publisher_module.h"

#include "context/context.h"
#include "context/init.h"
#include "yaml-cpp/yaml.h"

#include "event.pb.h"

#include <chrono>
#include <cstdint>
#include <thread>

namespace aimrt::examples::cpp::context::channel_publisher_module {

bool ChannelPublisherModule::Initialize(aimrt::CoreRef core) {
  ctx_ptr_ = GetContext();
  try {
    auto cfg_path = ctx_ptr_->GetRawRef().GetConfigurator().GetConfigFilePath();
    if (!cfg_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(cfg_path));
      topic_name_ = cfg_node["topic_name"].as<std::string>();
      channel_frq_ = cfg_node["channel_frq"].as<double>();
    }

    work_executor_ = ctx_ptr_->GetExecutor("work_executor");
    publisher_ = aimrt::context::init::Publisher<aimrt::protocols::example::ExampleEventMsg>(topic_name_);

    AIMRT_INFO("Channel publisher initialized on topic '{}' with frequency {} Hz.", topic_name_, channel_frq_);
  } catch (const std::exception& e) {
    AIMRT_ERROR("ChannelPublisherModule init failed: {}", e.what());
    return false;
  }

  return true;
}

bool ChannelPublisherModule::Start() {
  if (run_flag_) {
    return true;
  }
  run_flag_.store(true);

  work_executor_.Execute([this]() {
    ctx_ptr_->LetMe();
    RunPublishLoopTask();
  });

  AIMRT_INFO("ChannelPublisherModule start succeeded.");
  return true;
}

void ChannelPublisherModule::Shutdown() {
  if (!run_flag_.exchange(false)) {
    AIMRT_INFO("ChannelPublisherModule shutdown requested while not running.");
    return;
  }
}

void ChannelPublisherModule::RunPublishLoopTask() {
  using namespace std::chrono_literals;

  const auto interval_ms_double = 1000.0 / channel_frq_;

  const auto interval = std::chrono::milliseconds(static_cast<uint32_t>(interval_ms_double));

  uint32_t count = 0;

  while (aimrt::context::Ok()) {
    ++count;
    aimrt::protocols::example::ExampleEventMsg message;
    message.set_msg("Context channel message #" + std::to_string(count));
    message.set_num(static_cast<int32_t>(count));
    publisher_.Publish(message);

    AIMRT_INFO("Published message: {} (num={})", message.msg(), message.num());

    if (interval.count() > 0) {
      std::this_thread::sleep_for(interval);
    }
  }

  AIMRT_INFO("ChannelPublisherModule publish loop exited.");
}

}  // namespace aimrt::examples::cpp::context::channel_publisher_module
