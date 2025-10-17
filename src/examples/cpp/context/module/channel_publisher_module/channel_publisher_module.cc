// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "channel_publisher_module/channel_publisher_module.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"

#include "yaml-cpp/yaml.h"

#include "event.pb.h"

#include <chrono>
#include <cstdint>
#include <thread>

namespace aimrt::examples::cpp::context::channel_publisher_module {

bool ChannelPublisherModule::Initialize(aimrt::CoreRef core) {
  core_ = core;
  ctx_ = std::make_shared<aimrt::context::Context>(core_);

  try {
    auto cfg_path = core_.GetConfigurator().GetConfigFilePath();
    if (!cfg_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(cfg_path));
      topic_name_ = cfg_node["topic_name"].as<std::string>();
      channel_frq_ = cfg_node["channel_frq"].as<double>();
    }

    work_executor_ = ctx_->GetExecutor("work_executor");

    publisher_ = ctx_->pub().Init<aimrt::protocols::example::ExampleEventMsg>(topic_name_);
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

  while (run_flag_.load(std::memory_order_relaxed)) {
    if (interval.count() > 0) {
      std::this_thread::sleep_for(interval);
    }

    ++count;

    aimrt::protocols::example::ExampleEventMsg message;
    message.set_msg("Context channel message #" + std::to_string(count));
    message.set_num(static_cast<int32_t>(count));

    ctx_->pub().Publish(publisher_, message);

    AIMRT_INFO("Published message: {} (num={})", message.msg(), message.num());
  }

  AIMRT_INFO("ChannelPublisherModule publish loop exited.");
}

}  // namespace aimrt::examples::cpp::context::channel_publisher_module
