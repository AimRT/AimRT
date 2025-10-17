// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "normal_subscriber_module/normal_subscriber_module.h"
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::ros2_chn::normal_subscriber_module {

bool NormalSubscriberModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  try {
    // Read cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
      topic_name_ = cfg_node["topic_name"].as<std::string>();
    }
    executor_ = core_.GetExecutorManager().GetExecutor("work_thread_pool");

    // Subscribe
    subscriber_ = core_.GetChannelHandle().GetSubscriber(topic_name_);
    AIMRT_CHECK_ERROR_THROW(subscriber_, "Get subscriber for topic '{}' failed.", topic_name_);

    bool ret = aimrt::channel::Subscribe<example_ros2::msg::RosTestMsg>(
        subscriber_,
        std::bind(&NormalSubscriberModule::EventHandle, this, std::placeholders::_1));
    AIMRT_CHECK_ERROR_THROW(ret, "Subscribe failed.");

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool NormalSubscriberModule::Start() { return true; }

void NormalSubscriberModule::Shutdown() {}

void NormalSubscriberModule::EventHandle(const std::shared_ptr<const example_ros2::msg::RosTestMsg>& data) {
  executor_.Execute([this, data]() {
    AIMRT_INFO("Receive new ros event, data:\n{}", example_ros2::msg::to_yaml(*data));
  });
}

}  // namespace aimrt::examples::cpp::ros2_chn::normal_subscriber_module
