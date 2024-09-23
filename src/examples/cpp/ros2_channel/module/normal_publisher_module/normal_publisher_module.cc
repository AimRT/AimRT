// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "normal_publisher_module/normal_publisher_module.h"
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"

#include "yaml-cpp/yaml.h"

#include "example_ros2/msg/ros_test_msg.hpp"

namespace aimrt::examples::cpp::ros2_channel::normal_publisher_module {

bool NormalPublisherModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  try {
    // Read cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
      topic_name_ = cfg_node["topic_name"].as<std::string>();
      channel_frq_ = cfg_node["channel_frq"].as<double>();
    }

    // Get executor handle
    executor_ = core_.GetExecutorManager().GetExecutor("work_thread_pool");
    AIMRT_CHECK_ERROR_THROW(executor_ && executor_.SupportTimerSchedule(),
                            "Get executor 'work_thread_pool' failed.");

    // Register publish type
    publisher_ = core_.GetChannelHandle().GetPublisher(topic_name_);
    AIMRT_CHECK_ERROR_THROW(publisher_, "Get publisher for topic '{}' failed.", topic_name_);

    bool ret = aimrt::channel::RegisterPublishType<example_ros2::msg::RosTestMsg>(publisher_);
    AIMRT_CHECK_ERROR_THROW(ret, "Register publishType failed.");

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool NormalPublisherModule::Start() {
  try {
    executor_.Execute(std::bind(&NormalPublisherModule::MainLoop, this));
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Start succeeded.");
  return true;
}

void NormalPublisherModule::Shutdown() {
  try {
    run_flag_ = false;
    stop_sig_.get_future().wait();
  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
    return;
  }

  AIMRT_INFO("Shutdown succeeded.");
}

// Main loop
void NormalPublisherModule::MainLoop() {
  try {
    AIMRT_INFO("Start MainLoop.");

    aimrt::channel::PublisherProxy<example_ros2::msg::RosTestMsg> publisher_proxy(publisher_);

    uint32_t count = 0;
    while (run_flag_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint32_t>(1000 / channel_frq_)));

      count++;
      AIMRT_INFO("loop count : {} -------------------------", count);

      // publish ros event
      example_ros2::msg::RosTestMsg msg;
      msg.data = {1, 2, 3, 4};
      msg.num = count + 1000;

      AIMRT_INFO("Publish new ros event, data:\n{}", example_ros2::msg::to_yaml(msg));
      publisher_proxy.Publish(msg);
    }

    AIMRT_INFO("Exit MainLoop.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("Exit MainLoop with exception, {}", e.what());
  }

  stop_sig_.set_value();
}

}  // namespace aimrt::examples::cpp::ros2_channel::normal_publisher_module
