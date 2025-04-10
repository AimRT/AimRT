// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <csignal>
#include <iostream>

#include "core/aimrt_core.h"

#include "aimrt_module_cpp_interface/core.h"
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"

#include "example_ros2/msg/ros_test_msg.hpp"

using namespace aimrt::runtime::core;

bool run_flag = true;

void SignalHandler(int sig) {
  if (sig == SIGINT || sig == SIGTERM) {
    run_flag = false;
    return;
  }

  raise(sig);
};

int32_t main(int32_t argc, char** argv) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  std::cout << "AimRT start." << std::endl;

  try {
    AimRTCore core;

    // Initialize
    AimRTCore::Options options;
    if (argc > 1) options.cfg_file_path = argv[1];

    core.Initialize(options);

    // Create Module
    aimrt::CoreRef module_handle(
        core.GetModuleManager().CreateModule("NormalPublisherModule"));

    std::string topic_name = "test_topic";
    double channel_frq = 0.5;

    // Read cfg
    auto file_path = module_handle.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
      topic_name = cfg_node["topic_name"].as<std::string>();
      channel_frq = cfg_node["channel_frq"].as<double>();
    }

    // Register publish type
    auto publisher = module_handle.GetChannelHandle().GetPublisher(topic_name);
    AIMRT_HL_CHECK_ERROR_THROW(module_handle.GetLogger(),
                               publisher, "Get publisher for topic '{}' failed.", topic_name);

    bool ret = aimrt::channel::RegisterPublishType<example_ros2::msg::RosTestMsg>(publisher);
    AIMRT_HL_CHECK_ERROR_THROW(module_handle.GetLogger(), ret, "Register publishType failed.");

    aimrt::channel::PublisherProxy<example_ros2::msg::RosTestMsg> publisher_proxy(publisher);

    // Start
    auto fu = core.AsyncStart();

    // Publish event
    uint32_t count = 0;
    while (run_flag) {
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint32_t>(1000 / channel_frq)));

      count++;
      AIMRT_HL_INFO(module_handle.GetLogger(), "Loop count : {} -------------------------", count);

      // publish ros event
      example_ros2::msg::RosTestMsg msg;
      msg.data = {1, 2, 3, 4};
      msg.num = count + 1000;

      AIMRT_HL_INFO(module_handle.GetLogger(),
                    "Publish new ros event, data:\n{}", example_ros2::msg::to_yaml(msg));
      publisher_proxy.Publish(msg);
    }

    // Shutdown
    core.Shutdown();

    // Wait
    fu.wait();

  } catch (const std::exception& e) {
    std::cout << "AimRT run with exception and exit. " << e.what()
              << std::endl;
    return -1;
  }

  std::cout << "AimRT exit." << std::endl;

  return 0;
}