// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <csignal>
#include <iostream>

#include "core/aimrt_core.h"

#include "aimrt_module_cpp_interface/core.h"
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"

#include "example_ros2/msg/ros_test_msg.hpp"

using namespace aimrt::runtime::core;

AimRTCore* global_core_ptr = nullptr;

void SignalHandler(int sig) {
  if (global_core_ptr && (sig == SIGINT || sig == SIGTERM)) {
    global_core_ptr->Shutdown();
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
    global_core_ptr = &core;

    // Initialize
    AimRTCore::Options options;
    if (argc > 1) options.cfg_file_path = argv[1];

    core.Initialize(options);

    // Create Module
    aimrt::CoreRef module_handle(
        core.GetModuleManager().CreateModule("NormalSubscriberModule"));

    std::string topic_name = "test_topic";

    // Read cfg
    auto file_path = module_handle.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
      topic_name = cfg_node["topic_name"].as<std::string>();
    }

    // Subscribe
    auto subscriber = module_handle.GetChannelHandle().GetSubscriber(topic_name);
    AIMRT_HL_CHECK_ERROR_THROW(module_handle.GetLogger(),
                               subscriber, "Get subscriber for topic '{}' failed.", topic_name);

    bool ret = aimrt::channel::Subscribe<example_ros2::msg::RosTestMsg>(
        subscriber,
        [module_handle](const std::shared_ptr<const example_ros2::msg::RosTestMsg>& data) {
          AIMRT_HL_INFO(module_handle.GetLogger(),
                        "Receive new ros event, data:\n{}", example_ros2::msg::to_yaml(*data));
        });
    AIMRT_HL_CHECK_ERROR_THROW(module_handle.GetLogger(), ret, "Subscribe failed.");

    // Start
    core.Start();

    // Shutdown
    core.Shutdown();

    global_core_ptr = nullptr;
  } catch (const std::exception& e) {
    std::cout << "AimRT run with exception and exit. " << e.what()
              << std::endl;
    return -1;
  }

  std::cout << "AimRT exit." << std::endl;

  return 0;
}