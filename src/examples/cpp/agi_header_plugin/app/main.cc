// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>

#include "aimrt_module_cpp_interface/core.h"
#include "core/aimrt_core.h"

#ifdef AIMRT_BUILD_WITH_PROTOBUF
  #include "agi_header_plugin_example.pb.h"
  #include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#endif

#ifdef AIMRT_BUILD_WITH_ROS2
  #include "aimrt_module_ros2_interface/channel/ros2_channel.h"
  #include "ros2_msg/msg/agi_header_example_msg.hpp"
#endif

#ifdef AIMRT_BUILD_WITH_PROTOBUF
using aimrt::examples::agi_header_plugin::AgiHeaderExampleMsg;
#endif
using aimrt::runtime::core::AimRTCore;

int32_t main(int32_t argc, char** argv) {
  std::cout << "AimRT agi_header_plugin example start." << std::endl;

  try {
    AimRTCore core;

    AimRTCore::Options options;
    if (argc > 1) options.cfg_file_path = argv[1];

    core.Initialize(options);

#ifdef AIMRT_BUILD_WITH_PROTOBUF
    aimrt::CoreRef pb_module_handle(
        core.GetModuleManager().CreateModule("AgiHeaderExampleModule"));
    auto pb_publisher = pb_module_handle.GetChannelHandle().GetPublisher("agi_header_topic");
    AIMRT_HL_CHECK_ERROR_THROW(pb_module_handle.GetLogger(),
                               pb_publisher, "Get protobuf publisher failed.");
    auto pb_publisher_proxy =
        std::make_unique<aimrt::channel::PublisherProxy<AgiHeaderExampleMsg>>(pb_publisher);
    bool pb_ret = pb_publisher_proxy->RegisterPublishType();
    AIMRT_HL_CHECK_ERROR_THROW(pb_module_handle.GetLogger(), pb_ret, "Register protobuf publish type failed.");
    AgiHeaderExampleMsg pb_msg;
#endif

#ifdef AIMRT_BUILD_WITH_ROS2
    aimrt::CoreRef ros2_module_handle(
        core.GetModuleManager().CreateModule("AgiHeaderRos2ExampleModule"));
    auto ros2_publisher = ros2_module_handle.GetChannelHandle().GetPublisher("agi_header_ros2_topic");
    AIMRT_HL_CHECK_ERROR_THROW(ros2_module_handle.GetLogger(),
                               ros2_publisher, "Get ros2 publisher failed.");
    auto ros2_publisher_proxy =
        std::make_unique<aimrt::channel::PublisherProxy<ros2_msg::msg::AgiHeaderExampleMsg>>(ros2_publisher);
    bool ros2_ret = ros2_publisher_proxy->RegisterPublishType();
    AIMRT_HL_CHECK_ERROR_THROW(ros2_module_handle.GetLogger(), ros2_ret, "Register ros2 publish type failed.");
    ros2_msg::msg::AgiHeaderExampleMsg ros2_msg;
#endif

    auto fu = core.AsyncStart();

#ifdef AIMRT_BUILD_WITH_PROTOBUF
    pb_msg.set_payload("agi_header_plugin protobuf example");
    pb_publisher_proxy->Publish(pb_msg);
    pb_publisher_proxy->Publish(pb_msg);
#endif

#ifdef AIMRT_BUILD_WITH_ROS2
    ros2_msg.payload = "agi_header_plugin ros2 example";
    ros2_publisher_proxy->Publish(ros2_msg);
#endif

    core.Shutdown();
    fu.wait();

    bool passed = true;

#ifdef AIMRT_BUILD_WITH_PROTOBUF
    const auto& pb_header = pb_msg.header();
    if (pb_header.seq_num() == 0 || pb_header.publisher_name() != "AgiHeaderExampleModule" ||
        pb_header.publish_time().seconds() == 0) {
      std::cout << "Protobuf AgiHeader fill check failed, seq_num=" << pb_header.seq_num()
                << ", publisher_name=" << pb_header.publisher_name()
                << ", publish_time.seconds=" << pb_header.publish_time().seconds()
                << std::endl;
      passed = false;
    } else {
      std::cout << "Protobuf AgiHeader fill check passed, seq_num=" << pb_header.seq_num()
                << ", publisher_name=" << pb_header.publisher_name()
                << ", publish_time.seconds=" << pb_header.publish_time().seconds()
                << std::endl;
    }
#endif

#ifdef AIMRT_BUILD_WITH_ROS2
    const auto& ros2_header = ros2_msg.header;
    if (ros2_header.seq_num == 0 || ros2_header.publisher_name != "AgiHeaderRos2ExampleModule" ||
        ros2_header.publish_time.seconds == 0) {
      std::cout << "ROS2 AgiHeader fill check failed, seq_num=" << ros2_header.seq_num
                << ", publisher_name=" << ros2_header.publisher_name
                << ", publish_time.seconds=" << ros2_header.publish_time.seconds
                << std::endl;
      passed = false;
    } else {
      std::cout << "ROS2 AgiHeader fill check passed, seq_num=" << ros2_header.seq_num
                << ", publisher_name=" << ros2_header.publisher_name
                << ", publish_time.seconds=" << ros2_header.publish_time.seconds
                << std::endl;
    }
#endif

    if (!passed) {
      return -1;
    }
  } catch (const std::exception& e) {
    std::cout << "AimRT agi_header_plugin example run with exception and exit. "
              << e.what() << std::endl;
    return -1;
  }

  std::cout << "AimRT agi_header_plugin example exit." << std::endl;
  return 0;
}
