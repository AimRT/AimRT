// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "aimrt_type_support_pkg_c_interface/type_support_pkg_main.h"

#include "aimrt_module_ros2_interface/util/ros2_type_support.h"

#include "example_ros2/msg/benchmark_message.hpp"
#include "example_ros2/msg/benchmark_signal.hpp"
#include "example_ros2/msg/benchmark_status.hpp"
#include "example_ros2/msg/ros_test_data.hpp"
#include "example_ros2/msg/ros_test_msg.hpp"
#include "example_ros2/srv/ros_test_rpc.hpp"

static const aimrt_type_support_base_t* type_support_array[]{
    aimrt::GetRos2MessageTypeSupport<example_ros2::msg::RosTestMsg>(),
    aimrt::GetRos2MessageTypeSupport<example_ros2::msg::RosTestData>(),
    aimrt::GetRos2MessageTypeSupport<example_ros2::msg::BenchmarkMessage>(),
    aimrt::GetRos2MessageTypeSupport<example_ros2::msg::BenchmarkSignal>(),
    aimrt::GetRos2MessageTypeSupport<example_ros2::msg::BenchmarkStatus>(),
};

extern "C" {

size_t AimRTDynlibGetTypeSupportArrayLength() {
  return sizeof(type_support_array) / sizeof(type_support_array[0]);
}

const aimrt_type_support_base_t** AimRTDynlibGetTypeSupportArray() {
  return type_support_array;
}
}
