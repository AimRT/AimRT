// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "aimrt_type_support_pkg_c_interface/type_support_pkg_main.h"

#include "aimrt_module_ros2_interface/util/ros2_type_support.h"

#include "aimrt_msgs/msg/joint_command.hpp"
#include "aimrt_msgs/msg/joint_command_array.hpp"
#include "aimrt_msgs/msg/joint_state.hpp"
#include "aimrt_msgs/msg/joint_state_array.hpp"
#include "aimrt_msgs/msg/message_header.hpp"
#include "aimrt_msgs/msg/touch_sensor_state.hpp"
#include "aimrt_msgs/msg/touch_sensor_state_array.hpp"

static const aimrt_type_support_base_t* type_support_array[]{
    aimrt::GetRos2MessageTypeSupport<aimrt_msgs::msg::MessageHeader>(),
    aimrt::GetRos2MessageTypeSupport<aimrt_msgs::msg::TouchSensorStateArray>(),
    aimrt::GetRos2MessageTypeSupport<aimrt_msgs::msg::TouchSensorState>(),
    aimrt::GetRos2MessageTypeSupport<aimrt_msgs::msg::JointStateArray>(),
    aimrt::GetRos2MessageTypeSupport<aimrt_msgs::msg::JointState>(),
    aimrt::GetRos2MessageTypeSupport<aimrt_msgs::msg::JointCommandArray>(),
    aimrt::GetRos2MessageTypeSupport<aimrt_msgs::msg::JointCommand>()};

extern "C" {

size_t AimRTDynlibGetTypeSupportArrayLength() {
  return sizeof(type_support_array) / sizeof(type_support_array[0]);
}

const aimrt_type_support_base_t** AimRTDynlibGetTypeSupportArray() {
  return type_support_array;
}
}