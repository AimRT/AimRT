// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <memory>

#include "event.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_plugin_proto/msg/ros_msg_wrapper.hpp"

class RosTestWrapperChannelSubscriber : public rclcpp::Node {
 public:
  RosTestWrapperChannelSubscriber()
      : Node("native_ros2_protobuf_channel_subscriber") {
    subscription_ = this->create_subscription<ros2_plugin_proto::msg::RosMsgWrapper>(
        "test_topic/pb_3Aaimrt_2Eprotocols_2Eexample_2EExampleEventMsg",
        10,
        [this](ros2_plugin_proto::msg::RosMsgWrapper::UniquePtr wrapper_msg) {
          if (wrapper_msg->serialization_type == "pb") {
            // deserialize protobuf msg from RosMsgWrapper msg
            aimrt::protocols::example::ExampleEventMsg msg;

            if (msg.ParseFromArray(wrapper_msg->data.data(), wrapper_msg->data.size())) {
              RCLCPP_INFO(get_logger(), "msg: %s", msg.msg().c_str());
            } else {
              RCLCPP_WARN(get_logger(), "deserialize protobuf msg from RosMsgWrapper msg failed!");
            }

          } else {
            RCLCPP_WARN(get_logger(), "unsupport serialization_type: %s",
                        wrapper_msg->serialization_type.c_str());
          }
        });
  }

 private:
  rclcpp::Subscription<ros2_plugin_proto::msg::RosMsgWrapper>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosTestWrapperChannelSubscriber>());
  rclcpp::shutdown();
  return 0;
}
