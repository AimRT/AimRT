// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <memory>

#include "example_ros2/msg/ros_test_msg.hpp"
#include "rclcpp/rclcpp.hpp"

class RosTestChannelSubscriber : public rclcpp::Node {
 public:
  RosTestChannelSubscriber()
      : Node("native_ros2_chn_subscriber") {
    subscription_ = this->create_subscription<example_ros2::msg::RosTestMsg>(
        "test_topic",
        10,
        [this](example_ros2::msg::RosTestMsg::UniquePtr msg) {
          RCLCPP_INFO(get_logger(), "Heard msg:\n%s",
                      example_ros2::msg::to_yaml(*msg).c_str());
        });
  }

 private:
  rclcpp::Subscription<example_ros2::msg::RosTestMsg>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosTestChannelSubscriber>());
  rclcpp::shutdown();
  return 0;
}
