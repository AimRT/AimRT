// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <chrono>
#include <memory>
#include <string>

#include "example_ros2/msg/ros_test_msg.hpp"
#include "rclcpp/rclcpp.hpp"

class RosTestChannelPublisher : public rclcpp::Node {
 public:
  RosTestChannelPublisher()
      : Node("native_ros2_channel_publisher") {
    using namespace std::chrono_literals;

    publisher_ =
        this->create_publisher<example_ros2::msg::RosTestMsg>("test_topic", 10);
    timer_ = this->create_wall_timer(
        500ms,
        [this]() -> void {
          example_ros2::msg::RosTestMsg msg;
          msg.num = count_++;
          RCLCPP_INFO(get_logger(), "Publishing msg:\n%s",
                      example_ros2::msg::to_yaml(msg).c_str());
          publisher_->publish(msg);
        });
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<example_ros2::msg::RosTestMsg>::SharedPtr publisher_;
  size_t count_ = 0;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosTestChannelPublisher>());
  rclcpp::shutdown();
  return 0;
}
