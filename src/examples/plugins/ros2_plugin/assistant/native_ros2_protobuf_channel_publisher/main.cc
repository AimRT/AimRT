// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <chrono>
#include <memory>
#include <string>

#include "event.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_plugin_proto/msg/ros_msg_wrapper.hpp"

class RosTestWrapperChannelPublisher : public rclcpp::Node {
 public:
  RosTestWrapperChannelPublisher()
      : Node("native_ros2_protobuf_channel_publisher") {
    using namespace std::chrono_literals;

    publisher_ = this->create_publisher<ros2_plugin_proto::msg::RosMsgWrapper>(
        "test_topic/pb_3Aaimrt_2Eprotocols_2Eexample_2EExampleEventMsg", 10);
    timer_ = this->create_wall_timer(
        500ms,
        [this]() -> void {
          // serialize protobuf msg to RosMsgWrapper msg
          aimrt::protocols::example::ExampleEventMsg msg;
          msg.set_msg("hello world");
          size_t serialized_size = msg.ByteSizeLong();

          ros2_plugin_proto::msg::RosMsgWrapper wrapper_msg;
          wrapper_msg.serialization_type = "pb";

          wrapper_msg.data.resize(serialized_size);
          msg.SerializeToArray(wrapper_msg.data.data(), serialized_size);

          RCLCPP_INFO(get_logger(), "Publishing msg: %s", msg.msg().c_str());

          publisher_->publish(wrapper_msg);
        });
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ros2_plugin_proto::msg::RosMsgWrapper>::SharedPtr publisher_;
  size_t count_ = 0;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosTestWrapperChannelPublisher>());
  rclcpp::shutdown();
  return 0;
}
