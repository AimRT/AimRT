// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <memory>

#include "event.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_plugin_proto/msg/ros_msg_wrapper.hpp"

class RosTestWrapperChannelSubscriber : public rclcpp::Node {
 public:
  RosTestWrapperChannelSubscriber()
      : Node("native_ros2_pb_chn_subscriber") {
    subscription_ = this->create_subscription<ros2_plugin_proto::msg::RosMsgWrapper>(
        "test_topic/pb_3Aaimrt_2Eprotocols_2Eexample_2EExampleEventMsg",
        10,
        [this](ros2_plugin_proto::msg::RosMsgWrapper::UniquePtr wrapper_msg) {
          if (wrapper_msg->serialization_type == "pb") {
            // deserialize protobuf msg from RosMsgWrapper msg
            aimrt::protocols::example::ExampleEventMsg msg;
            std::cout << "wrapper_msg->data.size(): " << wrapper_msg->data.size() << std::endl;
            for (const auto& d : wrapper_msg->data) {
              // 以十六进制格式打印每个字节            
              std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(d) << " ";
              std::cout << std::dec << std::endl;  // 重置为十进制输出

            }
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
