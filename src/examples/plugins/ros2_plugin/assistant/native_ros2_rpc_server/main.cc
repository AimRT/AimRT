// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <cinttypes>
#include <memory>

#include "example_ros2/srv/ros_test_rpc.hpp"
#include "rclcpp/rclcpp.hpp"

using RosTestRpc = example_ros2::srv::RosTestRpc;

class RosTestRpcServer : public rclcpp::Node {
 public:
  RosTestRpcServer(const std::string& name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "Start node '%s'", name.c_str());

    // set QoS
    rclcpp::QoS qos(rclcpp::KeepLast(1000));
    qos.reliable();
    qos.lifespan(std::chrono::seconds(30));
    // rclcpp::QoS qos(rclcpp::KeepAll());

    // create service
    service_ = this->create_service<RosTestRpc>(
        "/example_ros2/srv/RosTestRpc",
        std::bind(&RosTestRpcServer::CoRpcHandle, this, std::placeholders::_1,
                  std::placeholders::_2),
        qos.get_rmw_qos_profile());
  }

 private:
  rclcpp::Service<RosTestRpc>::SharedPtr service_;

  void CoRpcHandle(const std::shared_ptr<RosTestRpc::Request>& request,
                   const std::shared_ptr<RosTestRpc::Response>& response) {
    response->code = 123;
    RCLCPP_INFO(this->get_logger(), "handle service");
  };
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosTestRpcServer>("native_ros2_rpc_server");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
