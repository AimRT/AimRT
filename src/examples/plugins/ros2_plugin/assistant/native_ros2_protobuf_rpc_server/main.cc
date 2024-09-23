// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <cinttypes>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_plugin_proto/srv/ros_rpc_wrapper.hpp"
#include "rpc.pb.h"

using RosRpcWrapper = ros2_plugin_proto::srv::RosRpcWrapper;

class RosTestRpcWrapperServer : public rclcpp::Node {
 public:
  RosTestRpcWrapperServer(const std::string& name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "Start node '%s'", name.c_str());

    // set QoS
    rclcpp::QoS qos(rclcpp::KeepLast(1000));
    qos.reliable();
    qos.lifespan(std::chrono::seconds(30));
    // rclcpp::QoS qos(rclcpp::KeepAll());

    // create service
    service_ = this->create_service<RosRpcWrapper>(
        "/aimrt_2Eprotocols_2Eexample_2EExampleService/GetFooData",
        std::bind(&RosTestRpcWrapperServer::CoRpcHandle, this, std::placeholders::_1, std::placeholders::_2),
        qos.get_rmw_qos_profile());
  }

 private:
  rclcpp::Service<RosRpcWrapper>::SharedPtr service_;

  void CoRpcHandle(const std::shared_ptr<RosRpcWrapper::Request>& wrapper_req,
                   const std::shared_ptr<RosRpcWrapper::Response>& wrapper_rsp) {
    // deserialize protobuf req from RosRpcWrapper Request
    aimrt::protocols::example::GetFooDataReq req;

    if (wrapper_req->serialization_type != "pb") {
      wrapper_rsp->code = 10000;
      return;
    }

    if (!req.ParseFromArray(wrapper_req->data.data(), wrapper_req->data.size())) {
      wrapper_rsp->code = 10001;
      return;
    }

    // serialize protobuf rsp to RosRpcWrapper Response
    aimrt::protocols::example::GetFooDataRsp rsp;
    rsp.set_msg("echo " + req.msg());
    size_t serialized_size = rsp.ByteSizeLong();

    wrapper_rsp->serialization_type = "pb";
    wrapper_rsp->data.resize(serialized_size);
    rsp.SerializeToArray(wrapper_rsp->data.data(), serialized_size);

    wrapper_rsp->code = 0;
    RCLCPP_INFO(this->get_logger(), "handle service");
  };
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosTestRpcWrapperServer>("native_ros2_protobuf_rpc_server");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
