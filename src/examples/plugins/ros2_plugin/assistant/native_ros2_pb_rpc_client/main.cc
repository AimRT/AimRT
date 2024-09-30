// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <chrono>
#include <cinttypes>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_plugin_proto/srv/ros_rpc_wrapper.hpp"
#include "rpc.pb.h"

using RosRpcWrapper = ros2_plugin_proto::srv::RosRpcWrapper;
using namespace std::chrono_literals;

class RosTestRpcWrapperClient : public rclcpp::Node {
 public:
  explicit RosTestRpcWrapperClient(
      const std::string &node_name,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions{})
      : Node(node_name, options) {
    // set QoS
    rclcpp::QoS qos(rclcpp::KeepLast(1000));
    qos.reliable();
    qos.lifespan(std::chrono::seconds(30));
    // rclcpp::QoS qos(rclcpp::KeepAll());

    client_ = create_client<RosRpcWrapper>(
        "/aimrt_2Eprotocols_2Eexample_2EExampleService/GetFooData",
        qos.get_rmw_qos_profile());

    timer_ = this->create_wall_timer(
        5s,
        [this]() {
          std::vector<int64_t> pruned_requests;
          // Prune all requests older than 5s.
          size_t pruned = this->client_->prune_requests_older_than(
              std::chrono::system_clock::now() - 5s, &pruned_requests);
          if (pruned) {
            RCLCPP_INFO(
                this->get_logger(),
                "The server hasn't replied for more than 5s, %zu requests were discarded, the discarded requests numbers are:",
                pruned);
            for (const auto &req_num : pruned_requests) {
              RCLCPP_INFO(this->get_logger(), "\t%" PRId64, req_num);
            }
          }
        });
  }

  bool wait_for_service_server() {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    return true;
  }

  void queue_async_request() {
    // serialize protobuf req to RosRpcWrapper Request
    aimrt::protocols::example::GetFooDataReq req;
    req.set_msg("hello world foo.");
    size_t serialized_size = req.ByteSizeLong();

    auto wrapper_req = std::make_shared<RosRpcWrapper::Request>();
    wrapper_req->serialization_type = "pb";
    wrapper_req->data.resize(serialized_size);
    req.SerializeToArray(wrapper_req->data.data(), serialized_size);

    using ServiceResponseFuture = rclcpp::Client<RosRpcWrapper>::SharedFuture;

    auto result = client_->async_send_request(
        wrapper_req,
        [logger = this->get_logger()](ServiceResponseFuture future) {
          const auto &wrapper_rsp = future.get();

          if (wrapper_rsp->code == 0 && wrapper_rsp->serialization_type == "pb") {
            // deserialize protobuf rsp from RosRpcWrapper Response
            aimrt::protocols::example::GetFooDataRsp rsp;

            if (rsp.ParseFromArray(wrapper_rsp->data.data(), wrapper_rsp->data.size())) {
              RCLCPP_INFO(logger, "rsp code: %lu, rsp msg: %s", rsp.code(), rsp.msg().c_str());
            } else {
              RCLCPP_WARN(logger, "deserialize protobuf rsp from RosRpcWrapper Response failed!");
            }

          } else {
            RCLCPP_WARN(logger, "rsp frame code: %lu", wrapper_rsp->code);
          }
        });
    RCLCPP_INFO(
        this->get_logger(),
        "Sending a request to the server (request_id =%" PRId64 "), we're going to let you know the result when ready!",
        result.request_id);
  }

 public:
  rclcpp::Client<RosRpcWrapper>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosTestRpcWrapperClient>("native_ros2_pb_rpc_client");

  if (!node->wait_for_service_server()) {
    return 1;
  }

  std::promise<void> stop_async_spinner;
  std::thread async_spinner_thread(
      [stop_token = stop_async_spinner.get_future(), node]() {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin_until_future_complete(stop_token);
      });

  node->queue_async_request();

  std::this_thread::sleep_for(1s);

  stop_async_spinner.set_value();
  async_spinner_thread.join();
  rclcpp::shutdown();
  return 0;
}
