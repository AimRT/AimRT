// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "RosTestRpc.aimrt_rpc.srv.h"

namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_async_server_module {

class RosTestRpcAsyncServiceImpl : public example_ros2::srv::RosTestRpcAsyncService {
 public:
  RosTestRpcAsyncServiceImpl() = default;
  ~RosTestRpcAsyncServiceImpl() override = default;

  void RosTestRpc(
      aimrt::rpc::ContextRef ctx,
      const example_ros2::srv::RosTestRpc_Request& req,
      example_ros2::srv::RosTestRpc_Response& rsp,
      std::function<void(aimrt::rpc::Status)>&& callback) override;
};

}  // namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_async_server_module
