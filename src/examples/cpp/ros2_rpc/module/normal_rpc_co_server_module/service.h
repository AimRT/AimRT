// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "RosTestRpc.aimrt_rpc.srv.h"

namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_co_server_module {

class RosTestRpcServiceImpl : public example_ros2::srv::RosTestRpcCoService {
 public:
  RosTestRpcServiceImpl() = default;
  ~RosTestRpcServiceImpl() override = default;

  co::Task<aimrt::rpc::Status> RosTestRpc(
      aimrt::rpc::ContextRef ctx,
      const example_ros2::srv::RosTestRpc_Request& req,
      example_ros2::srv::RosTestRpc_Response& rsp) override;
};

}  // namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_co_server_module
