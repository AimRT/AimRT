// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "RosTestRpc.aimrt_rpc.srv.h"

namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_sync_server_module {

class RosTestRpcSyncServiceImpl : public example_ros2::srv::RosTestRpcSyncService {
 public:
  RosTestRpcSyncServiceImpl() = default;
  ~RosTestRpcSyncServiceImpl() override = default;

  aimrt::rpc::Status RosTestRpc(
      aimrt::rpc::ContextRef ctx,
      const example_ros2::srv::RosTestRpc_Request& req,
      example_ros2::srv::RosTestRpc_Response& rsp) override;
};

}  // namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_sync_server_module
