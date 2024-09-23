// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "normal_rpc_co_server_module/service.h"

#include "normal_rpc_co_server_module/global.h"

namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_co_server_module {

co::Task<aimrt::rpc::Status> RosTestRpcServiceImpl::RosTestRpc(
    aimrt::rpc::ContextRef ctx,
    const example_ros2::srv::RosTestRpc_Request& req,
    example_ros2::srv::RosTestRpc_Response& rsp) {
  rsp.code = 123;

  AIMRT_INFO("Get new rpc call. context: {}\n, req:\n{}\nreturn rsp:\n{}",
             ctx.ToString(), example_ros2::srv::to_yaml(req), example_ros2::srv::to_yaml(rsp));

  co_return aimrt::rpc::Status();
}

}  // namespace aimrt::examples::cpp::ros2_rpc::normal_rpc_co_server_module
