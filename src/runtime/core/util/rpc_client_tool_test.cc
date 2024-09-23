// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "core/executor/executor_proxy.h"
#include "core/executor/time_wheel_executor.h"
#include "core/util/rpc_client_tool.h"

namespace aimrt::runtime::core::util {

TEST(RpcClientToolTest, Timeout) {
  RpcClientTool<std::string> rpc_client_tool;

  YAML::Node options_node = YAML::Load(R"str(
dt_us: 1000
wheel_size: [1000, 600]
)str");

  std::string_view name = "timeout_executor";

  executor::TimeWheelExecutor executor;
  executor.RegisterGetExecutorFunc([](auto) {
    return aimrt::executor::ExecutorRef();
  });
  executor.Initialize(name, options_node);
  executor.Start();

  auto executor_proxy = executor::ExecutorProxy(&executor);
  auto executor_ref = aimrt::executor::ExecutorRef(executor_proxy.NativeHandle());
  rpc_client_tool.RegisterTimeoutExecutor(executor_ref);

  std::string s;
  rpc_client_tool.RegisterTimeoutHandle([&s](std::string&& msg) {
    s = std::move(msg);
  });

  // 正常处理
  rpc_client_tool.Record(1, std::chrono::milliseconds(100), "msg 1");
  auto msg = rpc_client_tool.GetRecord(1);
  EXPECT_STREQ(msg->c_str(), "msg 1");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_STREQ(s.c_str(), "");

  // 超时处理
  rpc_client_tool.Record(2, std::chrono::milliseconds(100), "msg 2");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_STREQ(s.c_str(), "msg 2");
  EXPECT_FALSE(rpc_client_tool.GetRecord(2));

  executor.Shutdown();
}

}  // namespace aimrt::runtime::core::util
