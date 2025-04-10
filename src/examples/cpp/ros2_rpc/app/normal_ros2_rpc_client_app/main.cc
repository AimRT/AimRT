// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <csignal>
#include <iostream>

#include "core/aimrt_core.h"

#include "aimrt_module_cpp_interface/core.h"

#include "RosTestRpc.aimrt_rpc.srv.h"

using namespace aimrt::runtime::core;

bool run_flag = true;

void SignalHandler(int sig) {
  if (sig == SIGINT || sig == SIGTERM) {
    run_flag = false;
    return;
  }

  raise(sig);
};

int32_t main(int32_t argc, char** argv) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  std::cout << "AimRT start." << std::endl;

  try {
    AimRTCore core;

    // Initialize
    AimRTCore::Options options;
    if (argc > 1) options.cfg_file_path = argv[1];

    core.Initialize(options);

    // Create Module
    aimrt::CoreRef module_handle(
        core.GetModuleManager().CreateModule("NormalRpcSyncClientModule"));

    double rpc_frq = 0.5;

    // Read cfg
    auto file_path = module_handle.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
      if (cfg_node["rpc_frq"]) {
        rpc_frq = cfg_node["rpc_frq"].as<double>();
      }
    }

    // Get rpc handle
    auto rpc_handle = module_handle.GetRpcHandle();
    AIMRT_HL_CHECK_ERROR_THROW(module_handle.GetLogger(), rpc_handle, "Get rpc handle failed.");

    // Register rpc client
    bool ret = example_ros2::srv::RegisterRosTestRpcClientFunc(rpc_handle);
    AIMRT_HL_CHECK_ERROR_THROW(module_handle.GetLogger(), ret, "Register client failed.");

    // Create client proxy
    example_ros2::srv::RosTestRpcSyncProxy proxy(rpc_handle);

    // Start
    auto fu = core.AsyncStart();

    // Call RPC Service
    uint32_t count = 0;
    while (run_flag) {
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint32_t>(1000 / rpc_frq)));

      count++;
      AIMRT_HL_INFO(module_handle.GetLogger(), "Loop count : {} -------------------------", count);

      // call rpc
      example_ros2::srv::RosTestRpc_Request req;
      example_ros2::srv::RosTestRpc_Response rsp;
      req.data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

      auto ctx_ptr = proxy.NewContextSharedPtr();
      ctx_ptr->SetTimeout(std::chrono::seconds(3));

      AIMRT_HL_INFO(module_handle.GetLogger(),
                    "start new rpc call. req:\n{}", example_ros2::srv::to_yaml(req));

      auto status = proxy.RosTestRpc(ctx_ptr, req, rsp);

      if (status.OK()) {
        AIMRT_HL_INFO(module_handle.GetLogger(),
                      "Client get rpc ret, status: {}, rsp: {}",
                      status.ToString(), example_ros2::srv::to_yaml(rsp));
      } else {
        AIMRT_HL_WARN(module_handle.GetLogger(),
                      "Client get rpc error ret, status: {}", status.ToString());
      }
    }

    // Shutdown
    core.Shutdown();

    // Wait
    fu.wait();
  } catch (const std::exception& e) {
    std::cout << "AimRT run with exception and exit. " << e.what()
              << std::endl;
    return -1;
  }

  std::cout << "AimRT exit." << std::endl;

  return 0;
}