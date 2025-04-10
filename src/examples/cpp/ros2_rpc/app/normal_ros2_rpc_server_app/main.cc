// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <csignal>
#include <iostream>

#include "core/aimrt_core.h"

#include "aimrt_module_cpp_interface/core.h"

#include "RosTestRpc.aimrt_rpc.srv.h"

using namespace aimrt::runtime::core;

class RosTestRpcSyncServiceImpl : public example_ros2::srv::RosTestRpcSyncService {
 public:
  explicit RosTestRpcSyncServiceImpl(aimrt::CoreRef module_handle) : module_handle_(module_handle) {}
  ~RosTestRpcSyncServiceImpl() override = default;

  aimrt::rpc::Status RosTestRpc(
      aimrt::rpc::ContextRef ctx,
      const example_ros2::srv::RosTestRpc_Request& req,
      example_ros2::srv::RosTestRpc_Response& rsp) override {
    rsp.code = 123;

    AIMRT_HL_INFO(module_handle_.GetLogger(),
                  "Get new rpc call. context: {}, req:\n{}",
                  ctx.ToString(), example_ros2::srv::to_yaml(req));

    return aimrt::rpc::Status();
  }

 private:
  aimrt::CoreRef module_handle_;
};

AimRTCore* global_core_ptr = nullptr;

void SignalHandler(int sig) {
  if (global_core_ptr && (sig == SIGINT || sig == SIGTERM)) {
    global_core_ptr->Shutdown();
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
    global_core_ptr = &core;

    // Initialize
    AimRTCore::Options options;
    if (argc > 1) options.cfg_file_path = argv[1];

    core.Initialize(options);

    // Create Module
    aimrt::CoreRef module_handle(
        core.GetModuleManager().CreateModule("NormalRpcSyncServerModule"));

    // Create service
    auto service_ptr = std::make_shared<RosTestRpcSyncServiceImpl>(module_handle);

    // Register service
    bool ret = module_handle.GetRpcHandle().RegisterService(service_ptr.get());
    AIMRT_HL_CHECK_ERROR_THROW(module_handle.GetLogger(), ret, "Register service failed.");

    AIMRT_HL_INFO(module_handle.GetLogger(), "Register service succeeded.");

    // Start
    core.Start();

    // Shutdown
    core.Shutdown();

    global_core_ptr = nullptr;
  } catch (const std::exception& e) {
    std::cout << "AimRT run with exception and exit. " << e.what()
              << std::endl;
    return -1;
  }

  std::cout << "AimRT exit." << std::endl;

  return 0;
}