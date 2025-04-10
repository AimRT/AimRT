// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <csignal>
#include <iostream>

#include "core/aimrt_core.h"

#include "aimrt_module_cpp_interface/core.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"

#include "rpc.aimrt_rpc.pb.h"

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

    std::string service_name = "example_service";
    double rpc_frq = 0.5;

    // Read cfg
    auto file_path = module_handle.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
      if (cfg_node["service_name"]) {
        service_name = cfg_node["service_name"].as<std::string>();
      }
      if (cfg_node["rpc_frq"]) {
        rpc_frq = cfg_node["rpc_frq"].as<double>();
      }
    }

    // Get rpc handle
    auto rpc_handle = module_handle.GetRpcHandle();
    AIMRT_HL_CHECK_ERROR_THROW(module_handle.GetLogger(),
                               rpc_handle, "Get rpc handle failed.");

    // Register rpc client
    bool ret = aimrt::protocols::example::RegisterExampleServiceClientFunc(rpc_handle, service_name);
    AIMRT_HL_CHECK_ERROR_THROW(module_handle.GetLogger(), ret, "Register client failed.");

    // Create rpc proxy
    auto proxy = std::make_shared<aimrt::protocols::example::ExampleServiceSyncProxy>(rpc_handle);
    proxy->SetServiceName(service_name);

    // Start
    auto fu = core.AsyncStart();

    // Call RPC Service
    uint32_t count = 0;
    while (run_flag) {
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint32_t>(1000 / rpc_frq)));

      count++;
      AIMRT_HL_INFO(module_handle.GetLogger(), "Loop count : {} -------------------------", count);

      // Create req and rsp
      aimrt::protocols::example::GetFooDataReq req;
      aimrt::protocols::example::GetFooDataRsp rsp;
      req.set_msg("hello world foo, count " + std::to_string(count));

      // Create ctx
      auto ctx_ptr = proxy->NewContextSharedPtr();
      ctx_ptr->SetTimeout(std::chrono::seconds(3));

      AIMRT_HL_INFO(module_handle.GetLogger(),
                    "Client start new rpc call. req: {}", aimrt::Pb2CompactJson(req));

      // Call rpc
      auto status = proxy->GetFooData(ctx_ptr, req, rsp);

      // Check result
      if (status.OK()) {
        AIMRT_HL_INFO(module_handle.GetLogger(),
                      "Client get rpc ret, status: {}, rsp: {}",
                      status.ToString(), aimrt::Pb2CompactJson(rsp));
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