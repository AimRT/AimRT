// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <csignal>
#include <iostream>

#include "core/aimrt_core.h"

#include "aimrt_module_cpp_interface/core.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"

#include "rpc.aimrt_rpc.pb.h"

using namespace aimrt::runtime::core;

class ExampleServiceSyncServiceImpl : public aimrt::protocols::example::ExampleServiceSyncService {
 public:
  explicit ExampleServiceSyncServiceImpl(aimrt::CoreRef module_handle) : module_handle_(module_handle) {}
  ~ExampleServiceSyncServiceImpl() override = default;

 public:
  aimrt::rpc::Status GetFooData(
      aimrt::rpc::ContextRef ctx_ref,
      const aimrt::protocols::example::GetFooDataReq& req,
      aimrt::protocols::example::GetFooDataRsp& rsp) override {
    rsp.set_msg("echo " + req.msg());

    AIMRT_HL_INFO(module_handle_.GetLogger(), "Server receive GetFooData request, msg: {}", req.msg());

    return aimrt::rpc::Status();
  }

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
        core.GetModuleManager().CreateModule("NormalServerModule"));

    std::string service_name = "example_service";

    // Read cfg
    auto file_path = module_handle.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
      if (cfg_node["service_name"]) {
        service_name = cfg_node["service_name"].as<std::string>();
      }
    }

    // Create service
    auto service_ptr = std::make_shared<ExampleServiceSyncServiceImpl>(module_handle);

    // Register service
    bool ret = module_handle.GetRpcHandle().RegisterService(service_name, service_ptr.get());
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