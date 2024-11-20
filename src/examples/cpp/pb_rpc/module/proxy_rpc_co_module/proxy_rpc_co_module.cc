// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "proxy_rpc_co_module/proxy_rpc_co_module.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "proxy_rpc_co_module/filter.h"
#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::pb_rpc::proxy_rpc_co_module {

bool ProxyRpcCoModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  try {
    // Read cfg
    std::string file_path = std::string(core_.GetConfigurator().GetConfigFilePath());
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(file_path);

      if (cfg_node["service_name_for_client"]) {
        service_name_for_client_ = cfg_node["service_name_for_client"].as<std::string>();
      }
      if (cfg_node["service_name_for_server"]) {
        service_name_for_server_ = cfg_node["service_name_for_server"].as<std::string>();
      }
    }

    // Get executor handle
    executor_ = core_.GetExecutorManager().GetExecutor("work_thread_pool");
    AIMRT_CHECK_ERROR_THROW(executor_ && executor_.SupportTimerSchedule(),
                            "Get executor 'work_thread_pool' failed.");

    // Get rpc handle
    auto rpc_handle = core_.GetRpcHandle();
    AIMRT_CHECK_ERROR_THROW(rpc_handle, "Get rpc handle failed.");

    // Register rpc client
    bool ret = false;
    if (service_name_for_client_.empty()) {
      ret = aimrt::protocols::example::RegisterExampleServiceClientFunc(rpc_handle);
    } else {
      ret = aimrt::protocols::example::RegisterExampleServiceClientFunc(rpc_handle, service_name_for_client_);
    }

    AIMRT_CHECK_ERROR_THROW(ret, "Register client failed.");

    // Create rpc proxy
    proxy_ = std::make_shared<aimrt::protocols::example::ExampleServiceCoProxy>(rpc_handle);

    if (!service_name_for_client_.empty()) {
      proxy_->SetServiceName(service_name_for_client_);
    }

    // Register filter
    proxy_->RegisterFilter([this](aimrt::rpc::ContextRef ctx,
                                  const void* req_ptr, void* rsp_ptr,
                                  const aimrt::rpc::CoRpcHandle& next)
                               -> co::Task<aimrt::rpc::Status> {
      // debuglog
      AIMRT_INFO("Client start new rpc call. context: {}, req: {}",
                 ctx.ToString(), aimrt::Pb2CompactJson(*static_cast<const google::protobuf::Message*>(req_ptr)));
      const auto& status = co_await next(ctx, req_ptr, rsp_ptr);
      if (status.OK()) {
        AIMRT_INFO("Client get rpc ret, status: {}, rsp: {}", status.ToString(),
                   aimrt::Pb2CompactJson(*static_cast<const google::protobuf::Message*>(rsp_ptr)));
      } else {
        AIMRT_WARN("Client get rpc error ret, status: {}", status.ToString());
      }

      co_return status;
    });

    proxy_->RegisterFilter([this](aimrt::rpc::ContextRef ctx,
                                  const void* req_ptr, void* rsp_ptr,
                                  const aimrt::rpc::CoRpcHandle& next)
                               -> co::Task<aimrt::rpc::Status> {
      // timecost count
      auto begin_time = std::chrono::steady_clock::now();
      const auto& status = co_await next(ctx, req_ptr, rsp_ptr);
      auto end_time = std::chrono::steady_clock::now();

      AIMRT_INFO("Client rpc time cost {} us",
                 std::chrono::duration_cast<std::chrono::microseconds>(end_time - begin_time).count());

      co_return status;
    });

    SetLogger(core_.GetLogger());

    // Create service
    service_ptr_ = std::make_shared<ExampleCoComplexCoServiceImpl>(proxy_, executor_);

    // Register filter
    service_ptr_->RegisterFilter(DebugLogServerFilter);
    service_ptr_->RegisterFilter(TimeCostLogServerFilter);

    // Register service

    bool ret_srv;

    if (service_name_for_server_.empty()) {
      ret_srv = core_.GetRpcHandle().RegisterService(service_ptr_.get());
    } else {
      ret_srv = core_.GetRpcHandle().RegisterService(service_name_for_server_, service_ptr_.get());
    }

    AIMRT_CHECK_ERROR_THROW(ret_srv, "Register service failed.");

    AIMRT_INFO("Init succeeded.");

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  return true;
}

bool ProxyRpcCoModule::Start() {
  return true;
}

void ProxyRpcCoModule::Shutdown() {
  try {
    run_flag_ = false;
    co::SyncWait(scope_.complete());
  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
    return;
  }

  AIMRT_INFO("Shutdown succeeded.");
}

}  // namespace aimrt::examples::cpp::pb_rpc::proxy_rpc_co_module