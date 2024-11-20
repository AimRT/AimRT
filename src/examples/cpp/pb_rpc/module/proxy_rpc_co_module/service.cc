// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "proxy_rpc_co_module/service.h"
#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/inline_scheduler.h"
#include "aimrt_module_cpp_interface/co/on.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "proxy_rpc_co_module/global.h"

namespace aimrt::examples::cpp::pb_rpc::proxy_rpc_co_module {

co::Task<aimrt::rpc::Status> ExampleCoComplexCoServiceImpl::GetFooData(
    aimrt::rpc::ContextRef ctx,
    const ::aimrt::protocols::example::GetFooDataReq& req,
    ::aimrt::protocols::example::GetFooDataRsp& rsp) {
  rsp.set_msg("proxy echo " + req.msg());

  co_await Invoke(req.msg());

  AIMRT_INFO("Server handle new rpc call. context: {}, req: {}, return rsp: {}",
             ctx.ToString(), aimrt::Pb2CompactJson(req), aimrt::Pb2CompactJson(rsp));

  co_return aimrt::rpc::Status();
}

co::Task<aimrt::rpc::Status> ExampleCoComplexCoServiceImpl::GetBarData(
    aimrt::rpc::ContextRef ctx,
    const ::aimrt::protocols::example::GetBarDataReq& req,
    ::aimrt::protocols::example::GetBarDataRsp& rsp) {
  rsp.set_msg("bar echo " + req.msg());

  AIMRT_INFO("Server handle new rpc call. context: {}, req: {}, return rsp: {}",
             ctx.ToString(), aimrt::Pb2CompactJson(req), aimrt::Pb2CompactJson(rsp));
  co_return aimrt::rpc::Status();
}

co::Task<void> ExampleCoComplexCoServiceImpl::Invoke(const std::string& msg) {
  try {
    co::AimRTScheduler work_thread_pool_scheduler(executor_);

    // Create req and rsp
    aimrt::protocols::example::GetBarDataReq req;
    aimrt::protocols::example::GetBarDataRsp rsp;
    req.set_msg(msg);

    // Create ctx
    auto ctx_ptr = proxy_->NewContextSharedPtr();
    ctx_ptr->SetTimeout(std::chrono::seconds(3));

    // Call rpc
    auto status = co_await proxy_->GetBarData(ctx_ptr, req, rsp);

    // Check result
    if (status.OK()) {
      AIMRT_INFO("Client get rpc ret, status: {}, rsp: {}", status.ToString(),
                 aimrt::Pb2CompactJson(rsp));
    } else {
      AIMRT_WARN("Client get rpc error ret, status: {}", status.ToString());
    }

  } catch (const std::exception& e) {
    AIMRT_ERROR("Exit  with exception, {}", e.what());
  }

  co_return;
};

}  // namespace aimrt::examples::cpp::pb_rpc::proxy_rpc_co_module