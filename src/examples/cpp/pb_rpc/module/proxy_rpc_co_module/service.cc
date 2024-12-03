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
  try {
    // Create proxy req and rsp
    aimrt::protocols::example::GetBarDataReq proxy_req;
    aimrt::protocols::example::GetBarDataRsp proxy_rsp;
    proxy_req.set_msg(req.msg());

    // Create ctx
    auto ctx_ptr = proxy_->NewContextSharedPtr();
    ctx_ptr->SetTimeout(std::chrono::seconds(3));

    // Call rpc
    auto status = co_await proxy_->GetBarData(ctx_ptr, proxy_req, proxy_rsp);

    rsp.set_code(proxy_rsp.code());
    rsp.set_msg(proxy_rsp.msg());

    AIMRT_INFO(" Server handle new rpc call. context: {}, req: {}, return rsp: {}",
               ctx.ToString(), aimrt::Pb2CompactJson(req), aimrt::Pb2CompactJson(rsp));

  } catch (const std::exception& e) {
    AIMRT_ERROR("Exit  with exception, {}", e.what());
  }
  co_return aimrt::rpc::Status();
}

co::Task<aimrt::rpc::Status> ExampleCoComplexCoServiceImpl::GetBarData(
    aimrt::rpc::ContextRef ctx,
    const ::aimrt::protocols::example::GetBarDataReq& req,
    ::aimrt::protocols::example::GetBarDataRsp& rsp) {
  rsp.set_msg("echo " + req.msg());

  AIMRT_INFO("Server handle new rpc call. context: {}, req: {}, return rsp: {}",
             ctx.ToString(), aimrt::Pb2CompactJson(req), aimrt::Pb2CompactJson(rsp));
  co_return aimrt::rpc::Status();
}

}  // namespace aimrt::examples::cpp::pb_rpc::proxy_rpc_co_module