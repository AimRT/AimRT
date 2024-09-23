// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "normal_rpc_sync_server_module/service.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "normal_rpc_sync_server_module/global.h"

namespace aimrt::examples::cpp::protobuf_rpc::normal_rpc_sync_server_module {

aimrt::rpc::Status ExampleServiceSyncServiceImpl::GetFooData(
    aimrt::rpc::ContextRef ctx,
    const ::aimrt::protocols::example::GetFooDataReq& req,
    ::aimrt::protocols::example::GetFooDataRsp& rsp) {
  rsp.set_msg("echo " + req.msg());

  AIMRT_INFO("Server handle new rpc call. context: {}, req: {}, return rsp: {}",
             ctx.ToString(), aimrt::Pb2CompactJson(req), aimrt::Pb2CompactJson(rsp));

  return aimrt::rpc::Status();
}

aimrt::rpc::Status ExampleServiceSyncServiceImpl::GetBarData(
    aimrt::rpc::ContextRef ctx,
    const ::aimrt::protocols::example::GetBarDataReq& req,
    ::aimrt::protocols::example::GetBarDataRsp& rsp) {
  rsp.set_msg("echo " + req.msg());

  AIMRT_INFO("Server handle new rpc call. context: {}, req: {}, return rsp: {}",
             ctx.ToString(), aimrt::Pb2CompactJson(req), aimrt::Pb2CompactJson(rsp));

  return aimrt::rpc::Status();
}

}  // namespace aimrt::examples::cpp::protobuf_rpc::normal_rpc_sync_server_module
