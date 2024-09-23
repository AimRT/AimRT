// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "rpc.aimrt_rpc.pb.h"

namespace aimrt::examples::cpp::protobuf_rpc::normal_rpc_sync_server_module {

class ExampleServiceSyncServiceImpl : public aimrt::protocols::example::ExampleServiceSyncService {
 public:
  ExampleServiceSyncServiceImpl() = default;
  ~ExampleServiceSyncServiceImpl() override = default;

  aimrt::rpc::Status GetFooData(
      aimrt::rpc::ContextRef ctx,
      const ::aimrt::protocols::example::GetFooDataReq& req,
      ::aimrt::protocols::example::GetFooDataRsp& rsp) override;

  aimrt::rpc::Status GetBarData(
      aimrt::rpc::ContextRef ctx,
      const ::aimrt::protocols::example::GetBarDataReq& req,
      ::aimrt::protocols::example::GetBarDataRsp& rsp) override;
};

}  // namespace aimrt::examples::cpp::protobuf_rpc::normal_rpc_sync_server_module
