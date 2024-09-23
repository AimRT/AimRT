// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "rpc.aimrt_rpc.pb.h"

namespace aimrt::examples::cpp::protobuf_rpc::normal_rpc_co_server_module {

class ExampleServiceImpl : public aimrt::protocols::example::ExampleServiceCoService {
 public:
  ExampleServiceImpl() = default;
  ~ExampleServiceImpl() override = default;

  co::Task<aimrt::rpc::Status> GetFooData(
      aimrt::rpc::ContextRef ctx,
      const ::aimrt::protocols::example::GetFooDataReq& req,
      ::aimrt::protocols::example::GetFooDataRsp& rsp) override;

  co::Task<aimrt::rpc::Status> GetBarData(
      aimrt::rpc::ContextRef ctx,
      const ::aimrt::protocols::example::GetBarDataReq& req,
      ::aimrt::protocols::example::GetBarDataRsp& rsp) override;
};

}  // namespace aimrt::examples::cpp::protobuf_rpc::normal_rpc_co_server_module
