// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "rpc.aimrt_rpc.pb.h"

namespace aimrt::examples::cpp::pb_rpc::normal_rpc_async_server_module {

class ExampleServiceAsyncServiceImpl : public aimrt::protocols::example::ExampleServiceAsyncService {
 public:
  ExampleServiceAsyncServiceImpl() = default;
  ~ExampleServiceAsyncServiceImpl() override = default;

  void GetFooData(
      aimrt::rpc::ContextRef ctx,
      const ::aimrt::protocols::example::GetFooDataReq& req,
      ::aimrt::protocols::example::GetFooDataRsp& rsp,
      std::function<void(aimrt::rpc::Status)>&& callback) override;

  void GetBarData(
      aimrt::rpc::ContextRef ctx,
      const ::aimrt::protocols::example::GetBarDataReq& req,
      ::aimrt::protocols::example::GetBarDataRsp& rsp,
      std::function<void(aimrt::rpc::Status)>&& callback) override;
};

}  // namespace aimrt::examples::cpp::pb_rpc::normal_rpc_async_server_module
