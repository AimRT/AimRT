// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "rpc.aimrt_rpc.pb.h"

namespace aimrt::examples::cpp::pb_rpc::proxy_rpc_co_module {

using MsgHandleFunc = std::function<co::Task<void>()>;

class ExampleCoComplexCoServiceImpl : public aimrt::protocols::example::ExampleServiceCoService {
 public:
  ExampleCoComplexCoServiceImpl(const std::shared_ptr<aimrt::protocols::example::ExampleServiceCoProxy>& proxy,
                                aimrt::executor::ExecutorRef executor) : proxy_(proxy),
                                                                         executor_(executor) {}
  ~ExampleCoComplexCoServiceImpl() override = default;

  co::Task<aimrt::rpc::Status> GetFooData(
      aimrt::rpc::ContextRef ctx,
      const ::aimrt::protocols::example::GetFooDataReq& req,
      ::aimrt::protocols::example::GetFooDataRsp& rsp) override;

  co::Task<aimrt::rpc::Status> GetBarData(
      aimrt::rpc::ContextRef ctx,
      const ::aimrt::protocols::example::GetBarDataReq& req,
      ::aimrt::protocols::example::GetBarDataRsp& rsp) override;

 private:
  MsgHandleFunc handle_;
  std::shared_ptr<aimrt::protocols::example::ExampleServiceCoProxy> proxy_;
  aimrt::executor::ExecutorRef executor_;
};

}  // namespace aimrt::examples::cpp::pb_rpc::proxy_rpc_co_module