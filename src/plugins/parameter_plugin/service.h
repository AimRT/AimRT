// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "core/parameter/parameter_manager.h"

#include "parameter.aimrt_rpc.pb.h"

namespace aimrt::plugins::parameter_plugin {

class ParameterServiceImpl : public aimrt::protocols::parameter_plugin::ParameterServiceCoService {
 public:
  ParameterServiceImpl() = default;
  ~ParameterServiceImpl() override = default;

  void SetParameterManager(aimrt::runtime::core::parameter::ParameterManager* ptr) {
    parameter_manager_ptr_ = ptr;
  }

  aimrt::co::Task<aimrt::rpc::Status> Set(
      aimrt::rpc::ContextRef ctx_ref,
      const ::aimrt::protocols::parameter_plugin::SetParameterReq& req,
      ::aimrt::protocols::parameter_plugin::SetParameterRsp& rsp) override;

  aimrt::co::Task<aimrt::rpc::Status> Get(
      aimrt::rpc::ContextRef ctx_ref,
      const ::aimrt::protocols::parameter_plugin::GetParameterReq& req,
      ::aimrt::protocols::parameter_plugin::GetParameterRsp& rsp) override;

  aimrt::co::Task<aimrt::rpc::Status> List(
      aimrt::rpc::ContextRef ctx_ref,
      const ::aimrt::protocols::parameter_plugin::ListParameterReq& req,
      ::aimrt::protocols::parameter_plugin::ListParameterRsp& rsp) override;

  aimrt::co::Task<aimrt::rpc::Status> Dump(
      aimrt::rpc::ContextRef ctx_ref,
      const ::aimrt::protocols::parameter_plugin::DumpParameterReq& req,
      ::aimrt::protocols::parameter_plugin::DumpParameterRsp& rsp) override;

  aimrt::co::Task<aimrt::rpc::Status> Load(
      aimrt::rpc::ContextRef ctx_ref,
      const ::aimrt::protocols::parameter_plugin::LoadParameterReq& req,
      ::aimrt::protocols::parameter_plugin::LoadParameterRsp& rsp) override;

 private:
  enum class ErrorCode : uint32_t {
    kSuc = 0,
    kInvalidModuleName = 1,
  };

  static constexpr std::string_view kErrorInfoArray[] = {
      "",
      "INVALID_MODULE_NAME"};

  template <typename T>
  void SetErrorCode(ErrorCode code, T& rsp) {
    rsp.set_code(static_cast<uint32_t>(code));
    rsp.set_msg(std::string(kErrorInfoArray[static_cast<uint32_t>(code)]));
  }

  aimrt::runtime::core::parameter::ParameterManager* parameter_manager_ptr_ = nullptr;
};

}  // namespace aimrt::plugins::parameter_plugin
