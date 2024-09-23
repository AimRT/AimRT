// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "parameter_plugin/service.h"
#include "aimrt_module_cpp_interface/parameter/parameter_handle.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "parameter_plugin/global.h"

namespace aimrt::plugins::parameter_plugin {

aimrt::co::Task<aimrt::rpc::Status> ParameterServiceImpl::Set(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::parameter_plugin::SetParameterReq& req,
    ::aimrt::protocols::parameter_plugin::SetParameterRsp& rsp) {
  auto* parameter_handle_ptr = parameter_manager_ptr_->GetParameterHandle(req.module_name());
  if (parameter_handle_ptr == nullptr) {
    SetErrorCode(ErrorCode::kInvalidModuleName, rsp);
    co_return aimrt::rpc::Status();
  }

  parameter_handle_ptr->SetParameter(req.parameter_key(), req.parameter_value());

  co_return aimrt::rpc::Status();
}

aimrt::co::Task<aimrt::rpc::Status> ParameterServiceImpl::Get(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::parameter_plugin::GetParameterReq& req,
    ::aimrt::protocols::parameter_plugin::GetParameterRsp& rsp) {
  auto* parameter_handle_ptr = parameter_manager_ptr_->GetParameterHandle(req.module_name());
  if (parameter_handle_ptr == nullptr) {
    SetErrorCode(ErrorCode::kInvalidModuleName, rsp);
    co_return aimrt::rpc::Status();
  }

  auto parameter_ptr = parameter_handle_ptr->GetParameter(req.parameter_key());
  if (parameter_ptr) {
    rsp.set_parameter_value(*parameter_ptr);
    co_return aimrt::rpc::Status();
  }

  co_return aimrt::rpc::Status();
}

aimrt::co::Task<aimrt::rpc::Status> ParameterServiceImpl::List(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::parameter_plugin::ListParameterReq& req,
    ::aimrt::protocols::parameter_plugin::ListParameterRsp& rsp) {
  auto* parameter_handle_ptr = parameter_manager_ptr_->GetParameterHandle(req.module_name());
  if (parameter_handle_ptr == nullptr) {
    SetErrorCode(ErrorCode::kInvalidModuleName, rsp);
    co_return aimrt::rpc::Status();
  }

  auto parameter_names = parameter_handle_ptr->ListParameter();
  rsp.mutable_parameter_keys()->Assign(parameter_names.begin(), parameter_names.end());

  co_return aimrt::rpc::Status();
}

aimrt::co::Task<aimrt::rpc::Status> ParameterServiceImpl::Dump(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::parameter_plugin::DumpParameterReq& req,
    ::aimrt::protocols::parameter_plugin::DumpParameterRsp& rsp) {
  for (auto itr = req.module_names().begin(); itr != req.module_names().end(); ++itr) {
    std::string_view module_name(*itr);
    auto* parameter_handle_ptr = parameter_manager_ptr_->GetParameterHandle(module_name);

    // 检查module name合法性
    if (parameter_handle_ptr == nullptr) {
      SetErrorCode(ErrorCode::kInvalidModuleName, rsp);
      co_return aimrt::rpc::Status();
    }

    auto& pb_parameter_map = (*rsp.mutable_module_parameter_map())[module_name];

    const auto& parameter_names = parameter_handle_ptr->ListParameter();

    for (const auto& parameter_name : parameter_names) {
      auto parameter_ptr = parameter_handle_ptr->GetParameter(parameter_name);
      if (!parameter_ptr) [[unlikely]] {
        continue;
      }

      pb_parameter_map.mutable_value()->emplace(parameter_name, *parameter_ptr);
    }
  }

  co_return aimrt::rpc::Status();
}

aimrt::co::Task<aimrt::rpc::Status> ParameterServiceImpl::Load(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::parameter_plugin::LoadParameterReq& req,
    ::aimrt::protocols::parameter_plugin::LoadParameterRsp& rsp) {
  for (auto module_itr = req.module_parameter_map().begin();
       module_itr != req.module_parameter_map().end();
       ++module_itr) {
    std::string_view module_name(module_itr->first);
    auto* parameter_handle_ptr = parameter_manager_ptr_->GetParameterHandle(module_name);

    // 检查module name合法性
    if (parameter_handle_ptr == nullptr) {
      SetErrorCode(ErrorCode::kInvalidModuleName, rsp);
      co_return aimrt::rpc::Status();
    }

    const auto& pb_parameter_map = module_itr->second.value();
    for (auto parameter_itr = pb_parameter_map.begin();
         parameter_itr != pb_parameter_map.end();
         ++parameter_itr) {
      parameter_handle_ptr->SetParameter(parameter_itr->first, parameter_itr->second);
    }
  }

  co_return aimrt::rpc::Status();
}

}  // namespace aimrt::plugins::parameter_plugin
