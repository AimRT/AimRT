// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <string>

#include "aimrt_module_cpp_interface/rpc/rpc_context.h"
#include "aimrt_module_cpp_interface/rpc/rpc_status.h"
#include "aimrt_module_cpp_interface/util/type_support.h"

namespace aimrt::runtime::core::rpc {

struct FuncInfo {
  std::string func_name;
  std::string pkg_path;
  std::string module_name;

  const void* custom_type_support_ptr;
  aimrt::util::TypeSupportRef req_type_support_ref;
  aimrt::util::TypeSupportRef rsp_type_support_ref;
};

struct InvokeWrapper {
  const FuncInfo& info;

  const void* req_ptr;
  void* rsp_ptr;

  aimrt::rpc::ContextRef ctx_ref;

  std::function<void(aimrt::rpc::Status)> callback;

  std::unordered_map<
      std::string,
      std::shared_ptr<aimrt::util::BufferArrayView>,
      aimrt::common::util::StringHash,
      std::equal_to<>>
      req_serialization_cache;
  std::unordered_map<
      std::string,
      std::shared_ptr<aimrt::util::BufferArrayView>,
      aimrt::common::util::StringHash,
      std::equal_to<>>
      rsp_serialization_cache;
};

}  // namespace aimrt::runtime::core::rpc