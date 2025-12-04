// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <any>
#include "aimrt_module_cpp_interface/rpc/rpc_context.h"
#include "aimrt_module_cpp_interface/util/function.h"
#include "rpc/rpc_handle_base.h"

namespace aimrt::context {

enum class RpcState : uint32_t {
  kOn = 0,
  kOff,
};

struct RpcResource {
  std::any call_f;
  std::any serve_f;
  std::string func_name;
};

}  // namespace aimrt::context
