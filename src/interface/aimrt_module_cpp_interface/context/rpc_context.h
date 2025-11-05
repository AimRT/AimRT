// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <any>
#include "aimrt_module_cpp_interface/rpc/rpc_context.h"
#include "aimrt_module_cpp_interface/util/function.h"
#include "rpc/rpc_handle_base.h"

namespace aimrt::context {

struct RpcResource {
  // 擦除了类型的客户端调用接口（Client<Q, P>）
  std::any call_f;

  // 擦除了类型的服务端处理接口（Server<Q, P>）
  std::any serve_f;

  // 服务方法名称，作为 aimrt 内部 std::string_view 的内容 holder
  std::string func_name;
};

}  // namespace aimrt::context
