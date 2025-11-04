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

  // 擦除了类型的原生服务端处理接口，用于 Serving() 函数与 DoInitServer() 函数（ServerInvoker<Q, P>）
  std::any server_invoke_f;

  // AimRT 服务回调函数的容器，仅通过本类的 InitServerFunc() 会初始化它
  aimrt::util::Function<aimrt_function_service_func_ops_t> raw_server_holder;

  // 服务方法名称，作为 aimrt 内部 std::string_view 的内容 holder
  std::string func_name;
};

}  // namespace aimrt::context
