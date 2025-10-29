// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <functional>
#include <source_location>
#include <string>
#include <string_view>

#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/context/details/concepts.h"
#include "aimrt_module_cpp_interface/context/op_base.h"
#include "aimrt_module_cpp_interface/context/res/service.h"
#include "aimrt_module_cpp_interface/rpc/rpc_context.h"
#include "aimrt_module_cpp_interface/rpc/rpc_status.h"

namespace aimrt::context {

class OpCli : public OpBase {
 public:
  using OpBase::OpBase;

  OpCli(Context& ctx, std::source_location loc) noexcept : OpBase(ctx, loc) {}

  template <class Q, class P>
  [[nodiscard]] res::Service<Q, P> Init(std::string_view func_full_name);

  template <class Q, class P>
  aimrt::co::Task<aimrt::rpc::Status> Call(const res::Service<Q, P>& srv, const Q& q, P& p);

 private:
  template <class Q, class P>
  std::function<aimrt::co::Task<aimrt::rpc::Status>(aimrt::rpc::ContextRef, const Q&, P&)> CreateCallFunction(std::string_view func_full_name) const;
};

}  // namespace aimrt::context
