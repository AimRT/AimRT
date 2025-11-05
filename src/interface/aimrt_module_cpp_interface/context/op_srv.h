// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <source_location>
#include <string_view>

#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/context/details/concepts.h"
#include "aimrt_module_cpp_interface/context/op_base.h"
#include "aimrt_module_cpp_interface/context/res/service.h"
#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_cpp_interface/rpc/rpc_status.h"

namespace aimrt::context {

class OpSrv : public OpBase {
 public:
  using OpBase::OpBase;

  OpSrv(Context& ctx, std::source_location loc) noexcept : OpBase(ctx, loc) {}

  template <class Q, class P>
  [[nodiscard]] res::Service<Q, P> Init(std::string_view func_full_name);

  template <class Q, class P, concepts::SupportedServer<Q, P> TServer>
  void ServeInline(const res::Service<Q, P>& srv, TServer server);

  template <class Q, class P, concepts::SupportedServer<Q, P> TServer>
  void ServeOn(const res::Service<Q, P>& srv, aimrt::executor::ExecutorRef exe, TServer server);

  template <class Q, class P>
  aimrt::co::Task<aimrt::rpc::Status> Serving(const res::Service<Q, P>& srv, aimrt::rpc::ContextRef ctx, const Q& q, P& p);

 private:
  template <class Q, class P, concepts::SupportedServer<Q, P> Func>
  constexpr auto StandardizeServer(Func&& cb);
};

}  // namespace aimrt::context
