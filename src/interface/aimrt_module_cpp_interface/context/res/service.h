// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <optional>
#include <source_location>

#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/context/res/base.h"
#include "aimrt_module_cpp_interface/rpc/rpc_context.h"
#include "aimrt_module_cpp_interface/rpc/rpc_status.h"
#include "aimrt_module_cpp_interface/executor/executor.h"

namespace aimrt::context::res {

template <class Q, class P>
class Service : public details::Base {
 public:
  using RequestType = Q;
  using ResponseType = P;
  using details::Base::Base;
};

template <class Q, class P>
class Client : public res::Service<Q, P> {
 public:
  using ClientRequest = Q;
  using ClientResponse = P;

  [[nodiscard]] co::Task<aimrt::rpc::Status> Call(
      const Q& q, P& p, std::source_location loc = std::source_location::current()) const;

  [[nodiscard]] co::Task<aimrt::rpc::Status> operator()(
      const Q& q, P& p, std::source_location loc = std::source_location::current()) const {
    return Call(q, p, loc);
  }

  [[nodiscard]] co::Task<std::optional<P>> Call(
      const Q& q, std::source_location loc = std::source_location::current()) const {
    P p;
    if (const aimrt::rpc::Status status = co_await Call(q, p, loc); status.OK())
      co_return p;

    co_return {};
  }

  [[nodiscard]] co::Task<std::optional<P>> operator()(
      const Q& q, std::source_location loc = std::source_location::current()) const {
    return Call(q, loc);
  }

  [[nodiscard]] co::Task<aimrt::rpc::Status> Call(
      aimrt::rpc::ContextRef ctx, const Q& q, P& p,
      std::source_location loc = std::source_location::current()) const;

  [[nodiscard]] co::Task<aimrt::rpc::Status> operator()(
      aimrt::rpc::ContextRef ctx, const Q& q, P& p,
      std::source_location loc = std::source_location::current()) const {
    return Call(std::move(ctx), q, p, loc);
  }

  [[nodiscard]] co::Task<std::optional<P>> Call(
      aimrt::rpc::ContextRef ctx, const Q& q,
      std::source_location loc = std::source_location::current()) const {
    P p;
    if (const aimrt::rpc::Status status = co_await Call(std::move(ctx), q, p, loc); status.OK())
      co_return p;

    co_return {};
  }

  [[nodiscard]] co::Task<std::optional<P>> operator()(
      aimrt::rpc::ContextRef ctx, const Q& q,
      std::source_location loc = std::source_location::current()) const {
    return Call(std::move(ctx), q, loc);
  }

 public:
  Client() = default;
  using res::Service<Q, P>::Service;

  Client(res::Service<Q, P>&& res)
      : res::Service<Q, P>(std::move(res)) {}

  Client(const res::Service<Q, P>& res)
      : res::Service<Q, P>(res) {}
};


template <class Q, class P>
class Server : public res::Service<Q, P> {
 public:
  using ServerRequest = Q;
  using ServerResponse = P;

  template <typename TServer>
  void ServeInline(TServer server, std::source_location loc = std::source_location::current()) const;

  template <typename TServer>
  void ServeOn(const aimrt::executor::ExecutorRef& exe, TServer server,
               std::source_location loc = std::source_location::current()) const;

 public:
  Server() = default;
  using res::Service<Q, P>::Service;

  Server(res::Service<Q, P>&& res)
      : res::Service<Q, P>(std::move(res)) {}

  Server(const res::Service<Q, P>& res)
      : res::Service<Q, P>(res) {}
};

}  // namespace aimrt::context::res
