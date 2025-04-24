// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "aimrt_module_cpp_interface/co/sync_wait.h"
#include "aimrt_module_cpp_interface/rpc/rpc_co_filter.h"

namespace aimrt::rpc {

TEST(RPC_FILTER_TEST, CoFilterManager_base) {
  CoFilterManager m;

  auto rpc_handle = [](ContextRef ctx, const void *req, void *rsp) -> co::Task<Status> {
    *static_cast<std::string *>(rsp) = *static_cast<const std::string *>(req);
    co_return Status();
  };

  std::string req = "test req";
  std::string rsp;
  Context ctx;

  auto status_op = co::SyncWait(m.InvokeRpc(rpc_handle, ctx, &req, &rsp));

  EXPECT_TRUE(status_op);
  EXPECT_TRUE(*status_op);

  EXPECT_EQ(rsp, req);
}

TEST(RPC_FILTER_TEST, CoFilterManager_multiple_filters) {
  CoFilterManager m;

  // Filters registered first are in the inner layer
  m.RegisterFilter([](ContextRef ctx, const void *req, void *rsp, const CoRpcHandle &h) -> co::Task<Status> {
    ctx.SetMetaValue("order", std::string(ctx.GetMetaValue("order")) + " -> f1 begin");

    auto status = co_await h(ctx, req, rsp);

    ctx.SetMetaValue("order", std::string(ctx.GetMetaValue("order")) + " -> f1 end");

    co_return status;
  });

  // Filters registered later are in the outer layer
  m.RegisterFilter([](ContextRef ctx, const void *req, void *rsp, const CoRpcHandle &h) -> co::Task<Status> {
    ctx.SetMetaValue("order", std::string(ctx.GetMetaValue("order")) + " -> f2 begin");

    auto status = co_await h(ctx, req, rsp);

    ctx.SetMetaValue("order", std::string(ctx.GetMetaValue("order")) + " -> f2 end");

    co_return status;
  });

  // The rpc handle is in the innermost layer
  auto rpc_handle = [](ContextRef ctx, const void *req, void *rsp) -> co::Task<Status> {
    ctx.SetMetaValue("order", std::string(ctx.GetMetaValue("order")) + " -> rpc handle");
    co_return Status();
  };

  std::string req;
  std::string rsp;
  Context ctx;

  ctx.SetMetaValue("order", "begin");
  auto status_op = co::SyncWait(m.InvokeRpc(rpc_handle, ctx, &req, &rsp));
  ctx.SetMetaValue("order", std::string(ctx.GetMetaValue("order")) + " -> end");

  EXPECT_TRUE(status_op);
  EXPECT_TRUE(*status_op);

  EXPECT_STREQ(
      ctx.GetMetaValue("order").data(),
      "begin -> f2 begin -> f1 begin -> rpc handle -> f1 end -> f2 end -> end");
}

}  // namespace aimrt::rpc
