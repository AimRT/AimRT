// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include <future>

#include "core/rpc/rpc_framework_async_filter.h"

namespace aimrt::runtime::core::rpc {

TEST(RPC_FRAMEWORK_ASYNC_FILTER_TEST, FrameworkAsyncRpcFilterCollector_base) {
  FrameworkAsyncRpcFilterCollector filter_collector;

  auto rpc_handle = [](const std::shared_ptr<InvokeWrapper> &invoke_wrapper_ptr) {
    *static_cast<std::string *>(invoke_wrapper_ptr->rsp_ptr) =
        *static_cast<const std::string *>(invoke_wrapper_ptr->req_ptr);
    invoke_wrapper_ptr->callback(aimrt::rpc::Status());
  };

  FuncInfo info;

  std::string req = "test req";
  std::string rsp;

  auto invoke_wrapper_ptr = std::make_shared<InvokeWrapper>(
      InvokeWrapper{.info = info, .req_ptr = &req, .rsp_ptr = &rsp});

  aimrt::rpc::Context ctx;
  invoke_wrapper_ptr->ctx_ref = ctx;

  std::promise<aimrt::rpc::Status> status_promise;
  invoke_wrapper_ptr->callback =
      [&status_promise](aimrt::rpc::Status status) mutable {
        status_promise.set_value(status);
      };

  filter_collector.InvokeRpc(std::move(rpc_handle), invoke_wrapper_ptr);

  auto status = status_promise.get_future().get();
  EXPECT_TRUE(status);

  EXPECT_EQ(rsp, req);
}

TEST(RPC_FRAMEWORK_ASYNC_FILTER_TEST, FrameworkAsyncRpcFilterCollector_multiple_filters) {
  FrameworkAsyncRpcFilterCollector filter_collector;

  // 先注册的在内层
  FrameworkAsyncRpcFilter filter_1 =
      [](const std::shared_ptr<InvokeWrapper> &invoke_wrapper_ptr, FrameworkAsyncRpcHandle &&h) {
        std::thread([invoke_wrapper_ptr, h{std::move(h)}]() {
          auto ctx_ref = invoke_wrapper_ptr->ctx_ref;
          ctx_ref.SetMetaValue("order", std::string(ctx_ref.GetMetaValue("order")) + " -> f1 begin");

          invoke_wrapper_ptr->callback =
              [ctx_ref, callback{std::move(invoke_wrapper_ptr->callback)}](aimrt::rpc::Status status) mutable {
                ctx_ref.SetMetaValue("order", std::string(ctx_ref.GetMetaValue("order")) + " -> f1 end");

                callback(status);
              };

          h(invoke_wrapper_ptr);
        }).detach();
      };

  filter_collector.RegisterFilter(filter_1);

  // 后注册的在外层
  FrameworkAsyncRpcFilter filter_2 =
      [](const std::shared_ptr<InvokeWrapper> &invoke_wrapper_ptr, FrameworkAsyncRpcHandle &&h) {
        std::thread([invoke_wrapper_ptr, h{std::move(h)}]() {
          auto ctx_ref = invoke_wrapper_ptr->ctx_ref;
          ctx_ref.SetMetaValue("order", std::string(ctx_ref.GetMetaValue("order")) + " -> f2 begin");

          invoke_wrapper_ptr->callback =
              [ctx_ref, callback{std::move(invoke_wrapper_ptr->callback)}](aimrt::rpc::Status status) mutable {
                ctx_ref.SetMetaValue("order", std::string(ctx_ref.GetMetaValue("order")) + " -> f2 end");

                callback(status);
              };

          h(invoke_wrapper_ptr);
        }).detach();
      };
  filter_collector.RegisterFilter(filter_2);

  // rpc handle在最内层
  auto rpc_handle = [](const std::shared_ptr<InvokeWrapper> &invoke_wrapper_ptr) {
    auto ctx_ref = invoke_wrapper_ptr->ctx_ref;
    ctx_ref.SetMetaValue("order", std::string(ctx_ref.GetMetaValue("order")) + " -> rpc handle");
    invoke_wrapper_ptr->callback(aimrt::rpc::Status());
  };

  FuncInfo info;

  std::string req;
  std::string rsp;

  auto invoke_wrapper_ptr = std::make_shared<InvokeWrapper>(
      InvokeWrapper{.info = info, .req_ptr = &req, .rsp_ptr = &rsp});

  aimrt::rpc::Context ctx;
  invoke_wrapper_ptr->ctx_ref = ctx;

  std::promise<aimrt::rpc::Status> status_promise;
  invoke_wrapper_ptr->callback =
      [&ctx, &status_promise](aimrt::rpc::Status status) mutable {
        ctx.SetMetaValue("order", std::string(ctx.GetMetaValue("order")) + " -> end");

        status_promise.set_value(status);
      };

  ctx.SetMetaValue("order", "begin");

  filter_collector.InvokeRpc(std::move(rpc_handle), invoke_wrapper_ptr);

  auto status = status_promise.get_future().get();
  EXPECT_TRUE(status);

  EXPECT_STREQ(
      ctx.GetMetaValue("order").data(),
      "begin -> f2 begin -> f1 begin -> rpc handle -> f1 end -> f2 end -> end");
}

}  // namespace aimrt::runtime::core::rpc