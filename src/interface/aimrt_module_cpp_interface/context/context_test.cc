// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include <memory>
#include <source_location>
#include <string>

#include <aimrt_module_c_interface/executor/executor_base.h>
#include <aimrt_module_c_interface/executor/executor_manager_base.h>
#include <aimrt_module_cpp_interface/context/context.h>
#include <aimrt_module_cpp_interface/context/op_pub.h>
#include <aimrt_module_cpp_interface/context/op_sub.h>
#include <aimrt_module_cpp_interface/context/res/executor.h>
#include <aimrt_module_cpp_interface/executor/executor.h>
#include <aimrt_module_cpp_interface/executor/executor_manager.h>

namespace aimrt::context {

namespace {

struct FakeExecutorEnvironment {
  aimrt_executor_base_t executor_base{};
  aimrt_executor_manager_base_t executor_manager{};
  aimrt_core_base_t core_base{};

  bool executed = false;
  std::string last_request_name;

  FakeExecutorEnvironment() {
    executor_base.impl = this;
    executor_base.type = +[](void*) -> aimrt_string_view_t {
      static constexpr char kType[] = "fake";
      return {kType, sizeof(kType) - 1};
    };
    executor_base.name = +[](void*) -> aimrt_string_view_t {
      static constexpr char kName[] = "fake";
      return {kName, sizeof(kName) - 1};
    };
    executor_base.is_thread_safe = +[](void*) { return true; };
    executor_base.is_in_current_executor = +[](void*) { return false; };
    executor_base.is_support_timer_schedule = +[](void*) { return false; };
    executor_base.execute = +[](void* impl, aimrt_function_base_t* task) {
      auto* self = static_cast<FakeExecutorEnvironment*>(impl);
      self->executed = true;
      const auto* ops = static_cast<const aimrt_function_executor_task_ops_t*>(task->ops);
      ops->invoker(static_cast<void*>(task->object_buf));
    };
    executor_base.now = +[](void*) { return static_cast<uint64_t>(0); };
    executor_base.execute_at_ns = +[](void* impl, uint64_t, aimrt_function_base_t* task) {
      FakeExecutorEnvironment::ExecutorExecute(impl, task);
    };

    executor_manager.impl = this;
    executor_manager.get_executor = +[](void* impl, aimrt_string_view_t executor_name) -> const aimrt_executor_base_t* {
      auto* self = static_cast<FakeExecutorEnvironment*>(impl);
      self->last_request_name.assign(executor_name.str, executor_name.len);
      return &self->executor_base;
    };

    core_base.impl = this;
    core_base.executor_manager = +[](void* impl) -> const aimrt_executor_manager_base_t* {
      auto* self = static_cast<FakeExecutorEnvironment*>(impl);
      return &self->executor_manager;
    };
    core_base.info = nullptr;
    core_base.configurator = nullptr;
    core_base.logger = nullptr;
    core_base.allocator_handle = nullptr;
    core_base.rpc_handle = nullptr;
    core_base.channel_handle = nullptr;
    core_base.parameter_handle = nullptr;
  }

  static void ExecutorExecute(void* impl, aimrt_function_base_t* task) {
    auto* self = static_cast<FakeExecutorEnvironment*>(impl);
    self->executed = true;
    const auto* ops = static_cast<const aimrt_function_executor_task_ops_t*>(task->ops);
    ops->invoker(static_cast<void*>(task->object_buf));
  }
};

}  // namespace

class TestOpBase : public OpBase {
 public:
  using OpBase::OpBase;

  Context& ContextRef() { return ctx_; }
  std::source_location Location() const { return loc_; }
};

// OpExe removed; tests adapted to use Context + ExecutorRef

TEST(ContextTest, CoreAccessors) {
  aimrt_core_base_t fake_core{};
  aimrt::CoreRef core_ref(&fake_core);
  Context ctx(core_ref);
  EXPECT_EQ(core_ref.NativeHandle(), ctx.Core().NativeHandle());
  EXPECT_EQ(core_ref.NativeHandle(), Context::GetRawRef(ctx).NativeHandle());
}

TEST(ContextTest, AsyncScopeHelpers) {
  Context ctx;
  auto& scope = ctx.AsyncScope();
  auto& scope_alias = Context::GetAsyncScope(ctx);
  EXPECT_EQ(&scope, &scope_alias);
}

TEST(ContextTest, ShutdownFlag) {
  Context ctx;
  EXPECT_TRUE(ctx.Ok());
  ctx.RequireToShutdown();
  EXPECT_FALSE(ctx.Ok());
}

TEST(ContextTest, ExecutorLookup) {
  FakeExecutorEnvironment env;
  aimrt::CoreRef core_ref(&env.core_base);

  auto ctx = std::make_shared<Context>(core_ref);
  const auto loc = std::source_location::current();
  auto ref = core_ref.GetExecutorManager().GetExecutor("fake");
  EXPECT_TRUE(static_cast<bool>(ref));
}

TEST(ContextTest, ExecutorExecuteWorks) {
  FakeExecutorEnvironment env;
  aimrt::CoreRef core_ref(&env.core_base);

  auto ctx = std::make_shared<Context>(core_ref);
  auto ref = core_ref.GetExecutorManager().GetExecutor("fake");
  EXPECT_TRUE(ref);
  int counter = 0;
  ref.Dispatch(aimrt::executor::Task([&counter]() { ++counter; }));
  EXPECT_EQ(1, counter);
}

}  // namespace aimrt::context
