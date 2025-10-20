# Context


## 相关链接

### 核心接口

代码文件：
- {{ '[aimrt_module_cpp_interface/context/context.h]({}/src/interface/aimrt_module_cpp_interface/context/context.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/op_pub.h]({}/src/interface/aimrt_module_cpp_interface/context/op_pub.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/op_sub.h]({}/src/interface/aimrt_module_cpp_interface/context/op_sub.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/op_cli.h]({}/src/interface/aimrt_module_cpp_interface/context/op_cli.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/op_srv.h]({}/src/interface/aimrt_module_cpp_interface/context/op_srv.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/details/thread_context.h]({}/src/interface/aimrt_module_cpp_interface/context/details/thread_context.h)'.format(code_site_root_path_url) }}

关联能力：
- {{ '[executor 概念与用法]({}/document/sphinx-cn/tutorials/interface_cpp/executor.md)'.format(doc_site_root_path_url) }}
- {{ '[channel 概念与用法]({}/document/sphinx-cn/tutorials/interface_cpp/channel.md)'.format(doc_site_root_path_url) }}
- {{ '[rpc 概念与用法]({}/document/sphinx-cn/tutorials/interface_cpp/rpc.md)'.format(doc_site_root_path_url) }}


## 概念概述

`aimrt::context::Context` 用于在模块内承载一次业务上下文，统一管理发布/订阅、RPC 客户端/服务端以及执行器访问等操作。它还支持将上下文绑定到当前线程（Thread-Local），使通道与服务资源的使用语法更自然。

- **核心职责**：
  - 提供 `pub()/sub()/cli()/srv()` 四类操作器以初始化资源并执行发布、订阅、调用与服务。
  - 提供 `GetExecutor(name)` 以按名称获取执行器句柄。
  - 提供生命周期与线程上下文能力：`LetMe()`、`Ok()`、`RequireToShutdown()`。


## 基本用法总览

典型流程：
1) 构造 `Context` 并保存 `CoreRef`；
2) 使用 `pub()/sub()/cli()/srv()` 初始化资源句柄；
3) 在 `Start()` 中开始订阅、服务或调度任务；
4) 在 `Shutdown()` 中清理或请求停止。

示意：
```cpp
#include "aimrt_module_cpp_interface/module_base.h"
#include "aimrt_module_cpp_interface/context/context.h"

class DemoModule : public aimrt::ModuleBase {
 public:
  bool Initialize(aimrt::CoreRef core) override {
    ctx_ = std::make_shared<aimrt::context::Context>(core);
    // 绑定到当前线程，使后续资源调用可通过线程上下文获得 ctx
    ctx_->LetMe();
    return true;
  }

  bool Start() override {
    // 例如：获取执行器
    auto exe = ctx_->GetExecutor("work_executor");
    AIMRT_CHECK_ERROR_THROW(exe, "work_executor not found");
    exe.Execute([](){ AIMRT_INFO("Run in work_executor"); });
    return true;
  }

  void Shutdown() override {
    ctx_->RequireToShutdown();
  }

 private:
  std::shared_ptr<aimrt::context::Context> ctx_;
};
```


## 发布/订阅（Channel）

### 初始化与发布
```cpp
// 初始化发布端资源（类型需为直接支持的消息类型）
auto ch_pub = ctx_->pub().Init<MyMsg>("demo/topic");

// 直接发布（内部会创建临时 Channel Context）
ch_pub.Publish(MyMsg{/*...*/});

// 携带通道上下文发布（更细粒度的元信息控制）
aimrt::channel::Context ch_ctx;
ch_pub.Publish(ch_ctx, MyMsg{/*...*/});
```

### 初始化与订阅
```cpp
// 初始化订阅端资源
auto ch_sub = ctx_->sub().Init<MyMsg>("demo/topic");

// 1) 同线程回调（Inline）
ctx_->sub().SubscribeInline(ch_sub, [](std::shared_ptr<const MyMsg> msg){
  AIMRT_INFO("Recv msg");
});

// 2) 在指定执行器上回调（On Executor）
auto exe = ctx_->GetExecutor("callback_executor");
ctx_->sub().SubscribeOn(ch_sub, exe, [](aimrt::channel::ContextRef, const MyMsg& msg){
  AIMRT_INFO("Recv on executor");
});
```

要点：
- 消息类型需满足“直接支持类型”的约束（详见 `details/concepts.h` 与类型支持注册）。
- `SubscribeInline` 在当前线程执行回调；`SubscribeOn` 会将回调投递到指定执行器。


## RPC 调用与服务

### 客户端调用（Client）
```cpp
// 初始化服务资源（Q 请求，P 响应）
auto srv = ctx_->cli().Init<Req, Rsp>("package.Service/Func");

// 调用（基于协程）
aimrt::co::SyncWait(ctx_->cli().Call(srv, Req{/*...*/}, rsp_));
AIMRT_INFO("status={}", static_cast<int>(rsp_status.code()));
```

### 服务端处理（Server）
```cpp
// 初始化服务资源
auto srv = ctx_->srv().Init<Req, Rsp>("package.Service/Func");

// 注册处理器（Inline 执行）
ctx_->srv().ServeInline(srv, [](aimrt::rpc::ContextRef, const Req& q, Rsp& p) -> aimrt::co::Task<aimrt::rpc::Status> {
  // 填充响应
  co_return aimrt::rpc::Status::OK();
});
```

要点：
- 函数全名通常形如 `pkg.Service/Func`，需与部署和 IDL 保持一致。
- 客户端 `Call` 与服务端 `ServeInline` 都基于协程 `Task` 进行异步式流程表达，可与执行器配合调度。


## 线程上下文（Thread Context）

`thread_context.h` 提供线程局部的上下文对象，使资源句柄可通过线程环境获取：
- `Context::LetMe()`：将当前 `Context` 绑定到线程局部存储；
- 内部 `details::ExpectContext()` 在需要时从 TLS 中取回 `Context`；
- 一些便捷方法（如 `res::Channel<T>::Publish`）会隐式获取当前上下文并转发到 `OpPub`。

注意：
- 若线程未绑定 `Context` 而直接使用依赖线程上下文的 API，会触发断言。
- 建议在模块的工作线程进入主循环前调用一次 `LetMe()`，或在需要的执行器任务开始处调用。


## 执行器获取与调度

`Context::GetExecutor(name)` 通过内部持有的 `CoreRef` 查询执行器：
- 返回 `aimrt::executor::ExecutorRef`；
- 可检查 `ThreadSafe()`、`SupportTimerSchedule()` 等属性；
- 可使用 `Execute/ExecuteAt/ExecuteAfter` 投递任务或定时调度（具体见执行器文档）。

示例：
```cpp
auto exe = ctx_->GetExecutor("time_executor");
AIMRT_CHECK_ERROR_THROW(exe && exe.SupportTimerSchedule(), "executor not support timer");
exe.ExecuteAfter(std::chrono::seconds(1), [](){ AIMRT_INFO("tick"); });
```


## 生命周期与停止协商

- `Ok()`：查询当前上下文是否仍处于工作状态；
- `RequireToShutdown()`：请求上下文进入停止流程，配合 `Ok()` 可优雅退出循环：
```cpp
while (ctx_->Ok()) {
  // ... do work
}
```


## 最佳实践与注意事项

- 在进入需要使用上下文的线程或执行器回调时，先调用 `LetMe()` 绑定上下文。
- 订阅回调若有耗时操作，优先使用 `SubscribeOn` 切换到专用执行器，避免阻塞 IO 线程。
- RPC 处理函数建议无阻塞、可中断，必要时结合执行器进行调度与超时控制。
- 类型需满足“直接支持类型”或已通过类型支持注册后再用于 Channel/RPC。
- 获取执行器失败或能力不匹配时，务必进行检查并给出明确日志/错误处理。
