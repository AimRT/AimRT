# Context

## 相关链接

代码文件：
- {{ '[aimrt_module_cpp_interface/context/context.h]({}/src/interface/aimrt_module_cpp_interface/context/context.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/op_pub.h]({}/src/interface/aimrt_module_cpp_interface/context/op_pub.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/op_sub.h]({}/src/interface/aimrt_module_cpp_interface/context/op_sub.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/op_cli.h]({}/src/interface/aimrt_module_cpp_interface/context/op_cli.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/op_srv.h]({}/src/interface/aimrt_module_cpp_interface/context/op_srv.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/init.h]({}/src/interface/aimrt_module_cpp_interface/context/init.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/anytime.h]({}/src/interface/aimrt_module_cpp_interface/context/anytime.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/res/channel.h]({}/src/interface/aimrt_module_cpp_interface/context/res/channel.h)'.format(code_site_root_path_url) }}

参考示例：
- {{ '[context]({}/src/examples/cpp/context)'.format(code_site_root_path_url) }}
  - {{ '[channel_publisher_module.cc]({}/src/examples/cpp/context/module/channel_publisher_module/channel_publisher_module.cc)'.format(code_site_root_path_url) }}
  - {{ '[channel_subscriber_module.cc]({}/src/examples/cpp/context/module/channel_subscriber_module/channel_subscriber_module.cc)'.format(code_site_root_path_url) }}


## 概念

Context 是 AimRT 在 C++ 接口中提供的“运行期上下文”，贯穿模块 Initialize/Start/Shutdown 的调用路径，用于：
- **线程内上下文绑定**：通过 `Context::LetMe()` 把当前线程与上下文关联，后续可用 `aimrt::context::Ok()` 等便捷函数访问。
- **访问核心句柄**：通过 `Context::GetRawRef()`（`aimrt::CoreRef`）间接获取日志、执行器、配置器、Channel/RPC/参数等句柄。
- **编排操作**：通过 `ctx.pub()/sub()/cli()/srv()` 获取操作器对象，统一完成 Channel 发布/订阅与 RPC 客户端/服务端编排。
- **生命周期与中断**：`RequireToShutdown()` 和 `Ok()` 用于跨任务/线程表达退出意图。

在 AimRT 中，模块继承 `aimrt::ModuleBase`，框架会在初始化阶段为模块创建 `Context` 并注入，模块可通过 `GetContext()` 获取共享指针，并在任务线程中调用 `LetMe()` 绑定当前执行环境。


## 核心类型与接口

- `aimrt::context::Context`
  - `GetRawRef()`/`GetLogger()`：访问底层 `CoreRef` 与 Logger。
  - `LetMe()`：将当前线程的 thread-local 上下文指向本 `Context`。
  - `RequireToShutdown()`/`Ok()`：发出停止请求并在循环内检测。
  - `GetExecutor(name)`：按名称获取执行器句柄。
  - `pub()/sub()/cli()/srv()`：分别返回 `OpPub`/`OpSub`/`OpCli`/`OpSrv` 操作器。

- `aimrt::context::init` 便捷初始化函数
  - `Publisher<T>(topic)`：注册发布类型并返回 `res::Publisher<T>` 资源。
  - `Subscriber<T>(topic)`：返回 `res::Subscriber<T>` 资源，供后续订阅。

- Channel 资源与操作
  - `res::Publisher<T>::Publish(const T&)`
  - `res::Publisher<T>::Publish(ContextRef ch_ctx, const T&)`
  - `res::Subscriber<T>::SubscribeInline(callback)`
  - `res::Subscriber<T>::SubscribeOn(executor, callback)`
  - 仅支持“直接支持类型”(DirectlySupportedType)：Protobuf 或（启用 ROS2 时）ROS2 Message。

- 订阅回调形态（均被标准化支持）：
  - `void(std::shared_ptr<const T>)`
  - `void(const T&)`
  - `void(aimrt::channel::ContextRef, std::shared_ptr<const T>)`
  - `void(aimrt::channel::ContextRef, const T&)`


## 使用方法

### 1. 在模块中获取 Context
```cpp
class MyModule : public aimrt::ModuleBase {
 public:
  bool Initialize(aimrt::CoreRef core) override {
    ctx_ = GetContext();
    // 可在此解析配置、拿执行器等
    return true;
  }

  bool Start() override {
    // 若要在其他执行器线程使用 ctx，需先投递并在该线程调用 LetMe()
    auto work = ctx_->GetExecutor("work_executor");
    work.Execute([ctx = ctx_]() {
      ctx->LetMe();
      // ... 后续可安全使用 aimrt::context::Ok() 等便捷函数
    });
    return true;
  }

  void Shutdown() override {
    ctx_->RequireToShutdown();
  }

 private:
  std::shared_ptr<aimrt::context::Context> ctx_;
};
```

### 2. 初始化 Channel 发布/订阅
```cpp
// Initialize 阶段
publisher_ = aimrt::context::init::Publisher<ExampleEventMsg>(topic_name);
subscriber_ = aimrt::context::init::Subscriber<ExampleEventMsg>(topic_name);

// 订阅（当前上下文线程内回调）
subscriber_.SubscribeInline([](std::shared_ptr<const ExampleEventMsg> msg) {
  if (aimrt::context::Ok()) {
    AIMRT_INFO("Received: {}", msg->msg());
  }
});

// 发布（Start 后执行）
publisher_.Publish(ExampleEventMsg{ /*...*/ });
```

### 3. 在线程/执行器中循环工作
```cpp
auto exe = ctx_->GetExecutor("work_executor");
exe.Execute([ctx = ctx_, pub = publisher_]() {
  ctx->LetMe();
  uint32_t count = 0;
  while (aimrt::context::Ok()) {
    ExampleEventMsg m; m.set_num(++count);
    pub.Publish(m);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
});
```


## 与 Channel Context 的关系

发布时可传入 `aimrt::channel::ContextRef` 携带 key-val 元信息（序列化类型、后端路由、透传链路信息等）。当订阅端需要把上游 Context 补写到下游发布，可：
- 使用 `aimrt::channel::PublisherRef::MergeSubscribeContextToPublishContext()`；
- 或使用 `PublisherProxy::NewContextSharedPtr(subscribe_ctx)` 构造新的发布侧 ctx。

详细 key 说明见 {{ '[channel_context_base.h]({}/src/interface/aimrt_module_c_interface/channel/channel_context_base.h)'.format(code_site_root_path_url) }} 与 Channel 文档。


## 最佳实践

- **线程绑定**：在任何将回调/任务投递到其它执行器的代码路径内，进入回调第一时间调用 `ctx->LetMe()`，保证 thread-local 上下文可用。
- **阶段约束**：类型注册与订阅应在 Initialize 阶段；消息发布应在 Start 之后。
- **中断检查**：长循环或阻塞流程中定期检查 `aimrt::context::Ok()`。
- **回调选择**：轻量回调可 Inline，重任务请 `SubscribeOn(executor, ...)` 切到专用执行器。


## 常见问题（FAQ）

- Q: 未调用 `LetMe()` 导致 `ExpectContext` 断言？
  - A: 在任务回调或新线程入口处补上 `ctx->LetMe()`。
- Q: 发布或订阅失败？
  - A: 确认 Initialize 阶段完成类型注册/订阅；消息发布需在 Start 后执行；检查 Topic 名称与后端配置。
- Q: 为什么我的类型不被支持？
  - A: 仅支持 Protobuf 或启用 ROS2 时的 ROS2 Message。请按对应 IDL 生成并链接所需 target。


## 参考示例

- 发布模块：
  - {{ '[channel_publisher_module.cc]({}/src/examples/cpp/context/module/channel_publisher_module/channel_publisher_module.cc)'.format(code_site_root_path_url) }}
- 订阅模块：
  - {{ '[channel_subscriber_module.cc]({}/src/examples/cpp/context/module/channel_subscriber_module/channel_subscriber_module.cc)'.format(code_site_root_path_url) }}
