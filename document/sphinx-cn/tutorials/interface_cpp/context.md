# Context

## 相关链接

代码文件：
- {{ '[aimrt_module_cpp_interface/context/context.h]({}/src/interface/aimrt_module_cpp_interface/context/context.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/op_pub.h]({}/src/interface/aimrt_module_cpp_interface/context/op_pub.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/op_sub.h]({}/src/interface/aimrt_module_cpp_interface/context/op_sub.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/op_cli.h]({}/src/interface/aimrt_module_cpp_interface/context/op_cli.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/op_srv.h]({}/src/interface/aimrt_module_cpp_interface/context/op_srv.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/res/channel.h]({}/src/interface/aimrt_module_cpp_interface/context/res/channel.h)'.format(code_site_root_path_url) }}

参考示例：
- {{ '[context]({}/src/examples/cpp/context)'.format(code_site_root_path_url) }}
  - {{ '[channel_publisher_module.cc]({}/src/examples/cpp/context/module/chn_publisher_module/channel_publisher_module.cc)'.format(code_site_root_path_url) }}
  - {{ '[chn_subscriber_inline_module.cc]({}/src/examples/cpp/context/module/chn_subscriber_inline_module/chn_subscriber_inline_module.cc)'.format(code_site_root_path_url) }}
  - {{ '[chn_subscriber_on_exeutor_module.cc]({}/src/examples/cpp/context/module/chn_subscriber_on_exeutor_module/chn_subscriber_on_exeutor_module.cc)'.format(code_site_root_path_url) }}


## 概念

Context 是 AimRT 在 C++ 接口中提供的“运行期上下文”，贯穿模块 Initialize/Start/Shutdown 的调用路径，用于：
- **线程内上下文绑定**：通过 `Context::LetMe()` 把当前线程与上下文关联，后续可用 `aimrt::context::Running()` 等便捷函数访问。
- **访问核心句柄**：通过 `Context::GetRawRef()`（`aimrt::CoreRef`）间接获取日志、执行器、配置器、Channel/RPC/参数等句柄。
- **编排操作**：通过 `ctx.pub()/sub()/cli()/srv()` 获取操作器对象，统一完成 Channel 发布/订阅与 RPC 客户端/服务端编排。
- **生命周期与中断**：`StopRunning()` 和 `Running()` 用于跨任务/线程表达退出意图。

在 AimRT 中，模块继承 `aimrt::ModuleBase`，框架会在初始化阶段为模块创建 `Context` 并注入，模块可通过 `GetContext()` 获取共享指针，并在任务线程中调用 `LetMe()` 绑定当前执行环境。


## 核心类型与接口

- `aimrt::context::Context`
  - `GetRawRef()`/`GetLogger()`：访问底层 `CoreRef` 与 Logger。
  - `LetMe()`：将当前线程的 thread-local 上下文指向本 `Context`。
  - `StopRunning()`/`Running()`：发出停止请求并在循环内检测。
  - `CreateExecutor(name)`：按名称获取（创建）执行器句柄。
  - `pub()/sub()/cli()/srv()`：分别返回 `OpPub`/`OpSub`/`OpCli`/`OpSrv` 操作器。

  - `CreatePublisher<T>(topic)`：注册发布类型并返回 `res::Publisher<T>` 资源。
  - `CreateSubscriber<T>(topic)`：返回 `res::Subscriber<T>` 资源，供后续订阅。
  - `CreateSubscriber<T>(topic, callback)`：在指定 `topic` 中注册订阅，并在后端执行器中执行相应的回调函数并返回对应的 `res::Subscriber<T>` 实例。
  - `CreateSubscriber<T>(topic, executor, callback)`：在指定 `topic` 中注册订阅，并在指定执行器中执行回调函数并返回 `res::Subscriber<T>` 实例。

- Channel 资源与操作
  - `res::Publisher<T>::Publish(const T&)`
  - `res::Publisher<T>::Publish(ContextRef ch_ctx, const T&)`
  - `res::Subscriber<T>::SubscribeInline(callback)`
  - `res::Subscriber<T>::SubscribeOn(executor, callback)`
  - 仅支持“直接支持类型”(DirectlySupportedType)：Protobuf 或 ROS2 Message。

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
    ctx->LetMe();
    // 可在此解析配置、拿执行器等
    return true;
  }

  bool Start() override {
    // 若要在其他执行器线程使用 ctx，需先投递并在该线程调用 LetMe()
    auto work = ctx_->CreateExecutor("work_executor");
    work.Execute([ctx = ctx_]() {
      ctx->LetMe();
      // ... 后续可安全使用 aimrt::context::Running()
    });
    return true;
  }

  void Shutdown() override {
    ctx_->StopRunning();
  }

 private:
  std::shared_ptr<aimrt::context::Context> ctx_;
};
```

### 2. 初始化 Channel 发布/订阅
```cpp
// Initialize 阶段
publisher_ = ctx_->CreatePublisher<ExampleEventMsg>(topic_name);
subscriber_ = ctx_->CreateSubscriber<ExampleEventMsg>(topic_name);

// 订阅（当前上下文线程内回调）
subscriber_.SubscribeInline([](std::shared_ptr<const ExampleEventMsg> msg) {
  if (aimrt::context::Running()) {
    AIMRT_INFO("Received: {}", msg->msg());
  }
});

// 发布（Start 后执行）
publisher_.Publish(ExampleEventMsg{ /*...*/ });
```

### 3. 在线程/执行器中循环工作
```cpp
auto exe = ctx_->CreateExecutor("work_executor");
exe.Execute([ctx = ctx_, pub = publisher_]() {
  ctx->LetMe();
  uint32_t count = 0;
  while (aimrt::context::Running()) {
    ExampleEventMsg m; m.set_num(++count);
    pub.Publish(m);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
});
```


## 与 Channel/Rpc Context 的关系

本节区分“运行期 Context”（`aimrt::context::Context`，下称 Runtime Context）与“传输层 Context”（Channel/RPC Context）。二者作用不同：

- Runtime Context：模块生命周期内的运行期上下文，提供执行器、日志、配置、Channel/RPC 操作器等能力；需在任务/回调线程中调用 `LetMe()` 完成线程绑定，用 `Running()/StopRunning()` 表达退出语义。
- 传输层 Context：附着于一次消息或一次 RPC 调用的元信息容器，随消息/调用在通道内传播，用于路由、序列化策略、链路追踪等。

Channel Context（`aimrt::channel::ContextRef`）：
- 用途：
  - 携带序列化类型、后端选择、路由、链路追踪等 key-val 元信息，伴随单条消息在发布-订阅链路中传播。
- 使用要点：
  - 发布侧可调用 `publisher.Publish(ch_ctx, msg)` 指定上下文；
  - 订阅回调可获得 Channel Context，用于读取上游附带的元信息；
  - 若需把订阅侧的 Context 透传到后续发布：
    - 使用 `aimrt::channel::PublisherRef::MergeSubscribeContextToPublishContext()` 合并；
    - 或通过 `PublisherProxy::NewContextSharedPtr(subscribe_ctx)` 构造新的发布侧上下文。
- 参考键值定义：见 {{ '[channel_context_base.h]({}/src/interface/aimrt_module_c_interface/channel/channel_context_base.h)'.format(code_site_root_path_url) }} 与 Channel 文档。

RPC Context（`aimrt::rpc::ContextRef`）：
- 用途：
  - 携带单次 RPC 调用的元信息（如超时/截止时间、调用 ID、路由/后端、用户自定义 metadata），贯穿客户端发起到服务端处理的整条调用链。
- 使用要点：
  - 客户端：通过 `ctx.cli()` 获取操作器发起调用，可在 `ContextRef` 上设置截止时间、附加元信息；
  - 服务端：服务处理协程/回调会接收 `rpc::ContextRef`，可读取调用方元信息并回写响应相关字段；
  - 状态：调用以 `aimrt::rpc::Status` 表达结果码/错误信息，与 Channel 的“消息即达”语义不同。

与 Runtime Context 的区别：
- 作用域：Runtime Context 面向模块/线程的运行环境；Channel/RPC Context 面向“单条消息/单次调用”的传输元信息。
- 生命周期：Runtime Context 由框架在模块初始化时创建并贯穿模块生命周期；传输层 Context 由一次发布/一次调用创建并随之结束。
- 创建方式：Runtime Context 通过 `GetContext()` 获取并在工作线程 `LetMe()`；传输层 Context 由底层通道/RPC 栈在收发时构造，回调中以 `ContextRef` 形参提供。
- 关注点：Runtime Context 聚焦执行、日志、配置、资源编排；传输层 Context 聚焦链路级元信息与跨边界透传。


## 最佳实践

- **线程绑定**：在任何将回调/任务投递到其它执行器的代码路径内，进入回调第一时间调用 `ctx->LetMe()`，保证 thread-local 上下文可用。
- **阶段约束**：类型注册与订阅应在 Initialize 阶段；消息发布应在 Start 之后。
- **中断检查**：长循环或阻塞流程中定期检查 `aimrt::context::Ok()`。
- **回调选择**：轻量回调可 Inline，重任务请 `SubscribeOn(executor, ...)` 切到专用执行器。


## 相较旧接口的优势
- **单一入口与一致性**：
  - 由 `Context` 统一创建/管理执行器与 Channel 资源（`CreateExecutor/CreatePublisher/CreateSubscriber`），替代原来的接口，接口聚合更清晰。
- **生命周期表达更直观**：
  - 使用 `Running()/StopRunning()` 明确运行/退出语义，配合 `LetMe()` 在线程切换后可直接判断退出条件，减少样板代码与误用。
- **线程模型可控**：
  - 订阅支持“后端执行器回调”与“指定执行器回调”两种模式，按工作负载灵活选择，避免阻塞关键线程或产生不必要的线程切换。
- **类型安全与回调标准化**：
  - 基于约束（DirectlySupportedType、回调签名规范）在编译期检查类型支持；统一适配多种回调形态，减少胶水代码。
- **错误定位更友好**：
  - 接口内部使用 `std::source_location` 传递调用位置信息，断言与报错更易定位源头（如获取执行器/注册类型失败）。


## 参考示例

- 发布模块：
  - {{ '[channel_publisher_module.cc]({}/src/examples/cpp/context/module/chn_publisher_module/channel_publisher_module.cc)'.format(code_site_root_path_url) }}
- 订阅模块（内联回调）：
  - {{ '[chn_subscriber_inline_module.cc]({}/src/examples/cpp/context/module/chn_subscriber_inline_module/chn_subscriber_inline_module.cc)'.format(code_site_root_path_url) }}
- 订阅模块（在线程执行器上回调）：
  - {{ '[chn_subscriber_on_exeutor_module.cc]({}/src/examples/cpp/context/module/chn_subscriber_on_exeutor_module/chn_subscriber_on_exeutor_module.cc)'.format(code_site_root_path_url) }}
