# Context

## Related links

Code files:
- {{ '[aimrt_module_cpp_interface/context/context.h]({}/src/interface/aimrt_module_cpp_interface/context/context.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/op_pub.h]({}/src/interface/aimrt_module_cpp_interface/context/op_pub.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/op_sub.h]({}/src/interface/aimrt_module_cpp_interface/context/op_sub.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/op_cli.h]({}/src/interface/aimrt_module_cpp_interface/context/op_cli.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/op_srv.h]({}/src/interface/aimrt_module_cpp_interface/context/op_srv.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/context/res/channel.h]({}/src/interface/aimrt_module_cpp_interface/context/res/channel.h)'.format(code_site_root_path_url) }}

Reference examples:
- {{ '[context]({}/src/examples/cpp/context)'.format(code_site_root_path_url) }}
  - {{ '[channel_publisher_module.cc]({}/src/examples/cpp/context/module/chn_publisher_module/channel_publisher_module.cc)'.format(code_site_root_path_url) }}
  - {{ '[chn_subscriber_inline_module.cc]({}/src/examples/cpp/context/module/chn_subscriber_inline_module/chn_subscriber_inline_module.cc)'.format(code_site_root_path_url) }}
  - {{ '[chn_subscriber_on_exeutor_module.cc]({}/src/examples/cpp/context/module/chn_subscriber_on_exeutor_module/chn_subscriber_on_exeutor_module.cc)'.format(code_site_root_path_url) }}


## Concepts

Context is the "runtime context" provided by AimRT in the C++ interface. It spans the module's Initialize/Start/Shutdown call path and is used to:
- **In-thread context binding**: associate the current thread with the context via `Context::LetMe()`. Afterwards, convenience helpers such as `aimrt::context::Running()` are available.
- **Access core handles**: indirectly obtain handles for logger, executors, configurator, Channel/RPC/parameters via `Context::GetRawRef()` (`aimrt::CoreRef`).
- **Operation orchestration**: obtain operator objects via `ctx.pub()/sub()/cli()/srv()` to uniformly orchestrate Channel publish/subscribe and RPC client/server.
- **Lifecycle and cancellation**: use `StopRunning()` and `Running()` to express exit intent across tasks/threads.

In AimRT, modules inherit from `aimrt::ModuleBase`. The framework creates and injects a `Context` during initialization. The module can obtain a shared pointer via `GetContext()` and call `LetMe()` in task threads to bind the current execution environment.


## Core types and APIs

- `aimrt::context::Context`
  - `GetRawRef()`/`GetLogger()`: access the underlying `CoreRef` and logger.
  - `LetMe()`: point the current thread's thread-local context to this `Context`.
  - `StopRunning()`/`Running()`: signal a stop request and check it inside loops.
  - `pub()/sub()/cli()/srv()`: return `OpPub`/`OpSub`/`OpCli`/`OpSrv` operators respectively.

  - `CreateExecutor(name)`: get (create) an executor handle by name.
  - `CreatePublisher<T>(topic)`: register the publish type and return a `res::Publisher<T>` resource.
  - `CreateSubscriber<T>(topic)`: return a `res::Subscriber<T>` resource for subsequent subscription.
  - `CreateSubscriber<T>(topic, callback)`: register a subscription on the given `topic`, execute the callback on the backend executor, and return the corresponding `res::Subscriber<T>` instance.
  - `CreateSubscriber<T>(topic, executor, callback)`: register a subscription on the given `topic`, execute the callback on the specified executor, and return the `res::Subscriber<T>` instance.


## Usage

### 1. Obtain the Context inside a module
```cpp
class MyModule : public aimrt::ModuleBase {
 public:
  bool Initialize(aimrt::CoreRef core) override {
    ctx_ = GetContext();
    ctx->LetMe();
    // Parse config, get executors, etc. here
    return true;
  }

  bool Start() override {
    // If you use ctx on another executor thread, first post a task
    // and call LetMe() on that thread
    auto work = ctx_->CreateExecutor("work_executor");
    work.Execute([ctx = ctx_]() {
      ctx->LetMe();
      // ... You can safely use aimrt::context::Running() thereafter
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

### 2. Initialize Channel publish/subscribe
```cpp
// During Initialize phase
publisher_ = ctx_->CreatePublisher<ExampleEventMsg>(topic_name);

// Subscribe (callback on the current context thread)
subscriber_inline_ = ctx_->CreateSubscriber<ExampleEventMsg>(
  topic_name_, [this](std::shared_ptr<ExampleEventMsg> msg) {
  if (aimrt::context::Running()) {
    AIMRT_INFO("Received: {}", msg->msg());
  }
});

// Subscribe (callback executed on a specified executor)
subscriber_inline_ = ctx_->CreateSubscriber<ExampleEventMsg>(
  topic_name_, work_executor_, [this](std::shared_ptr<ExampleEventMsg> msg) {
  if (aimrt::context::Running()) {
    AIMRT_INFO("Received: {}", msg->msg());
  }
});


// Publish (after Start)
publisher_.Publish(ExampleEventMsg{ /*...*/ });
```

### 3. Looping work in a thread/executor
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


## Relationship with Channel/RPC Context

This section distinguishes the "runtime Context" (`aimrt::context::Context`, hereafter Runtime Context) from the "transport-layer Context" (Channel/RPC Context). They serve different purposes:

- Runtime Context: the runtime environment for a module's lifecycle, providing executors, logging, configuration, Channel/RPC operators, etc.; you must call `LetMe()` in task/callback threads to complete thread binding, and use `Running()/StopRunning()` to express exit semantics.
- Transport-layer Context: a metadata container attached to a single message or a single RPC call that travels through the channel/call to carry routing, serialization strategy, tracing, and more.

Channel Context (`aimrt::channel::ContextRef`):
- Purpose:
  - Carries key-value metadata such as serialization type, backend selection, routing, and tracing. It travels with a single message along the publish-subscribe chain.
- Key usage points:
  - Publisher side can call `publisher.Publish(ch_ctx, msg)` to specify the context;
  - Subscription callbacks receive the Channel Context to read upstream metadata;
  - If you need to propagate the subscriber-side Context into subsequent publishing:
    - Use `aimrt::channel::PublisherRef::MergeSubscribeContextToPublishContext()` to merge; or
    - Construct a new publisher-side context via `PublisherProxy::NewContextSharedPtr(subscribe_ctx)`.
- For key definitions, see {{ '[channel_context_base.h]({}/src/interface/aimrt_module_c_interface/channel/channel_context_base.h)'.format(code_site_root_path_url) }} and the Channel documentation.

RPC Context (`aimrt::rpc::ContextRef`):
- Purpose:
  - Carries per-call RPC metadata (e.g., timeout/deadline, call ID, routing/backend, user-defined metadata) across the entire chain from the client invocation to the server handling.
- Key usage points:
  - Client: obtain the operator via `ctx.cli()` to initiate the call; set deadline and attach metadata on `ContextRef`;
  - Server: the service coroutine/callback receives `rpc::ContextRef` to read caller metadata and set response-related fields;
  - Status: calls use `aimrt::rpc::Status` to represent result codes/error messages, which differs from the Channel's "message-delivery" semantics.

Differences from Runtime Context:
- Scope: Runtime Context targets the module/thread runtime environment; Channel/RPC Context targets transport metadata for a single message/call.
- Lifecycle: Runtime Context is created by the framework at module initialization and lasts for the module's lifecycle; transport-layer Context is created for a single publish/call and ends with it.
- Creation: Runtime Context is obtained via `GetContext()` and `LetMe()` is called in worker threads; transport-layer Context is constructed by the underlying channel/RPC stack during send/receive and is provided as a `ContextRef` parameter in callbacks.
- Focus: Runtime Context focuses on execution, logging, configuration, and resource orchestration; transport-layer Context focuses on link-level metadata and cross-boundary propagation.


## Best practices

- **Thread binding**: in any code path that posts callbacks/tasks to other executors, call `ctx->LetMe()` at the very beginning of the callback to ensure the thread-local context is available.
- **Phase constraints**: perform type registration and subscriptions during Initialize; publish messages only after Start.
- **Cancellation checks**: regularly check `aimrt::context::Ok()` in long loops or blocking flows.
- **Callback placement**: use Inline for lightweight callbacks; for heavy work, use `SubscribeOn(executor, ...)` to switch to a dedicated executor.


## Advantages Compared to Non-Context Interfaces
- **Single entry and consistency**:
  - `Context` unifies the creation/management of executors and Channel resources (`CreateExecutor/CreatePublisher/CreateSubscriber`), replacing older interfaces and making aggregation clearer.
- **More intuitive lifecycle expression**:
  - `Running()/StopRunning()` make run/exit semantics explicit. Combined with `LetMe()`, you can check exit conditions directly after a thread switch, reducing boilerplate and misuse.
- **Controllable threading model**:
  - Subscriptions support both "backend-executor callbacks" and "specified-executor callbacks". Choose flexibly based on workload to avoid blocking critical threads or unnecessary thread switches.
- **Type safety and standardized callbacks**:
  - Compile-time checks based on constraints (DirectlySupportedType, callback signature rules) validate type support; unified adaptation of various callback forms reduces glue code.
- **Better error localization**:
  - The APIs internally use `std::source_location` to propagate call-site information, making assertions and errors easier to trace (e.g., when fetching executors or registering types fails).


## Reference examples

- Publisher module:
  - {{ '[channel_publisher_module.cc]({}/src/examples/cpp/context/module/chn_publisher_module/channel_publisher_module.cc)'.format(code_site_root_path_url) }}
- Subscriber module (inline callback):
  - {{ '[chn_subscriber_inline_module.cc]({}/src/examples/cpp/context/module/chn_subscriber_inline_module/chn_subscriber_inline_module.cc)'.format(code_site_root_path_url) }}
- Subscriber module (callback on executor thread):
  - {{ '[chn_subscriber_on_exeutor_module.cc]({}/src/examples/cpp/context/module/chn_subscriber_on_exeutor_module/chn_subscriber_on_exeutor_module.cc)'.format(code_site_root_path_url) }}
