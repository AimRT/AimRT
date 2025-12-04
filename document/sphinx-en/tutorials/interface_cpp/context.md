# Context

## Related Links

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
  - {{ '[rpc_client_module.cc]({}/src/examples/cpp/context/module/rpc_client_module/rpc_client_module.cc)'.format(code_site_root_path_url) }}
  - {{ '[rpc_server_inline_module.cc]({}/src/examples/cpp/context/module/rpc_server_inline_module/rpc_server_inline_module.cc)'.format(code_site_root_path_url) }}
  - {{ '[rpc_server_on_executor_module.cc]({}/src/examples/cpp/context/module/rpc_server_on_executor_module/rpc_server_on_executor_module.cc)'.format(code_site_root_path_url) }}


## Concepts

Context is the runtime context provided by AimRT in the C++ interface. It runs through the module's Initialize/Start/Shutdown call path and is used for:
- **Per-thread context binding**: Associate the current thread with the context via `Context::LetMe()`, then access it with helpers like `aimrt::context::Running()`.
- **Access to core handles**: Indirectly obtain handles for logging, executors, configuration, Channel/RPC/parameters through `Context::GetRawRef()` (`aimrt::CoreRef`).
- **Operation orchestration**: Get operator objects via `ctx.pub()/sub()/cli()/srv()` to uniformly orchestrate Channel publish/subscribe and RPC client/server.
- **Lifecycle and interruption**: Use `StopRunning()` and `Running()` to express exit intent across tasks/threads.

In AimRT, modules inherit from `aimrt::ModuleBase`. During Initialize, create and bind a `Context` to the current thread via `aimrt::context::Context::Letme(core)`; in callbacks dispatched to other executors, call `ctx->LetMe()` first to complete thread binding.


## Core Types and Interfaces

- `aimrt::context::Context`
  - `GetRawRef()`/`GetLogger()`: Access underlying `CoreRef` and the logger.
  - `LetMe()`: Point the current thread's thread-local context to this `Context`.
  - `StopRunning()`/`Running()`: Issue a stop request and check it within loops.
  - `SetPubState()`/`SetSubState()`: Use `ChannelState::kOn|kOff` to control whether all publish/subscribe channels under the current Context are open to the outside. The corresponding `GetPubState()`/`GetSubState()` can be used to check the real-time status.
  - `SetCliState()`/`SetSrvState()`: Use `RpcState::kOn|kOff` to control whether the RPC client or server accepts requests or initiates calls. You can monitor their status with `GetCliState()`/`GetSrvState()`.
  - `pub()/sub()/cli()/srv()`: Return `OpPub`/`OpSub`/`OpCli`/`OpSrv` operators, respectively.
  - `CreateExecutor(name)`: Get (or create) an executor handle by name.
  - `CreatePublisher<T>(topic)`: Register the publish type and return a `res::Publisher<T>` resource.
  - `CreateSubscriber<T>(topic)`: Return a `res::Subscriber<T>` resource for subsequent subscription.
  - `CreateSubscriber<T>(topic, callback)`: Register a subscription on the given `topic`, execute the callback on the backend executor, and return the `res::Subscriber<T>` instance.
  - `CreateSubscriber<T>(topic, executor, callback)`: Register a subscription on the given `topic`, execute the callback on the specified executor, and return the `res::Subscriber<T>` instance.
  - `CreateClient<T>()`: Create an RPC client and return the client instance for initiating RPC calls.
  - `CreateServer<T>()`: Create an RPC server and return the server instance, register service handlers via `ServeInline()` or `ServeOn(executor, ...)`.


## Usage

### 1. Acquire Context in a Module
```cpp
class MyModule : public aimrt::ModuleBase {
 public:
  bool Initialize(aimrt::CoreRef core) override {
    // Create and bind Context to the current thread
    ctx_ = std::make_shared<aimrt::context::Context>(core);
    ctx_->LetMe();
    // Parse config, obtain executors, etc. (e.g., ctx_->GetConfigFilePath())
    return true;
  }

  bool Start() override {
    // If ctx is used on another executor thread, dispatch first and call LetMe() there
    auto work = ctx_->CreateExecutor("work_executor");
    work.Execute([ctx = ctx_]() {
      ctx->LetMe();
      // ... It is now safe to use aimrt::context::Running()
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

### 2. Initialize Channel Publish/Subscribe

```cpp
// Initialize phase
publisher_ = ctx_->CreatePublisher<ExampleEventMsg>(topic_name_);

// Subscribe (callback on current context thread)
subscriber_inline_ = ctx_->CreateSubscriber<ExampleEventMsg>(
    topic_name_, [this](std::shared_ptr<const ExampleEventMsg> msg) {
      if (aimrt::context::Running()) {
        AIMRT_INFO("Received: {}", msg->msg());
      }
    });

// Subscribe (callback on specified executor)
subscriber_on_executor_ = ctx_->CreateSubscriber<ExampleEventMsg>(
    topic_name_, work_executor_, [this](std::shared_ptr<const ExampleEventMsg> msg) {
      if (aimrt::context::Running()) {
        AIMRT_INFO("Received: {}", msg->msg());
      }
    });

// Publish (after Start)
publisher_.Publish(ExampleEventMsg{ /*...*/ });
```

### 3. Loop work on a thread/executor
```cpp
auto exe = ctx_->CreateExecutor("work_executor");
exe.Execute([ctx = ctx_, pub = publisher_]() {
  ctx_ = std::make_shared<aimrt::context::Context>(core);
  ctx_->LetMe();
  uint32_t count = 0;
  while (aimrt::context::Running()) {
    ExampleEventMsg m; m.set_num(++count);
    pub.Publish(m);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
});
```

### 4. Initialize RPC Client and Server

#### RPC Client
```cpp
// Create client during Initialize phase
client_ = ctx_->CreateClient<aimrt::protocols::example::ExampleServiceCoClient>();

// Initiate RPC calls after Start
aimrt::co::Task<void> RpcTask() {
  ctx_->LetMe();

  while (ctx_->Running()) {
    aimrt::protocols::example::GetFooDataReq req;
    aimrt::protocols::example::GetFooDataRsp rsp;
    req.set_msg("hello world");

    // Initiate async RPC call
    auto status = co_await client_.GetFooData(req, rsp);
    if (status.OK()) {
      AIMRT_INFO("RPC call succeeded, response: {}", rsp.msg());
    } else {
      AIMRT_WARN("RPC call failed, status: {}", status.ToString());
    }

    co_await aimrt::co::ScheduleAfter(
        aimrt::co::AimRTScheduler(time_executor_),
        std::chrono::milliseconds(1000));
  }

  co_return;
}
```

#### RPC Server
```cpp
// Create server during Initialize phase, use ServeInline to handle on backend executor (not recommended)
auto server = ctx_->CreateServer<aimrt::protocols::example::ExampleServiceCoServer>();

server->GetBarData.ServeInline(
    [this](aimrt::rpc::ContextRef ctx,
           const aimrt::protocols::example::GetBarDataReq& req,
           aimrt::protocols::example::GetBarDataRsp& rsp) {
      AIMRT_INFO("Handle GetBarData: {}", aimrt::Pb2CompactJson(req));
      rsp.set_msg("echo " + req.msg());
      return aimrt::rpc::Status();
    });
```

#### RPC Server (Execute on Executor)
```cpp
// Create server during Initialize phase, use ServeOn to handle requests on specified executor
auto work_executor = ctx_->CreateExecutor("work_thread_pool");
auto server = ctx_->CreateServer<aimrt::protocols::example::ExampleServiceCoServer>();

server->GetFooData.ServeOn(work_executor,
    [this](aimrt::rpc::ContextRef ctx,
           const aimrt::protocols::example::GetFooDataReq& req,
           aimrt::protocols::example::GetFooDataRsp& rsp) {
      AIMRT_INFO("Handle GetFooData on executor: {}", aimrt::Pb2CompactJson(req));
      rsp.set_msg("echo " + req.msg());
      return aimrt::rpc::Status();
    });
```


## Relation to Channel/RPC Context

This section distinguishes the "Runtime Context" (`aimrt::context::Context`, below as Runtime Context) from the "Transport-layer Context" (Channel/RPC Context). They serve different purposes:

- Runtime Context: The runtime environment within the module lifecycle that provides executors, logging, configuration, Channel/RPC operators, etc.; call `LetMe()` in task/callback threads to complete thread binding, and use `Running()/StopRunning()` to express exit semantics.
- Transport-layer Context: A metadata container attached to a single message or a single RPC call, propagating through the channel to carry routing, serialization strategy, tracing, and more.

Channel Context (`aimrt::channel::ContextRef`):
- Purpose:
  - Carries key-value metadata such as serialization type, backend selection, routing, and tracing; it travels with a single message along the pub-sub pipeline.
- Usage notes:
  - On the publish side, call `publisher.Publish(ch_ctx, msg)` to specify the context;
  - Subscription callbacks can receive the Channel Context to read upstream metadata;
  - To propagate the subscriber-side context to subsequent publishing:
    - Use `aimrt::channel::PublisherRef::MergeSubscribeContextToPublishContext()` to merge; or
    - Construct a new publish-side context via `PublisherProxy::NewContextSharedPtr(subscribe_ctx)`.
- Reference key definitions: see {{ '[channel_context_base.h]({}/src/interface/aimrt_module_c_interface/channel/channel_context_base.h)'.format(code_site_root_path_url) }} and the Channel documentation.

RPC Context (`aimrt::rpc::ContextRef`):
- Purpose:
  - Carries metadata for a single RPC call (e.g., deadline, call ID, routing, user-defined metadata), spanning from client invocation to server handling.
- Usage notes:
  - Client: Initiate calls via the operator obtained from `ctx.cli()`, set deadlines and attach metadata on the `ContextRef`;
  - Server: Service coroutine/callback receives `rpc::ContextRef` to read caller metadata and write response-related fields back;
  - Status: Results are represented by `aimrt::rpc::Status` (code/message), differing from Channel's "message delivered" semantics.

Differences from Runtime Context:
- Scope: Runtime Context targets the module/thread runtime environment; Channel/RPC Context targets transport metadata for a single message/call.
- Lifecycle: Runtime Context is created by the framework during module initialization and spans the module's lifecycle; transport Context is created per publish/call and ends with it.
- Creation: Runtime Context is created via `Context::Letme(core)` and bound in worker threads with `LetMe()`; transport Context is constructed by the underlying channel/RPC stack on send/receive and is provided as a `ContextRef` parameter in callbacks.
- Focus: Runtime Context focuses on execution, logging, configuration, and resource orchestration; transport Context focuses on link-level metadata and cross-boundary propagation.


## Best Practices

- **Thread binding**: In any code path that dispatches callbacks/tasks to other executors, call `ctx->LetMe()` at the beginning of the callback to ensure the thread-local context is available.
- **Phase constraints**:
  - Channel: Type registration and subscription should be done during Initialize; message publishing should occur after Start.
  - RPC: Client/server creation and service registration should be done during Initialize; RPC calls should occur after Start.
- **Interruption checks**: Periodically check `aimrt::context::Running()` in long loops or blocking flows.
- **Callback placement**:
  - Channel subscription: Lightweight callbacks can be inline; for heavy work, use `CreateSubscriber(topic, executor, callback)` to switch to a dedicated executor.
  - RPC service: Lightweight handlers can use `ServeInline()`; for heavy work, use `ServeOn(executor, ...)` to switch to a dedicated executor.


## Advantages over non-context APIs
- **Single entry and consistency**:
  - `Context` uniformly creates/manages executors, Channel and RPC resources (`CreateExecutor/CreatePublisher/CreateSubscriber/CreateClient/CreateServer`), replacing older interfaces with clearer aggregation.
- **More intuitive lifecycle expression**:
  - `Running()/StopRunning()` clearly conveys run/exit semantics; with `LetMe()` after thread switches, exit conditions can be checked directly, reducing boilerplate and misuse.
- **Controllable threading model**:
  - Channel subscriptions support both "backend executor callback" and "specified executor callback"; RPC servers support both "inline execution" (`ServeInline`) and "executor execution" (`ServeOn`), allowing flexible choices based on workload to avoid blocking critical threads or unnecessary thread switches.
- **Type safety and standardized callbacks**:
  - Callback signatures are constrained and checked at compile time (supporting with/without `ContextRef`, pointer/reference, etc.); unified adaptation reduces glue code.
- **Friendlier error localization**:
  - Interfaces pass call-site information using `std::source_location`, making assertions and errors easier to trace (e.g., failing to get an executor or register a type).


## Reference Examples

### Channel Examples
- Publisher module:
  - {{ '[channel_publisher_module.cc]({}/src/examples/cpp/context/module/chn_publisher_module/channel_publisher_module.cc)'.format(code_site_root_path_url) }}
- Subscriber module (backend executor callback):
  - {{ '[chn_subscriber_inline_module.cc]({}/src/examples/cpp/context/module/chn_subscriber_inline_module/chn_subscriber_inline_module.cc)'.format(code_site_root_path_url) }}
- Subscriber module (callback on specified executor):
  - {{ '[chn_subscriber_on_exeutor_module.cc]({}/src/examples/cpp/context/module/chn_subscriber_on_exeutor_module/chn_subscriber_on_exeutor_module.cc)'.format(code_site_root_path_url) }}

### RPC Examples
- RPC client module:
  - {{ '[rpc_client_module.cc]({}/src/examples/cpp/context/module/rpc_client_module/rpc_client_module.cc)'.format(code_site_root_path_url) }}
- RPC server module (backend executor execution):
  - {{ '[rpc_server_inline_module.cc]({}/src/examples/cpp/context/module/rpc_server_inline_module/rpc_server_inline_module.cc)'.format(code_site_root_path_url) }}
- RPC server module (execution on specified executor):
  - {{ '[rpc_server_on_executor_module.cc]({}/src/examples/cpp/context/module/rpc_server_on_executor_module/rpc_server_on_executor_module.cc)'.format(code_site_root_path_url) }}
