# context examples

本目录给出基于 Context 能力的四个独立示例：执行器（定时调度）、Channel 发布、Channel 订阅（在给定执行器上执行回调）与 Channel 订阅（在后端执行器上执行回调）。示例均以 Pkg 方式集成，运行脚本与配置文件已在构建时拷贝至 build 目录。

通用前置条件（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF` 选项编译 AimRT；
- 构建后，直接在 build 目录运行对应 `start_examples_cpp_context_*.sh`；
- 键入 `ctrl-c` 结束进程；


## context executor

演示内容：
- **使用 Context 创建可定时调度的执行器**；
- **按 1s 周期循环调度任务**；
- **以 Pkg 方式加载模块并启动**。

核心代码：
- [executor_module.cc](./module/executor/executor_module.cc)
- [executor_module.h](./module/executor/executor_module.h)
- [pkg_main.cc](./pkg/pkg_main.cc)

配置文件：
- [examples_cpp_context_executor_cfg.yaml](./install/linux/bin/cfg/examples_cpp_context_executor_cfg.yaml)

运行方式（linux）：
- 在 build 目录运行 `start_examples_cpp_context_executor.sh`；

说明：
- 模块 `ContextExecutorModule` 通过 `time_schedule_executor` 每秒调度一次，打印计数并继续投递下一次；
- 仅演示执行器与定时调度能力，不涉及消息通信。


## context channel publisher

演示内容：
- **在 Context 中创建发布者，发布 `ExampleEventMsg`（protobuf）消息**；
- **使用 local 类型的 channel 后端**；
- **与 pb_chn 示例中的订阅模块协同运行**（`NormalSubscriberModule`）；
- **以 Pkg 方式集成并启动**。

核心代码：
- [channel_publisher_module.cc](./module/chn_publisher_module/channel_publisher_module.cc)
- [channel_publisher_module.h](./module/chn_publisher_module/channel_publisher_module.h)
- [pkg_main.cc](./pkg/pkg_main.cc)
- 相关消息定义：[event.proto](../../../protocols/pb/example/event.proto)

配置文件：
- [examples_cpp_context_publisher_cfg.yaml](./install/linux/bin/cfg/examples_cpp_context_publisher_cfg.yaml)

运行方式（linux）：
- 在 build 目录运行 `start_examples_cpp_context_publisher.sh`；

说明：
- 模块 `ContextChannelPublisherModule` 从配置读取 `topic_name` 与发布频率 `channel_frq`，在 `work_executor` 上循环发布；
- 配置中同时加载 `pb_chn_sub_pkg` 以启用 `NormalSubscriberModule`，完成同进程本地通信；
- 通信后端使用 `local`，发布与订阅主题均默认开启。


## context channel subscriber on executor

演示内容：
- **在 Context 中创建订阅者，回调在线程执行器 `work_executor` 上运行**；
- **publisher 由 pb_chn 示例提供**（`NormalPublisherModule`）；
- **使用 local 类型的 channel 后端**；
- **以 Pkg 方式集成并启动**。

核心代码：
- [chn_subscriber_on_exeutor_module.cc](./module/chn_subscriber_on_exeutor_module/chn_subscriber_on_exeutor_module.cc)
- [chn_subscriber_on_exeutor_module.h](./module/chn_subscriber_on_exeutor_module/chn_subscriber_on_exeutor_module.h)
- [pkg_main.cc](./pkg/pkg_main.cc)

配置文件：
- [examples_cpp_context_subscriber_on_executor_cfg.yaml](./install/linux/bin/cfg/examples_cpp_context_subscriber_on_executor_cfg.yaml)

运行方式（linux）：
- 在 build 目录运行 `start_examples_cpp_context_subscriber_on_executor.sh`；

说明：
- 模块 `ContextSubscriberOnExecutorModule` 在 `work_executor` 上执行回调，打印收到的消息；
- 配置中加载 `pb_chn_pub_pkg` 以启用 `NormalPublisherModule` 作为发布端；
- 通信后端使用 `local`，订阅端不使用内联执行器。


## context channel subscriber inline

演示内容：
- **在 Context 中创建订阅者，回调以内联方式执行**；
- **publisher 由 pb_chn 示例提供**（`NormalPublisherModule`）；
- **使用 local 类型的 channel 后端**；
- **以 Pkg 方式集成并启动**。

核心代码：
- [chn_subscriber_inline_module.cc](./module/chn_subscriber_inline_module/chn_subscriber_inline_module.cc)
- [chn_subscriber_inline_module.h](./module/chn_subscriber_inline_module/chn_subscriber_inline_module.h)
- [pkg_main.cc](./pkg/pkg_main.cc)

配置文件：
- [examples_cpp_context_subscriber_inline_cfg.yaml](./install/linux/bin/cfg/examples_cpp_context_subscriber_inline_cfg.yaml)

运行方式（linux）：
- 在 build 目录运行 `start_examples_cpp_context_subscriber_inline.sh`；

说明：
- 模块 `ContextSubscriberInlineModule` 以内联方式处理回调并打印消息；
- 配置中加载 `pb_chn_pub_pkg` 作为发布端；
- 通信后端使用 `local`，并开启 `subscriber_use_inline_executor: true`。


## context rpc server on executor

演示内容：
- **在 Context 中创建 RPC 服务端，回调在线程执行器 `work_thread_pool` 上运行**；
- **实现 `ExampleService` 的 `GetBarData` 和 `GetFooData` 两个 RPC 方法**；
- **使用 local 类型的 RPC 后端**；
- **与 pb_rpc 示例中的客户端模块协同运行**（`NormalRpcCoClientModule`）；
- **以 Pkg 方式集成并启动**。

核心代码：
- [rpc_server_on_executor_module.cc](./module/rpc_server_on_executor_module/rpc_server_on_executor_module.cc)
- [rpc_server_on_executor_module.h](./module/rpc_server_on_executor_module/rpc_server_on_executor_module.h)
- [pkg_main.cc](./pkg/pkg_main.cc)
- 相关协议定义：[rpc.proto](../../../protocols/pb/example/rpc.proto)

配置文件：
- [examples_cpp_context_rpc_server_on_executor_cfg.yaml](./install/linux/bin/cfg/examples_cpp_context_rpc_server_on_executor_cfg.yaml)

运行方式（linux）：
- 在 build 目录运行 `start_examples_cpp_context_rpc_server_on_executor.sh`；

说明：
- 模块 `RpcServerOnExecutorModule` 通过 `CreateServer` 创建 RPC 服务端，使用 `ServeOn` 将回调绑定到 `work_thread_pool` 执行器；
- 配置中同时加载 `pb_rpc_client_pkg` 以启用 `NormalRpcCoClientModule` 作为客户端；
- RPC 后端使用 `local`，在同进程内完成调用与响应。


## context rpc server inline

演示内容：
- **在 Context 中创建 RPC 服务端，回调以内联方式执行**；
- **实现 `ExampleService` 的 `GetBarData` 和 `GetFooData` 两个 RPC 方法**；
- **使用 local 类型的 RPC 后端**；
- **与 pb_rpc 示例中的客户端模块协同运行**（`NormalRpcCoClientModule`）；
- **以 Pkg 方式集成并启动**。

核心代码：
- [rpc_server_inline_module.cc](./module/rpc_server_inline_module/rpc_server_inline_module.cc)
- [rpc_server_inline_module.h](./module/rpc_server_inline_module/rpc_server_inline_module.h)
- [pkg_main.cc](./pkg/pkg_main.cc)
- 相关协议定义：[rpc.proto](../../../protocols/pb/example/rpc.proto)

配置文件：
- [examples_cpp_context_rpc_server_inline_cfg.yaml](./install/linux/bin/cfg/examples_cpp_context_rpc_server_inline_cfg.yaml)

运行方式（linux）：
- 在 build 目录运行 `start_examples_cpp_context_rpc_server_inline.sh`；

说明：
- 模块 `RpcServerInlineModule` 通过 `CreateServer` 创建 RPC 服务端，使用 `ServeInline` 进行内联处理；
- 配置中同时加载 `pb_rpc_client_pkg` 以启用 `NormalRpcCoClientModule` 作为客户端；
- RPC 后端使用 `local`，回调在调用线程中直接执行。


## context rpc client

演示内容：
- **在 Context 中创建 RPC 客户端，发起异步协程调用**；
- **调用 `ExampleService` 的 `GetFooData` RPC 方法**；
- **使用 local 类型的 RPC 后端**；
- **与 pb_rpc 示例中的服务端模块协同运行**（`NormalRpcCoServerModule`）；
- **以 Pkg 方式集成并启动**。

核心代码：
- [rpc_client_module.cc](./module/rpc_client_module/rpc_client_module.cc)
- [rpc_client_module.h](./module/rpc_client_module/rpc_client_module.h)
- [pkg_main.cc](./pkg/pkg_main.cc)
- 相关协议定义：[rpc.proto](../../../protocols/pb/example/rpc.proto)

配置文件：
- [examples_cpp_context_rpc_client_cfg.yaml](./install/linux/bin/cfg/examples_cpp_context_rpc_client_cfg.yaml)

运行方式（linux）：
- 在 build 目录运行 `start_examples_cpp_context_rpc_client.sh`；

说明：
- 模块 `RpcClientModule` 通过 `CreateClient` 创建 RPC 客户端，在 `time_schedule_executor` 上循环发起异步调用；
- 配置中加载 `pb_rpc_server_pkg` 以启用 `NormalRpcCoServerModule` 作为服务端；
- RPC 后端使用 `local`，调用频率由配置项 `rpc_frq` 控制。


备注：
- 上述 Channel 示例均依赖 protobuf 与本地 channel 后端，启动脚本会从 `./cfg/*.yaml` 读取主题与执行器配置；
- 上述 RPC 示例均依赖 protobuf 与本地 RPC 后端，启动脚本会从 `./cfg/*.yaml` 读取 RPC 配置与执行器配置；
- 编译启用示例后，pb_chn 示例的发布/订阅 Pkg（`libpb_chn_pub_pkg.so`、`libpb_chn_sub_pkg.so`）以及 pb_rpc 示例的客户端/服务端 Pkg（`libpb_rpc_client_pkg.so`、`libpb_rpc_server_pkg.so`）会一并构建，供本目录示例复用。