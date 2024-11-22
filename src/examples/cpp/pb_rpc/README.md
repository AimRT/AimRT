# protobuf rpc examples


## protobuf rpc sync


一个基于 protobuf 协议、同步型接口与 local 后端的 rpc 示例，演示内容包括：
- 如何使用 protobuf 协议作为 rpc 服务协议；
- 如何基于 Module 方式使用 Executor、同步型 Rpc client 和 server 接口；
- 如何使用 local 类型的 rpc 后端；
- 如何以 Pkg 模式集成 Module 并启动；



核心代码：
- [rpc.proto](../../../protocols/example/rpc.proto)
- [normal_rpc_sync_client_module.cc](./module/normal_rpc_sync_client_module/normal_rpc_sync_client_module.cc)
- [normal_rpc_sync_server_module.cc](./module/normal_rpc_sync_server_module/normal_rpc_sync_server_module.cc)
- [service.cc](./module/normal_rpc_sync_server_module/service.cc)
- [pb_rpc_client_pkg/pkg_main.cc](./pkg/pb_rpc_client_pkg/pkg_main.cc)
- [pb_rpc_server_pkg/pkg_main.cc](./pkg/pb_rpc_server_pkg/pkg_main.cc)


配置文件：
- [examples_cpp_pb_rpc_sync_cfg.yaml](./install/linux/bin/cfg/examples_cpp_pb_rpc_sync_cfg.yaml)



运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_pb_rpc_sync.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalRpcSyncClientModule`：会基于 `work_thread_pool` 执行器，以配置的频率，通过同步 Client 接口，向 `ExampleService` 发起 RPC 请求；
  - `NormalRpcSyncServerModule`：会注册 `ExampleService` 服务端，通过同步 Server 接口，提供 echo 功能；
- 此示例将 `NormalRpcSyncClientModule` 和 `NormalRpcSyncServerModule` 分别集成到 `pb_rpc_client_pkg` 和 `pb_rpc_server_pkg` 两个 Pkg 中，并在配置文件中加载这两个 Pkg 到一个 AimRT 进程中；
- 此示例使用 local 类型的 rpc 后端进行通信，并配置了 `timeout_handle` 执行器作为超时执行器；



## protobuf rpc async


一个基于 protobuf 协议、异步型接口与 local 后端的 rpc 示例，演示内容包括：
- 如何使用 protobuf 协议作为 rpc 服务协议；
- 如何基于 Module 方式使用 Executor、异步型 Rpc client 和 server 接口；
- 如何使用 local 类型的 rpc 后端；
- 如何以 Pkg 模式集成 Module 并启动；



核心代码：
- [rpc.proto](../../../protocols/example/rpc.proto)
- [normal_rpc_async_client_module.cc](./module/normal_rpc_async_client_module/normal_rpc_async_client_module.cc)
- [normal_rpc_async_server_module.cc](./module/normal_rpc_async_server_module/normal_rpc_async_server_module.cc)
- [service.cc](./module/normal_rpc_async_server_module/service.cc)
- [pb_rpc_client_pkg/pkg_main.cc](./pkg/pb_rpc_client_pkg/pkg_main.cc)
- [pb_rpc_server_pkg/pkg_main.cc](./pkg/pb_rpc_server_pkg/pkg_main.cc)


配置文件：
- [examples_cpp_pb_rpc_async_cfg.yaml](./install/linux/bin/cfg/examples_cpp_pb_rpc_async_cfg.yaml)



运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_pb_rpc_async.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalRpcAsyncClientModule`：会基于 `work_thread_pool` 执行器，以配置的频率，通过异步 Client 接口，向 `ExampleService` 发起 RPC 请求；
  - `NormalRpcAsyncServerModule`：会注册 `ExampleService` 服务端，通过异步 Server 接口，提供 echo 功能；
- 此示例将 `NormalRpcAsyncClientModule` 和 `NormalRpcAsyncServerModule` 分别集成到 `pb_rpc_client_pkg` 和 `pb_rpc_server_pkg` 两个 Pkg 中，并在配置文件中加载这两个 Pkg 到一个 AimRT 进程中；
- 此示例使用 local 类型的 rpc 后端进行通信，并配置了 `timeout_handle` 执行器作为超时执行器；



## protobuf rpc future

一个基于 protobuf 协议、future 型接口与 local 后端的 rpc 示例，演示内容包括：
- 如何使用 protobuf 协议作为 rpc 服务协议；
- 如何基于 Module 方式使用 Executor、future 型 Rpc client 接口和同步型 Rpc server 接口；
- 如何使用 local 类型的 rpc 后端；
- 如何以 Pkg 模式集成 Module 并启动；



核心代码：
- [rpc.proto](../../../protocols/example/rpc.proto)
- [normal_rpc_future_client_module.cc](./module/normal_rpc_future_client_module/normal_rpc_future_client_module.cc)
- [normal_rpc_sync_server_module.cc](./module/normal_rpc_sync_server_module/normal_rpc_sync_server_module.cc)
- [service.cc](./module/normal_rpc_sync_server_module/service.cc)
- [pb_rpc_client_pkg/pkg_main.cc](./pkg/pb_rpc_client_pkg/pkg_main.cc)
- [pb_rpc_server_pkg/pkg_main.cc](./pkg/pb_rpc_server_pkg/pkg_main.cc)


配置文件：
- [examples_cpp_pb_rpc_future_cfg.yaml](./install/linux/bin/cfg/examples_cpp_pb_rpc_future_cfg.yaml)



运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_pb_rpc_future.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalRpcFutureClientModule`：会基于 `work_thread_pool` 执行器，以配置的频率，通过 future 型 Client 接口，向 `ExampleService` 发起 RPC 请求；
  - `NormalRpcSyncServerModule`：会注册 `ExampleService` 服务端，通过同步 Server 接口，提供 echo 功能；
- 此示例将 `NormalRpcFutureClientModule` 和 `NormalRpcSyncServerModule` 分别集成到 `pb_rpc_client_pkg` 和 `pb_rpc_server_pkg` 两个 Pkg 中，并在配置文件中加载这两个 Pkg 到一个 AimRT 进程中；
- 此示例使用 local 类型的 rpc 后端进行通信，并配置了 `timeout_handle` 执行器作为超时执行器；



## protobuf rpc co


一个基于 protobuf 协议、协程型接口与 local 后端的 rpc 示例，演示内容包括：
- 如何使用 protobuf 协议作为 rpc 服务协议；
- 如何基于 Module 方式使用 Executor、协程型 Rpc client 和 server 接口；
- 如何使用协程形式的 Rpc filter 功能；
- 如何使用 local 类型的 rpc 后端；
- 如何以 Pkg 模式集成 Module 并启动；



核心代码：
- [rpc.proto](../../../protocols/example/rpc.proto)
- [normal_rpc_co_client_module.cc](./module/normal_rpc_co_client_module/normal_rpc_co_client_module.cc)
- [normal_rpc_co_server_module.cc](./module/normal_rpc_co_server_module/normal_rpc_co_server_module.cc)
- [service.cc](./module/normal_rpc_co_server_module/service.cc)
- [pb_rpc_client_pkg/pkg_main.cc](./pkg/pb_rpc_client_pkg/pkg_main.cc)
- [pb_rpc_server_pkg/pkg_main.cc](./pkg/pb_rpc_server_pkg/pkg_main.cc)


配置文件：
- [examples_cpp_pb_rpc_co_cfg.yaml](./install/linux/bin/cfg/examples_cpp_pb_rpc_co_cfg.yaml)



运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_pb_rpc_co.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalRpcCoClientModule`：会基于 `work_thread_pool` 执行器，以配置的频率，通过协程 Client 接口，向 `ExampleService` 发起 RPC 请求；
  - `NormalRpcCoServerModule`：会注册 `ExampleService` 服务端，通过协程 Server 接口，提供 echo 功能；
- 此示例在 Rpc Client 端和 Server 端分别注册了两个 Filter 用于打印请求日志和计算耗时；
- 此示例将 `NormalRpcCoClientModule` 和 `NormalRpcCoServerModule` 分别集成到 `pb_rpc_client_pkg` 和 `pb_rpc_server_pkg` 两个 Pkg 中，并在配置文件中加载这两个 Pkg 到一个 AimRT 进程中；
- 此示例使用 local 类型的 rpc 后端进行通信，并配置了 `timeout_handle` 执行器作为超时执行器；



## protobuf rpc single pkg


一个基于 protobuf 协议、协程型接口与 local 后端的 rpc 示例，演示内容包括：
- 如何使用 protobuf 协议作为 rpc 服务协议；
- 如何基于 Module 方式使用 Executor、协程型 Rpc client 和 server 接口；
- 如何使用协程形式的 Rpc filter 功能；
- 如何使用 local 类型的 rpc 后端；
- 如何以 Pkg 模式集成 Module 并启动；



核心代码：
- [rpc.proto](../../../protocols/example/rpc.proto)
- [normal_rpc_co_client_module.cc](./module/normal_rpc_co_client_module/normal_rpc_co_client_module.cc)
- [normal_rpc_co_server_module.cc](./module/normal_rpc_co_server_module/normal_rpc_co_server_module.cc)
- [service.cc](./module/normal_rpc_co_server_module/service.cc)
- [pb_rpc_pkg/pkg_main.cc](./pkg/pb_rpc_pkg/pkg_main.cc)



配置文件：
- [examples_cpp_pb_rpc_single_pkg_cfg.yaml](./install/linux/bin/cfg/examples_cpp_pb_rpc_single_pkg_cfg.yaml)



运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_pb_rpc_single_pkg.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例与 **protobuf rpc co** 示例基本一致，唯一的区别是将 `NormalRpcCoClientModule` 和 `NormalRpcCoServerModule` 集成到 `pb_rpc_pkg` 一个 Pkg 中；

## protobuf proxy rpc co


一个基于 protobuf 协议、协程型接口与 local 后端的 rpc 示例，演示内容包括：
- 如何使用 protobuf 协议作为 rpc 服务协议；
- 如何基于 Module 方式使用 Executor、协程型 Rpc client 和 server 接口；
- 如何使用协程形式的 Rpc filter 功能；
- 如何使用 local 类型的 rpc 后端；
- 如何以 Pkg 模式集成 Module 并启动；
- 如何构建链式 rpc 调用；



核心代码：
- [rpc.proto](../../../protocols/example/rpc.proto)
- [proxy_rpc_co_module.cc](./module/proxy_rpc_co_module/proxy_rpc_co_module.cc)
- [service.cc](./module/proxy_rpc_co_module/service.cc)
- [pb_rpc_client_pkg/pkg_main.cc](./pkg/pb_rpc_client_pkg/pkg_main.cc)
- [pb_rpc_server_pkg/pkg_main.cc](./pkg/pb_rpc_server_pkg/pkg_main.cc)


配置文件：
- [examples_cpp_pb_proxy_rpc_co_cfg.yaml](./install/linux/bin/cfg/examples_cpp_pb_proxy_rpc_co_cfg.yaml)



运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_pb_rpc_co.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了以下三个模块， 以构成 RPC 的链式调用：
  - `NormalRpcCoClientModule`：会基于 `work_thread_pool` 执行器，以配置的频率，通过协程 Client 接口，向 `ExampleService_1` 发起 RPC 请求；
  - `ProxyRpcCoModule`：会注册 `ExampleService_1` 服务端，通过协程 Server 接口，提供 echo 功能, 并向 `ExampleService_2` 发起 RPC 请求；
  - `NormalRpcCoServerModule`：会注册 `ExampleService_2` 服务端，通过协程 Server 接口，提供 echo 功能；
- 此示例将 `NormalRpcCoClientModule` 集成到 `pb_rpc_client_pkg`, 将 `ProxyRpcCoModule` 和 `NormalRpcCoServerModule` 集成到 `pb_rpc_server_pkg`  Pkg 中，并在配置文件中加载这两个 Pkg 到一个 AimRT 进程中；
- 此示例在 Rpc Client 端和 Server 端分别注册了两个 Filter 用于打印请求日志和计算耗时；
- 此示例使用 local 类型的 rpc 后端进行通信，并配置了 `timeout_handle` 执行器作为超时执行器；
