# grpc plugin examples

## protobuf rpc

一个基于 protobuf 协议、协程型接口与 grpc 后端的 rpc 示例，演示内容包括：
- 如何在配置文件中加载**grpc_plugin**；
- 如何使用 grpc 类型的 rpc 后端；
- 如何与原生 grpc 进行 protobuf 协议的 rpc 相互调用；


核心代码：
- [rpc.proto](../../../protocols/pb/example/rpc.proto)
- aimrt code：
  - [normal_rpc_co_client_module.cc](../../cpp/pb_rpc/module/normal_rpc_co_client_module/normal_rpc_co_client_module.cc)
  - [normal_rpc_co_server_module.cc](../../cpp/pb_rpc/module/normal_rpc_co_server_module/normal_rpc_co_server_module.cc)
  - [service.cc](../../cpp/pb_rpc/module/normal_rpc_co_server_module/service.cc)
- native grpc code:
  - [client.py](./assistant/client.py)
  - [server.py](./assistant/server.py)


配置文件：
- [examples_plugins_grpc_plugin_pb_rpc_client_cfg.yaml](./install/linux/bin/cfg/examples_plugins_grpc_plugin_pb_rpc_client_cfg.yaml)
- [examples_plugins_grpc_plugin_pb_rpc_server_cfg.yaml](./install/linux/bin/cfg/examples_plugins_grpc_plugin_pb_rpc_server_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF`、`AIMRT_BUILD_GRPC_PLUGIN` 选项编译 AimRT；
- 如果要运行原生 grpc 服务，需要先安装 `grpcio` 和 `grpcio-tools` 库，可以直接通过`pip install grpcio grpcio-tools`安装，然后运行 assistant 目录下的[build_grpc_native_examples.sh](./assistant/build_grpc_native_examples.sh)脚本；
- aimrt client 调用 aimrt server：
  - 编译成功后在终端运行 build 目录下`start_examples_plugins_grpc_plugin_pb_rpc_server.sh`脚本启动服务端（srv 进程）；
  - 开启新的终端运行 build 目录下`start_examples_plugins_grpc_plugin_pb_rpc_client.sh`脚本启动客户端（cli 进程）；
  - 分别在两个终端键入`ctrl-c`停止对应进程；
- aimrt client 调用 native grpc server：
  - 直接运行 assistant 目录下的[start_examples_plugins_grpc_plugin_native_server.sh](./assistant/start_examples_plugins_grpc_plugin_native_server.sh)脚本启动原生 grpc 服务端；
  - 开启新的终端运行 build 目录下`start_examples_plugins_grpc_plugin_pb_rpc_client.sh`脚本启动客户端（cli 进程）；
  - 分别在两个终端键入`ctrl-c`停止对应进程；
- native ros2 client 调用 aimrt server：
  - 编译成功后在终端运行 build 目录下`start_examples_plugins_grpc_plugin_pb_rpc_server.sh`脚本启动服务端（srv 进程）；
  - 直接运行 assistant 目录下的[start_examples_plugins_grpc_plugin_native_client.sh](./assistant/start_examples_plugins_grpc_plugin_native_client.sh)脚本启动原生 grpc 客户端；
  - 分别在两个终端键入`ctrl-c`停止对应进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalRpcCoClientModule`：会基于 `work_thread_pool` 执行器，以配置的频率，通过协程 Client 接口，向 `ExampleService` 发起 RPC 请求；
  - `NormalRpcCoServerModule`：会注册 `ExampleService` 服务端，通过协程 Server 接口，提供 echo 功能；
- 此示例在 Rpc Client 端和 Server 端分别注册了两个 Filter 用于打印请求日志和计算耗时；
- 此示例将 `NormalRpcCoClientModule` 和 `NormalRpcCoServerModule` 分别集成到 `pb_rpc_client_pkg` 和 `pb_rpc_server_pkg` 两个 Pkg 中, 并在两个配置文件中分别加载对应的 Pkg 到 srv 和 cli 进程中；
- 此示例还创建了原生 grpc 客户端和服务端，用于展示如何使用原生 grpc 程序与 aimrt 程序通过 grpc 后端通信；
- 此示例加载了**grpc_plugin**，并使用 grpc 类型的 rpc 后端进行通信；

## ros2 rpc

一个基于 ros2 协议、协程型接口与 grpc 后端的 rpc 示例，演示内容包括：
- 如何在配置文件中加载**grpc_plugin**；
- 如何使用 grpc 类型的 rpc 后端；
- 如何与原生 grpc 进行 ros2 协议的 rpc 相互调用；

核心代码：
- [example_ros2/srv/RosTestRpc.srv](../../../protocols/example_ros2/srv/RosTestRpc.srv)
- aimrt code：
  - [normal_rpc_co_client_module.cc](../../cpp/ros2_rpc/module/normal_rpc_co_client_module/normal_rpc_co_client_module.cc)
  - [normal_rpc_co_server_module.cc](../../cpp/ros2_rpc/module/normal_rpc_co_server_module/normal_rpc_co_server_module.cc)
  - [service.cc](../../cpp/ros2_rpc/module/normal_rpc_co_server_module/service.cc)

配置文件：
- [examples_plugins_grpc_plugin_ros2_rpc_client_cfg.yaml](./install/linux/bin/cfg/examples_plugins_grpc_plugin_ros2_rpc_client_cfg.yaml)
- [examples_plugins_grpc_plugin_ros2_rpc_server_cfg.yaml](./install/linux/bin/cfg/examples_plugins_grpc_plugin_ros2_rpc_server_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_ROS2`、`AIMRT_BUILD_GRPC_PLUGIN` 选项编译 AimRT；
- 编译成功后在终端运行 build 目录下`start_examples_plugins_grpc_plugin_ros2_rpc_server.sh`脚本启动服务端（srv 进程）；
- 开启新的终端运行 build 目录下`start_examples_plugins_grpc_plugin_ros2_rpc_client.sh`脚本启动客户端（cli 进程）；
- 分别在两个终端键入`ctrl-c`停止对应进程；


说明：
- 此示例与 **protobuf rpc** 示例基本一致，除了业务层使用的是 ros2 srv 形式的协议；
