# net plugin examples

## http protobuf rpc

一个基于 protobuf 协议、协程型接口与 http 后端的 rpc 示例，演示内容包括：
- 如何在配置文件中加载**net_plugin**；
- 如何使用 http 类型的 rpc 后端；
- 如何使用 curl 命令直接向服务端发起请求；

核心代码：
- [rpc.proto](../../../protocols/example/rpc.proto)
- [normal_rpc_co_client_module.cc](../../cpp/protobuf_rpc/module/normal_rpc_co_client_module/normal_rpc_co_client_module.cc)
- [normal_rpc_co_server_module.cc](../../cpp/protobuf_rpc/module/normal_rpc_co_server_module/normal_rpc_co_server_module.cc)
- [service.cc](../../cpp/protobuf_rpc/module/normal_rpc_co_server_module/service.cc)


配置文件：
- [examples_plugins_net_plugin_protobuf_rpc_http_server_cfg.yaml](./install/linux/bin/cfg/examples_plugins_net_plugin_protobuf_rpc_http_server_cfg.yaml)
- [examples_plugins_net_plugin_protobuf_rpc_http_client_cfg.yaml](./install/linux/bin/cfg/examples_plugins_net_plugin_protobuf_rpc_http_client_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_NET_PLUGIN` 选项编译 AimRT；
- 在终端运行 build 目录下`start_examples_plugins_net_plugin_protobuf_rpc_http_server.sh`脚本启动服务端（srv 进程）；
- 开启新的终端运行 build 目录下`start_examples_plugins_net_plugin_protobuf_rpc_http_client.sh`脚本启动客户端（cli 进程）；
- 在服务端开启的状态下，运行[protobuf_rpc_http_client_tool.sh](./install/linux/bin/tools/protobuf_rpc_http_client_tool.sh)脚本向服务端直接发起 RPC 请求；
- 分别在两个终端键入`ctrl-c`停止对应进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalRpcCoClientModule`：会基于 `work_thread_pool` 执行器，以配置的频率，通过协程 Client 接口，向 `ExampleService` 发起 RPC 请求；
  - `NormalRpcCoServerModule`：会注册 `ExampleService` 服务端，通过协程 Server 接口，提供 echo 功能；
- 此示例在 Rpc Client 端和 Server 端分别注册了两个 Filter 用于打印请求日志和计算耗时；
- 此示例将 `NormalRpcCoClientModule` 和 `NormalRpcCoServerModule` 分别集成到 `protobuf_rpc_client_pkg` 和 `protobuf_rpc_server_pkg` 两个 Pkg 中，并在两个配置文件中分别加载对应的 Pkg 到 srv 和 cli 进程中；
- 此示例加载了**net_plugin**，并使用 http 类型的 rpc 后端进行通信，配置 `127.0.0.1:50080` 作为服务端地址；
- 此示例演示了如何通过 curl 命令直接向服务端发起请求；
 

## http ros2 rpc

一个基于 ros2 srv 协议、协程型接口与 http 后端的 rpc 示例，演示内容包括：
- 如何在配置文件中加载**net_plugin**；
- 如何使用 http 类型的 rpc 后端；
- 如何使用 curl 命令直接向服务端发起请求；


核心代码：
- [example_ros2/srv/RosTestRpc.srv](../../../protocols/example_ros2/srv/RosTestRpc.srv)
- [normal_rpc_co_client_module.cc](../../cpp/ros2_rpc/module/normal_rpc_co_client_module/normal_rpc_co_client_module.cc)
- [normal_rpc_co_server_module.cc](../../cpp/ros2_rpc/module/normal_rpc_co_server_module/normal_rpc_co_server_module.cc)
- [service.cc](../../cpp/ros2_rpc/module/normal_rpc_co_server_module/service.cc)


配置文件：
- [examples_plugins_net_plugin_ros2_rpc_http_server_cfg.yaml](./install/linux/bin/cfg/examples_plugins_net_plugin_ros2_rpc_http_server_cfg.yaml)
- [examples_plugins_net_plugin_ros2_rpc_http_client_cfg.yaml](./install/linux/bin/cfg/examples_plugins_net_plugin_ros2_rpc_http_client_cfg.yaml)




运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_NET_PLUGIN` 选项编译 AimRT；
- 在终端运行 build 目录下`start_examples_plugins_ros2_plugin_ros2_rpc_server.sh`脚本启动服务端（srv 进程）；
- 开启新的终端运行 build 目录下`start_examples_plugins_ros2_plugin_ros2_rpc_client.sh`脚本启动客户端（cli 进程）；
- 在服务端开启的状态下，运行[ros2_rpc_http_client_tool.sh](./install/linux/bin/tools/ros2_rpc_http_client_tool.sh)脚本向服务端直接发起 RPC 请求；
- 分别在两个终端键入`ctrl-c`停止对应进程；


说明：
- 本示例与 **http protobuf rpc** 示例基本一致，除了业务层使用的是 ros2 srv 形式的协议；



## http protobuf channel

一个基于 protobuf 协议与 http 后端的 channel 示例，演示内容包括：
- 如何在配置文件中加载**net_plugin**；
- 如何使用 http 类型的 channel 后端；
- 如何使用 curl 命令直接向订阅端发布数据；

核心代码：
- [event.proto](../../../protocols/example/event.proto)
- [normal_publisher_module.cc](../../cpp/protobuf_channel/module/normal_publisher_module/normal_publisher_module.cc)
- [normal_subscriber_module.cc](../../cpp/protobuf_channel/module/normal_subscriber_module/normal_subscriber_module.cc)


配置文件：
- [examples_plugins_net_plugin_protobuf_channel_http_pub_cfg.yaml](./install/linux/bin/cfg/examples_plugins_net_plugin_protobuf_channel_http_pub_cfg.yaml)
- [examples_plugins_net_plugin_protobuf_channel_http_sub_cfg.yaml](./install/linux/bin/cfg/examples_plugins_net_plugin_protobuf_channel_http_sub_cfg.yaml)

运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_NET_PLUGIN` 选项编译 AimRT；
- 在终端运行 build 目录下`start_examples_plugins_net_plugin_protobuf_channel_http_sub.sh`脚本启动订阅端（sub 进程）；
- 再开启一个新的终端窗口运行`start_examples_plugins_net_plugin_protobuf_channel_http_pub.sh`脚本启动发布端（pub 进程）；
- 在订阅端开启的状态下，运行[protobuf_channel_http_pub_tool.sh](./install/linux/bin/tools/protobuf_channel_http_pub_tool.sh)脚本向订阅端直接发布数据；
- 分别在两个终端键入`ctrl-c`停止对应进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalPublisherModule`：会基于 `work_thread_pool` 执行器，以配置的频率、向配置的 topic 中发布 `ExampleEventMsg` 类型的消息；
  - `NormalSubscriberModule`：会订阅配置的 topic 下的 `ExampleEventMsg` 类型的消息；
- 此示例将 `NormalPublisherModule` 和 `NormalSubscriberModule` 分别集成到 `protobuf_channel_pub_pkg` 和 `protobuf_channel_sub_pkg` 两个 Pkg 中，并在两个配置文件中分别加载对应的 Pkg 到 pub 和 sub 进程中；
- 此示例加载了**net_plugin**，并使用 http 类型的 channel 后端进行通信，配置 `127.0.0.1:50080` 作为订阅端地址；
- 此示例演示了如何通过 curl 命令直接向订阅端直接发布数据；


## http ros2 channel

一个基于 ros2 msg 协议与 http 后端的 channel 示例，演示内容包括：
- 如何在配置文件中加载**net_plugin**；
- 如何使用 http 类型的 channel 后端；
- 如何使用 curl 命令直接向订阅端发布数据；


核心代码：
- [example_ros2/msg/RosTestMsg.msg](../../../protocols/example_ros2/msg/RosTestMsg.msg)
- [normal_publisher_module.cc](../../cpp/ros2_channel/module/normal_publisher_module/normal_publisher_module.cc)
- [normal_subscriber_module.cc](../../cpp/ros2_channel/module/normal_subscriber_module/normal_subscriber_module.cc)


配置文件：
- [examples_plugins_net_plugin_ros2_channel_http_pub_cfg.yaml](./install/linux/bin/cfg/examples_plugins_net_plugin_ros2_channel_http_pub_cfg.yaml)
- [examples_plugins_net_plugin_ros2_channel_http_sub_cfg.yaml](./install/linux/bin/cfg/examples_plugins_net_plugin_ros2_channel_http_sub_cfg.yaml)

运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_NET_PLUGIN` 选项编译 AimRT；
- 在终端运行 build 目录下`start_examples_plugins_net_plugin_ros2_channel_http_sub.sh`脚本启动订阅端（sub 进程）；
- 再开启一个新的终端窗口运行`start_examples_plugins_net_plugin_ros2_channel_http_pub.sh`脚本启动发布端（pub 进程）；
- 在订阅端开启的状态下，运行[ros2_channel_http_pub_tool.sh](./install/linux/bin/tools/ros2_channel_http_pub_tool.sh)脚本向订阅端直接发布数据；
- 分别在两个终端键入`ctrl-c`停止对应进程；


说明：
- 本示例与 **http protobuf channel** 示例基本一致，除了业务层使用的是 ros2 msg 形式的协议；



## tcp protobuf channel

一个基于 protobuf 协议与 tcp 后端的 channel 示例，演示内容包括：
- 如何在配置文件中加载**net_plugin**；
- 如何使用 tcp 类型的 channel 后端；


核心代码：
- [event.proto](../../../protocols/example/event.proto)
- [normal_publisher_module.cc](../../cpp/protobuf_channel/module/normal_publisher_module/normal_publisher_module.cc)
- [normal_subscriber_module.cc](../../cpp/protobuf_channel/module/normal_subscriber_module/normal_subscriber_module.cc)


配置文件：
- [examples_plugins_net_plugin_protobuf_channel_tcp_pub_cfg.yaml](./install/linux/bin/cfg/examples_plugins_net_plugin_protobuf_channel_tcp_pub_cfg.yaml)
- [examples_plugins_net_plugin_protobuf_channel_tcp_sub_cfg.yaml](./install/linux/bin/cfg/examples_plugins_net_plugin_protobuf_channel_tcp_sub_cfg.yaml)

运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_NET_PLUGIN` 选项编译 AimRT；
- 在终端运行 build 目录下`start_examples_plugins_net_plugin_protobuf_channel_tcp_sub.sh`脚本启动订阅端（sub 进程）；
- 再开启一个新的终端窗口运行`start_examples_plugins_net_plugin_protobuf_channel_tcp_pub.sh`脚本启动发布端（pub 进程）；
- 分别在两个终端键入`ctrl-c`停止对应进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalPublisherModule`：会基于 `work_thread_pool` 执行器，以配置的频率、向配置的 topic 中发布 `ExampleEventMsg` 类型的消息；
  - `NormalSubscriberModule`：会订阅配置的 topic 下的 `ExampleEventMsg` 类型的消息；
- 此示例将 `NormalPublisherModule` 和 `NormalSubscriberModule` 分别集成到 `protobuf_channel_pub_pkg` 和 `protobuf_channel_sub_pkg` 两个 Pkg 中，并在两个配置文件中分别加载对应的 Pkg 到 pub 和 sub 进程中；
- 此示例加载了**net_plugin**，并使用 tcp 类型的 channel 后端进行通信，配置 `127.0.0.1:50060` 作为订阅端地址；


## udp protobuf channel

一个基于 protobuf 协议与 udp 后端的 channel 示例，演示内容包括：
- 如何在配置文件中加载**net_plugin**；
- 如何使用 udp 类型的 channel 后端；


核心代码：
- [event.proto](../../../protocols/example/event.proto)
- [normal_publisher_module.cc](../../cpp/protobuf_channel/module/normal_publisher_module/normal_publisher_module.cc)
- [normal_subscriber_module.cc](../../cpp/protobuf_channel/module/normal_subscriber_module/normal_subscriber_module.cc)


配置文件：
- [examples_plugins_net_plugin_protobuf_channel_udp_pub_cfg.yaml](./install/linux/bin/cfg/examples_plugins_net_plugin_protobuf_channel_udp_pub_cfg.yaml)
- [examples_plugins_net_plugin_protobuf_channel_udp_sub_cfg.yaml](./install/linux/bin/cfg/examples_plugins_net_plugin_protobuf_channel_udp_sub_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_NET_PLUGIN` 选项编译 AimRT；
- 在终端运行 build 目录下`start_examples_plugins_net_plugin_protobuf_channel_udp_sub.sh`脚本启动订阅端（sub 进程）；
- 再开启一个新的终端窗口运行`start_examples_plugins_net_plugin_protobuf_channel_udp_pub.sh`脚本启动发布端（pub 进程）；
- 分别在两个终端键入`ctrl-c`停止对应进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalPublisherModule`：会基于 `work_thread_pool` 执行器，以配置的频率、向配置的 topic 中发布 `ExampleEventMsg` 类型的消息；
  - `NormalSubscriberModule`：会订阅配置的 topic 下的 `ExampleEventMsg` 类型的消息；
- 此示例将 `NormalPublisherModule` 和 `NormalSubscriberModule` 分别集成到 `protobuf_channel_pub_pkg` 和 `protobuf_channel_sub_pkg` 两个 Pkg 中，并在两个配置文件中分别加载对应的 Pkg 到 pub 和 sub 进程中；
- 此示例加载了**net_plugin**，并使用 udp 类型的 channel 后端进行通信，配置 `127.0.0.1:50040` 作为订阅端地址；
