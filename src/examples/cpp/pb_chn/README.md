# protobuf channel examples


## protobuf channel

一个最基本的、基于 protobuf 协议与 local 后端的 channel 示例，演示内容包括：
- 如何使用 protobuf 协议作为 channel 传输协议；
- 如何基于 Module 方式使用 Executor、Channel publish 和 subscribe 功能；
- 如何使用 local 类型的 channel 后端；
- 如何以 Pkg 模式集成 Module 并启动；


核心代码：
- [event.proto](../../../protocols/pb/example/event.proto)
- [normal_publisher_module.cc](./module/normal_publisher_module/normal_publisher_module.cc)
- [normal_subscriber_module.cc](./module/normal_subscriber_module/normal_subscriber_module.cc)
- [pb_chn_pub_pkg/pkg_main.cc](./pkg/pb_chn_pub_pkg/pkg_main.cc)
- [pb_chn_sub_pkg/pkg_main.cc](./pkg/pb_chn_sub_pkg/pkg_main.cc)


配置文件：
- [examples_cpp_pb_chn_cfg.yaml](./install/linux/bin/cfg/examples_cpp_pb_chn_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_pb_chn.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalPublisherModule`：会基于 `work_thread_pool` 执行器，以配置的频率、向配置的 topic 中发布 `ExampleEventMsg` 类型的消息；
  - `NormalSubscriberModule`：会订阅配置的 topic 下的 `ExampleEventMsg` 类型的消息；
- 此示例将 `NormalPublisherModule` 和 `NormalSubscriberModule` 分别集成到 `pb_chn_pub_pkg` 和 `pb_chn_sub_pkg` 两个 Pkg 中，并在配置文件中加载这两个 Pkg 到一个 AimRT 进程中；
- 此示例使用 local 类型的 channel 后端进行通信；



## protobuf channel single pkg


一个最基本的、基于 protobuf 协议与 local 后端的 channel 示例，演示内容包括：
- 如何使用 protobuf 协议作为 channel 传输协议；
- 如何基于 Module 方式使用 Executor、Channel publish 和 subscribe 功能；
- 如何使用 local 类型的 channel 后端；
- 如何以 Pkg 模式集成 Module 并启动；


核心代码：
- [event.proto](../../../protocols/pb/example/event.proto)
- [normal_publisher_module.cc](./module/normal_publisher_module/normal_publisher_module.cc)
- [normal_subscriber_module.cc](./module/normal_subscriber_module/normal_subscriber_module.cc)
- [pb_chn_pkg/pkg_main.cc](./pkg/pb_chn_pkg/pkg_main.cc)


配置文件：
- [examples_cpp_pb_chn_single_pkg_cfg.yaml](./install/linux/bin/cfg/examples_cpp_pb_chn_single_pkg_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_pb_chn_single_pkg.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例与 **protobuf channel** 示例基本一致，唯一的区别是将 `NormalPublisherModule` 和 `NormalSubscriberModule` 集成到 `pb_chn_pkg` 一个 Pkg 中；


## protobuf channel publisher app


一个基于 protobuf 协议与 local 后端的 channel 示例，演示内容包括：
- 如何使用 protobuf 协议作为 channel 传输协议；
- 如何基于 App 模式创建模块的方式使用 Channel publish 功能；
- 如何基于 Module 方式使用 Channel subscribe 功能；
- 如何使用 local 类型的 channel 后端；
- 如何以 App 模式启动；


核心代码：
- [event.proto](../../../protocols/pb/example/event.proto)
- [normal_publisher_app/main.cc](./app/normal_publisher_app/main.cc)
- [normal_subscriber_module.cc](./module/normal_subscriber_module/normal_subscriber_module.cc)
- [pb_chn_sub_pkg/pkg_main.cc](./pkg/pb_chn_sub_pkg/pkg_main.cc)


配置文件：
- [examples_cpp_pb_chn_publisher_app_cfg.yaml](./install/linux/bin/cfg/examples_cpp_pb_chn_publisher_app_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_pb_chn_publisher_app.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例在 App 模式下，直接创建 `NormalPublisherModule` 模块，获取 CoreRef 句柄，以配置的频率、向配置的 topic 中发布 `ExampleEventMsg` 类型的消息；
- 此示例以继承 `ModuleBase` 的方式创建了`NormalSubscriberModule`模块，会订阅配置的 topic 下的 `ExampleEventMsg` 类型的消息，并集成到了 `pb_chn_sub_pkg` 中，在启动时加载；
- 此示例使用 local 类型的 channel 后端进行通信；
- 此示例以 App 模式启动；


## protobuf channel subscriber app


一个基于 protobuf 协议与 local 后端的 channel 示例，演示内容包括：
- 如何使用 protobuf 协议作为 channel 传输协议；
- 如何基于 App 模式创建模块的方式使用 Channel subscribe 功能；
- 如何基于 Module 方式使用 Channel publish 功能；
- 如何使用 local 类型的 channel 后端；
- 如何以 App 模式启动；


核心代码：
- [event.proto](../../../protocols/pb/example/event.proto)
- [normal_pb_chn_subscriber_app/main.cc](./app/normal_pb_chn_subscriber_app/main.cc)
- [normal_publisher_module.cc](./module/normal_publisher_module/normal_publisher_module.cc)
- [pb_chn_pub_pkg/pkg_main.cc](./pkg/pb_chn_pub_pkg/pkg_main.cc)



配置文件：
- [examples_cpp_pb_chn_subscriber_app_cfg.yaml](./install/linux/bin/cfg/examples_cpp_pb_chn_subscriber_app_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_pb_chn_subscriber_app.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例在 App 模式下，直接创建 `NormalSubscriberModule` 模块，获取 CoreRef 句柄，会订阅配置的 topic 下的 `ExampleEventMsg` 类型的消息；
- 此示例以继承 `ModuleBase` 的方式创建了`NormalPublisherModule`模块，以配置的频率、向配置的 topic 中发布 `ExampleEventMsg` 类型的消息，并集成到了 `pb_chn_pub_pkg` 中，在启动时加载；
- 此示例使用 local 类型的 channel 后端进行通信；
- 此示例以 App 模式启动；


