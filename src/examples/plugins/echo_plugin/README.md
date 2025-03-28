# echo plugin examples

## echo with pb msg

一个基于 **echo_plugin** 的回显消息示例，演示内容包括：
- 如何在启动时加载 **echo_plugin**；
- 如何回显指定 topic、msg 类型的数据；
- 如何配置回显 pb 消息；


核心代码：
- [event.proto](../../../protocols/pb/example/event.proto)
- [normal_publisher_module.cc](../../cpp/pb_chn/module/normal_publisher_module/normal_publisher_module.cc)
- [type_support_pkg_main.cc](./example_event_ts_pkg/type_support_pkg_main.cc)


配置文件：
- [examples_plugins_echo_plugin_pb_cfg.yaml](./install/linux/bin/cfg/examples_plugins_echo_plugin_pb_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF` 、`AIMRT_BUILD_WITH_ROS2` 、`AIMRT_BUILD_WITH_ECHO_PLUGIN` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_plugins_echo_plugin_pb.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了以下模块：
  - `NormalPublisherModule`：会基于 `work_thread_pool` 执行器，以配置的频率、向配置的 topic 中发布 `ExampleEventMsg` 类型的消息；
- 此示例加载了 `example_event_ts_pkg`，其中提供了 `ExampleEventMsg` 和 `RosTestMsg` 类型的 type support 工具，作为回显时的序列化工具；
- 请注意，echo 插件的原理是向 AimRT 订阅指定的 Topic，因此需要在 channel 配置中为该 topic 设置合适的后端，以保证插件能接收到数据；


## echo with ros2 msg

一个基于 **echo_plugin** 的回显消息示例，演示内容包括：
- 如何在启动时加载 **echo_plugin**；
- 如何回显指定 topic、msg 类型的数据；
- 如何配置回显 ROS2 消息；


核心代码：
- [RosTestMsg.msg](../../../protocols/ros2/example_ros2/msg/RosTestMsg.msg)
- [normal_publisher_module.cc](../../cpp/ros2_chn/module/normal_publisher_module/normal_publisher_module.cc)
- [type_support_pkg_main.cc](./example_event_ts_pkg/type_support_pkg_main.cc)


配置文件：
- [examples_plugins_echo_plugin_ros2_cfg.yaml](./install/linux/bin/cfg/examples_plugins_echo_plugin_ros2_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF`、`AIMRT_BUILD_WITH_ROS2` 、`AIMRT_BUILD_WITH_ECHO_PLUGIN` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_plugins_echo_plugin_ros2.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了以下模块：
  - `NormalPublisherModule`：会基于 `work_thread_pool` 执行器，以配置的频率、向配置的 topic 中发布 `RosTestMsg` 类型的消息；
- 此示例加载了 `example_event_ts_pkg`，其中提供了 `ExampleEventMsg` 和 `RosTestMsg` 类型的 type support 工具，作为回显时的序列化工具；
- 请注意，echo 插件的原理是向 AimRT 订阅指定的 Topic，因此需要在 channel 配置中为该 topic 设置合适的后端，以保证插件能接收到数据；
