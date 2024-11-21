# proxy plugin examples

一个基于 **proxy_plugin** 的代理转发示例，演示内容包括：
- 如何在启动时加载 **proxy_plugin**；
- 如何配置代理转发；
- 如何配置代理转发后的 topic；

核心代码：
- [event.proto](../../../protocols/example/event.proto)
- [normal_publisher_module.cc](../../cpp/pb_chn/module/normal_publisher_module/normal_publisher_module.cc)
- [type_support_pkg_main.cc](./example_event_ts_pkg/type_support_pkg_main.cc)


配置文件：
- [examples_plugins_proxy_plugin_cfg.yaml](./install/linux/bin/cfg/examples_plugins_proxy_plugin_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF`、`AIMRT_BUILD_WITH_ROS2`、`AIMRT_BUILD_WITH_PROXY_PLUGIN`、`AIMRT_BUILD_WITH_ZENOH_PLUGIN`、`AIMRT_BUILD_WITH_ROS2_PLUGIN` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_plugins_proxy_plugin.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了 `NormalPublisherModule`，会基于 `work_thread_pool` 执行器，以配置的频率，通过 `Publisher` 接口，发布 `ExampleEventMsg` 消息；
- 此示例加载了**proxy_plugin**，并配置了代理转发，将 `test_topic` 代理转发到 `test_topic_zenoh` 和 `test_topic_ros2` 两个 topic上；
- 此实例为要转发的 `test_topic` 配置了 `local` 后端，`test_topic_zenoh` 配置了 `zenoh` 后端，`test_topic_ros2` 配置了 `ros2` 后端；
