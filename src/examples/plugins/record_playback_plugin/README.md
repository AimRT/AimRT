# record playback plugin examples


## imd record

一个基于 **record_playback_plugin** 的录包示例，演示内容包括：
- 如何在启动时加载 **record_playback_plugin**；
- 如何录制指定 topic、msg 类型的数据；
- 如何设置落盘文件的格式为 mcap；

核心代码：
- [event.proto](../../../protocols/pb/example/event.proto)
- [normal_publisher_module.cc](../../cpp/pb_chn/module/normal_publisher_module/normal_publisher_module.cc)
- [normal_subscriber_module.cc](../../cpp/pb_chn/module/normal_subscriber_module/normal_subscriber_module.cc)
- [type_support_pkg_main.cc](./example_event_ts_pkg/type_support_pkg_main.cc)


配置文件：
- [examples_plugins_record_playback_plugin_record_imd_cfg.yaml](./install/linux/bin/cfg/examples_plugins_record_playback_plugin_record_imd_cfg.yaml)



运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF`、`AIMRT_BUILD_WITH_ROS2`、`AIMRT_BUILD_NET_PLUGIN`、`AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_plugins_record_playback_plugin_record_imd_mcap.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalPublisherModule`：会基于 `work_thread_pool` 执行器，以配置的频率、向配置的 topic 中发布 `ExampleEventMsg` 类型的消息；
  - `NormalSubscriberModule`：会订阅配置的 topic 下的 `ExampleEventMsg` 类型的消息；
- 此示例加载了 `example_event_ts_pkg`，其中提供了 `ExampleEventMsg` 类型的 type support 工具，作为录包时的序列化工具；
- 此示例创建了一个 `my_imd_record` 的录包 action，会在进程启动时立即开始录制指定的 topic 下的 msg，录制下来的包存放在进程同目录下的 `bag` 路径下；
- 请注意，录包的原理是向 AimRT 订阅指定的 Topic，因此需要在 channel 配置中为该 topic 设置合适的后端，以保证录包插件能接收到数据；

### ros2 录包
一个基于 **record_playback_plugin** 的录包示例，演示内容包括：
- 如何在启动时加载 **record_playback_plugin**；
- 如何录制 ROS2 消息；

核心代码：
- [RosTestMsg.msg](../../../protocols/ros2/example_ros2/msg/RosTestMsg.msg)
- [normal_publisher_module.cc](../../cpp/ros2_chn/module/normal_publisher_module/normal_publisher_module.cc)
- [normal_subscriber_module.cc](../../cpp/ros2_chn/module/normal_subscriber_module/normal_subscriber_module.cc)
- [type_support_pkg_main.cc](./example_event_ts_pkg/type_support_pkg_main.cc)


配置文件：
- [examples_plugins_record_playback_plugin_record_ros2_imd_cfg.yaml](./install/linux/bin/cfg/examples_plugins_record_playback_plugin_record_ros2_imd_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF`、`AIMRT_BUILD_WITH_ROS2`、`AIMRT_BUILD_NET_PLUGIN`、`AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_plugins_record_playback_plugin_record_ros2_imd.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalPublisherModule`：会基于 `work_thread_pool` 执行器，以配置的频率、向配置的 topic 中发布 `RosTestMsg` 类型的消息；
  - `NormalSubscriberModule`：会订阅配置的 topic 下的 `RosTestMsg` 类型的消息；
- 此示例加载了 `example_event_ts_pkg`，其中提供了 `RosTestMsg` 类型的 type support 工具，作为录包时的序列化工具；
- 此示例创建了一个 `my_imd_record` 的录包 action，会在进程启动时立即开始录制指定的 topic 下的 msg，录制下来的包存放在进程同目录下的 `bag` 路径下；
- 请注意，录包的原理是向 AimRT 订阅指定的 Topic，因此需要在 channel 配置中为该 topic 设置合适的后端，以保证录包插件能接收到数据；


## signal record

一个基于 **record_playback_plugin** 和 **net_plugin** 中 http 后端的通过信号触发录包的示例，演示内容包括：
- 如何在启动时加载 **record_playback_plugin**；
- 如何为 **record_playback_plugin** 提供的控制服务配置 http 后端；
- 如何通过 curl 命令触发式录制指定 topic、msg 类型的数据；


核心代码：
- [event.proto](../../../protocols/pb/example/event.proto)
- [normal_publisher_module.cc](../../cpp/pb_chn/module/normal_publisher_module/normal_publisher_module.cc)
- [normal_subscriber_module.cc](../../cpp/pb_chn/module/normal_subscriber_module/normal_subscriber_module.cc)
- [type_support_pkg_main.cc](./example_event_ts_pkg/type_support_pkg_main.cc)


配置文件：
- [examples_plugins_record_playback_plugin_record_signal_cfg.yaml](./install/linux/bin/cfg/examples_plugins_record_playback_plugin_record_signal_cfg.yaml)



运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF`、`AIMRT_BUILD_WITH_ROS2`、`AIMRT_BUILD_NET_PLUGIN`、`AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_plugins_record_playback_plugin_record_signal.sh`脚本启动进程；
- 启动[tools](./install/linux/bin/tools)下的脚本并观察进程打印出来的日志：
  - 运行[record_playback_plugin_start_record.sh](./install/linux/bin/tools/record_playback_plugin_start_record.sh)脚本开始录制；
  - 运行[record_playback_plugin_stop_record.sh](./install/linux/bin/tools/record_playback_plugin_stop_record.sh)脚本结束录制；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalPublisherModule`：会基于 `work_thread_pool` 执行器，以配置的频率、向配置的 topic 中发布 `ExampleEventMsg` 类型的消息；
  - `NormalSubscriberModule`：会订阅配置的 topic 下的 `ExampleEventMsg` 类型的消息；
- 此示例加载了 `example_event_ts_pkg`，其中提供了 `ExampleEventMsg` 类型的 type support 工具，作为录包时的序列化工具；
- 此示例创建了一个 `my_signal_record` 的录包 action，会在接收到开始录制的 rpc 请求后录制指定的 topic 下的 msg，在接收到停止录制的 rpc 请求后结束录制，录制下来的包存放在进程同目录下的 `bag` 路径下；
- 请注意，录包的原理是向 AimRT 订阅指定的 Topic，因此需要在 channel 配置中为该 topic 设置合适的后端，以保证录包插件能接收到数据；



## imd playback

一个基于 **record_playback_plugin** 的播包示例，演示内容包括：
- 如何在启动时加载 **record_playback_plugin**；
- 如何播放指定 topic、msg 类型的数据；


核心代码：
- [event.proto](../../../protocols/pb/example/event.proto)
- [normal_subscriber_module.cc](../../cpp/pb_chn/module/normal_subscriber_module/normal_subscriber_module.cc)
- [type_support_pkg_main.cc](./example_event_ts_pkg/type_support_pkg_main.cc)


配置文件：
- [examples_plugins_record_playback_plugin_playback_imd_cfg.yaml](./install/linux/bin/cfg/examples_plugins_record_playback_plugin_playback_imd_cfg.yaml)



运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF`、`AIMRT_BUILD_WITH_ROS2`、`AIMRT_BUILD_NET_PLUGIN`、`AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN` 选项编译 AimRT；
- 修改配置文件中 `bag_path` 项，改为希望播放的包的地址；
- 直接运行 build 目录下`start_examples_plugins_record_playback_plugin_playback_imd.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了 `NormalSubscriberModule`，会订阅配置的 topic 下的 `ExampleEventMsg` 类型的消息；
- 此示例加载了 `example_event_ts_pkg`，其中提供了 `ExampleEventMsg` 类型的 type support 工具，作为播放时的反序列化工具；
- 此示例创建了一个 `my_imd_playback` 的播包 action，会在进程启动时加载数据包，并立即开始播放其中指定的 topic 下的 msg；
- 请注意，播包的原理是向 AimRT 发布指定的 Topic，因此需要在 channel 配置中为该 topic 设置合适的后端，以保证数据能够通过指定的后端发布出去；


## signal playback


一个基于 **record_playback_plugin** 和 **net_plugin** 中 http 后端的通过信号触发播包的示例，演示内容包括：
- 如何在启动时加载 **record_playback_plugin**；
- 如何为 **record_playback_plugin** 提供的控制服务配置 http 后端；
- 如何通过 curl 命令触发式播放指定 topic、msg 类型的数据；


核心代码：
- [event.proto](../../../protocols/pb/example/event.proto)
- [normal_subscriber_module.cc](../../cpp/pb_chn/module/normal_subscriber_module/normal_subscriber_module.cc)
- [type_support_pkg_main.cc](./example_event_ts_pkg/type_support_pkg_main.cc)


配置文件：
- [examples_plugins_record_playback_plugin_playback_signal_cfg.yaml](./install/linux/bin/cfg/examples_plugins_record_playback_plugin_playback_signal_cfg.yaml)



运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF`、`AIMRT_BUILD_WITH_ROS2`、`AIMRT_BUILD_NET_PLUGIN`、`AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN` 选项编译 AimRT；
- 修改配置文件中 `bag_path` 项，改为希望播放的包的地址；
- 直接运行 build 目录下`start_examples_plugins_record_playback_plugin_playback_signal.sh`脚本启动进程；
- 启动[tools](./install/linux/bin/tools)下的脚本并观察进程打印出来的日志：
  - 运行[record_playback_plugin_start_playback.sh](./install/linux/bin/tools/record_playback_plugin_start_playback.sh)脚本开始播放；
  - 运行[record_playback_plugin_stop_playback.sh](./install/linux/bin/tools/record_playback_plugin_stop_playback.sh)脚本结束播放；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了 `NormalSubscriberModule`，会订阅配置的 topic 下的 `ExampleEventMsg` 类型的消息；
- 此示例加载了 `example_event_ts_pkg`，其中提供了 `ExampleEventMsg` 类型的 type support 工具，作为播放时的反序列化工具；
- 此示例创建了一个 `my_signal_playback` 的播包 action，会在进程启动时加载数据包，并在接收到开始播放的 rpc 请求后播放指定的 topic 下的 msg，在接收到停止播放的 rpc 请求后结束播放，
- 请注意，播包的原理是向 AimRT 发布指定的 Topic，因此需要在 channel 配置中为该 topic 设置合适的后端，以保证数据能够通过指定的后端发布出去；




