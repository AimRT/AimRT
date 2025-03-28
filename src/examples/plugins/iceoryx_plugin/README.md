# iceoryx plugin examples

## protobuf channel

一个基于 protobuf 协议与 iceoryx 后端的 channel 示例，演示内容包括：
- 如何在配置文件中加载**iceoryx_plugin**；
- 如何使用 iceoryx 类型的 channel 后端；


核心代码：
- [event.proto](../../../protocols/pb/example/event.proto)
- [normal_publisher_module.cc](../../cpp/pb_chn/module/normal_publisher_module/normal_publisher_module.cc)
- [normal_subscriber_module.cc](../../cpp/pb_chn/module/normal_subscriber_module/normal_subscriber_module.cc)


配置文件：
- [examples_plugins_iceoryx_plugin_pb_chn_pub_cfg.yaml](./install/linux/bin/cfg/examples_plugins_iceoryx_plugin_pb_chn_pub_cfg.yaml)
- [examples_plugins_iceoryx_plugin_pb_chn_sub_cfg.yaml](./install/linux/bin/cfg/examples_plugins_iceoryx_plugin_pb_chn_sub_cfg.yaml)

运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_ICEORYX_PLUGIN` 选项编译 AimRT；
- 编译成功后，在终端运行 build 目录下 iox-roudi 可执行文件以启动 iceoryx 的守护进程；
- 开启新的终端运行 build 目录下`start_examples_plugins_iceoryx_plugin_pb_chn_sub.sh`脚本启动订阅端（sub 进程）；
- 再开启一个新的终端窗口运行`start_examples_plugins_iceoryx_plugin_pb_chn_pub.sh`脚本启动发布端（pub 进程）；
- 分别在开启的三个终端键入`ctrl-c`停止对应进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalPublisherModule`：会基于 `work_thread_pool` 执行器，以配置的频率、向配置的 topic 中发布 `ExampleEventMsg` 类型的消息；
  - `NormalSubscriberModule`：会订阅配置的 topic 下的 `ExampleEventMsg` 类型的消息；
- 此示例将 `NormalPublisherModule` 和 `NormalSubscriberModule` 分别集成到 `pb_chn_pub_pkg` 和 `pb_chn_sub_pkg` 两个 Pkg 中，并在两个配置文件中分别加载对应的 Pkg 到 pub 和 sub 进程中；
- 此示例加载了**iceoryx_plugin**，并使用 iceoryx 类型的 channel 后端进行通信；
