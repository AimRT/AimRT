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
- [examples_plugins_proxy_plugin_http_pub_cfg.yaml](./install/linux/bin/cfg/examples_plugins_proxy_plugin_http_pub_cfg.yaml)
- [examples_plugins_proxy_plugin_cfg.yaml](./install/linux/bin/cfg/examples_plugins_proxy_plugin_cfg.yaml)
- [examples_plugins_proxy_plugin_zenoh_sub_cfg.yaml](./install/linux/bin/cfg/examples_plugins_proxy_plugin_zenoh_sub_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF`、`AIMRT_BUILD_WITH_PROXY_PLUGIN`、`AIMRT_BUILD_WITH_ZENOH_PLUGIN`、`AIMRT_BUILD_WITH_NET_PLUGIN` 选项编译 AimRT；
- 分别运行 build 目录下 `start_examples_plugins_proxy_plugin_zenoh_sub.sh` ， `start_examples_plugins_proxy_plugin.sh` 和 `start_examples_plugins_proxy_plugin_http_pub.sh` 脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了三个进程，分别为 http 发布消息进程、zenoh 订阅消息进程和 proxy 转发进程：
  - `http_pub`，会以配置的频率，通过 `Publisher` 接口，发布 `ExampleEventMsg` 消息；
  - `proxy`，会基于 `proxy_plugin_executor` 执行器，执行代理转发操作；
  - `zenoh_sub`，会订阅 `test_topic_zenoh` 消息；
- 请注意，proxy 插件的原理是向 AimRT 订阅指定的 Topic，因此需要在 channel 配置中为该 topic 设置合适的后端，以保证插件能接收到数据，在此示例中，`test_topic_http` 配置了 `http` 后端，`test_topic_zenoh` 配置了 `zenoh` 后端；