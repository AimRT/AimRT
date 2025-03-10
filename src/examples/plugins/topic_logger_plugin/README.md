# topic logger plugin examples

一个基于 **topic_logger_plugin** 的话题日志后端示例，演示内容包括：
- 如何在启动时加载 **topic_logger_plugin**；
- 如何配置 **topic_logger** 日志后端以将日志信息以 topic 形式发送；


核心代码：
- [topic_logger.proto](../../../protocols/plugins/topic_logger_plugin/topic_logger.proto)
- [logger_module.cc](../../cpp/logger/module/logger_module/logger_module.cc)
- [type_support_pkg_main.cc](./topic_logger_plugin_ts_pkg/type_support_pkg_main.cc)


配置文件：
- [examples_plugins_topic_logger_plugin_cfg.yaml](./install/linux/bin/cfg/examples_plugins_topic_logger_plugin_cfg.yaml)
- [examples_plugins_topic_logger_plugin_echo_cfg.yaml](./install/linux/bin/cfg/examples_plugins_topic_logger_plugin_echo_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF` 、`AIMRT_BUILD_NET_PLUGIN` 、`AIMRT_BUILD_WITH_ECHO_PLUGIN` 、`AIMRT_BUILD_TOPIC_LOGGER_PLUGIN` 选项编译 AimRT；
- 在终端运行 build 目录下`start_examples_plugins_topic_logger_plugin_echo.sh`脚本启动 echo 进程来订阅日志数据；
- 在新的终端运行 build 目录下`start_examples_plugins_topic_logger_plugin.sh`脚本启动话题日志进程发送日志数据；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了一个 `LoggerModule`，会在其 `Start` 的阶段循环打印各种等级的日志， 并通过 topic_logger 日志后端将其以话题形式发送到 echo 进程中，  echo 进程订阅数据并将收到的数据打印到终端；
- 启动的两个线程通过 udp 后端进行通信；
