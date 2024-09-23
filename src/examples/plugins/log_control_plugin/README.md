# log control plugin examples


# control log level by http

一个基于 **log_control_plugin** 和 **net_plugin** 中 http 后端的日志等级实时控制示例，演示内容包括：
- 如何在启动时加载 **log_control_plugin**；
- 如何为 **log_control_plugin** 提供的日志控制服务配置 http 后端；
- 如何通过 curl 命令实时控制日志等级；



核心代码：
- [logger_module.cc](../../cpp/logger/module/logger_module/logger_module.cc)


配置文件：
- [examples_plugins_log_control_plugin_cfg.yaml](./install/linux/bin/cfg/examples_plugins_log_control_plugin_cfg.yaml)



运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF`、`AIMRT_BUILD_LOG_CONTROL_PLUGIN`、`AIMRT_BUILD_NET_PLUGIN` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_plugins_log_control_plugin.sh`脚本启动进程；
- 启动[tools](./install/linux/bin/tools)下的脚本并观察进程打印出来的日志：
  - 运行[log_control_plugin_get_lvl.sh](./install/linux/bin/tools/log_control_plugin_get_lvl.sh)脚本获取当前日志等级；
  - 运行[log_control_plugin_set_lvl.sh](./install/linux/bin/tools/log_control_plugin_set_lvl.sh)脚本设置日志等级为 Trace；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了一个 `LoggerModule`，会在其 `Start` 的阶段循环打印各种等级的日志；
- 此示例通过 http 后端对外暴露 **log_control_plugin** 提供的 `LogControlService` 服务，并通过 curl 命令调用他们，实现实时控制日志等级的效果；

