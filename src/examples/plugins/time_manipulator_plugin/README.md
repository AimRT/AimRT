# time manipulator plugin examples


# time manipulator by http



一个基于 **time_manipulator_plugin** 和 **net_plugin** 中 http 后端的可调速执行器的示例，演示内容包括：
- 如何在启动时加载 **time_manipulator_plugin**；
- 如何为 **time_manipulator_plugin** 提供的执行器调速服务配置 http 后端；
- 如何通过 curl 命令调节执行器速度；



核心代码：
- [executor_co_loop_module.cc](../../cpp/executor/module/executor_co_loop_module/executor_co_loop_module.cc)


配置文件：
- [examples_plugins_time_manipulator_plugin_cfg.yaml](./install/linux/bin/cfg/examples_plugins_time_manipulator_plugin_cfg.yaml)



运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF`、`AIMRT_BUILD_TIME_MANIPULATOR_PLUGIN`、`AIMRT_BUILD_NET_PLUGIN` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_plugins_time_manipulator_plugin.sh`脚本启动进程；
- 启动[tools](./install/linux/bin/tools)下的脚本并观察进程打印出来的日志：
  - 运行[time_manipulator_get_time_ratio.sh](./install/linux/bin/tools/time_manipulator_get_time_ratio.sh)脚本获取当前执行器时间系数；
  - 运行[time_manipulator_pause.sh](./install/linux/bin/tools/time_manipulator_pause.sh)脚本暂停执行器；
  - 运行[time_manipulator_set_time_ratio.sh](./install/linux/bin/tools/time_manipulator_set_time_ratio.sh)脚本设置执行器时间系数为 0.5；
  - 使用 python 运行[time_manipulator.py](./install/linux/bin/tools/time_manipulator.py)脚本，可以通过一个 gui 界面上的滑动条控制执行器的时间系数；
- 键入`ctrl-c`停止进程；


说明：
- 此示例基于 `ExecutorCoLoopModule`，会在 `Start`的阶段以一定的时间间隔段循打印日志，通过将执行器配置为 `time_manipulator` 类型，即可通过外部 rpc 控制循环的间隔时间；


