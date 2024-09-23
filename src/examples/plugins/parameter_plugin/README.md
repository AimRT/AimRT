# parameter plugin examples

# set/get parameter by http


一个基于 **parameter_plugin** 和 **net_plugin** 中 http 后端的参数 set/get 示例，演示内容包括：
- 如何在启动时加载 **parameter_plugin**；
- 如何为 **parameter_plugin** 提供的 set/get 服务配置 http 后端；
- 如何通过 curl 命令实现参数 set/get；



核心代码：
- [parameter_module.cc](../../cpp/parameter/module/parameter_module/parameter_module.cc)


配置文件：
- [examples_plugins_parameter_plugin_cfg.yaml](./install/linux/bin/cfg/examples_plugins_parameter_plugin_cfg.yaml)



运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_WITH_PROTOBUF`、`AIMRT_BUILD_PARAMETER_PLUGIN`、`AIMRT_BUILD_NET_PLUGIN` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_plugins_parameter_plugin.sh`脚本启动进程；
- 启动[tools](./install/linux/bin/tools)下的脚本并观察进程打印出来的日志：
  - 运行[parameter_plugin_get_parameter.sh](./install/linux/bin/tools/parameter_plugin_get_parameter.sh)脚本获取某个参数值；
  - 运行[parameter_plugin_list_parameter.sh](./install/linux/bin/tools/parameter_plugin_list_parameter.sh)脚本获取所有参数的 key；
  - 运行[parameter_plugin_set_parameter.sh](./install/linux/bin/tools/parameter_plugin_set_parameter.sh)脚本设置某个参数的值；
  - 运行[parameter_plugin_dump_parameter.sh](./install/linux/bin/tools/parameter_plugin_dump_parameter.sh)脚本将当前所有参数 dump 到本地；
  - 运行[parameter_plugin_load_parameter.sh](./install/linux/bin/tools/parameter_plugin_load_parameter.sh)脚本加载之前 dump 下来的参数包；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了一个 `ParameterModule`，会在其 `Start` 的阶段循通过 SetParameter 和 GetParameter 方法设置和获取参数的值，并通过日志打印出来；
- 此示例通过 http 后端对外暴露 **parameter_plugin** 提供的 `ParameterService` 服务，并通过 curl 命令调用他们，实现对参数的 set/get 能力；

