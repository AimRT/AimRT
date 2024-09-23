# helloworld examples


## helloworld

一个最基本的 cpp helloworld 示例，演示内容包括：
- 如何继承 ModuleBase 创建一个模块；
- 如何以 Pkg 模式启动；
- 如何使用 Configurator 功能；
- 如何使用 Log 功能；


核心代码：
- [helloworld_module.cc](./module/helloworld_module/helloworld_module.cc)
- [pkg_main.cc](./pkg/helloworld_pkg/pkg_main.cc)


配置文件：
- [examples_cpp_helloworld_cfg.yaml](./install/linux/bin/cfg/examples_cpp_helloworld_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_helloworld.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了一个 `HelloWorldModule`，会在`Initialize`时读取配置并打印出来，同时还会在其 `Initialize`、`Start`、`Shutdown`的阶段各打印一行日志；
- 此示例将 `HelloWorldModule` 集成到 `helloworld_pkg` 中，并在配置文件中加载此 Pkg；
- 可以在启动后观察控制台打印出来的日志，了解框架运行情况；


## helloworld app registration mode

一个基于 App 模式注册模块的方式启动的 cpp helloworld 示例，演示内容包括：
- 如何继承 ModuleBase 创建一个模块；
- 如何以 App 模式注册模块并启动；
- 如何使用 Configurator 功能；
- 如何使用 Log 功能；


核心代码：
- [helloworld_module.cc](./module/helloworld_module/helloworld_module.cc)
- [main.cc](./app/helloworld_app_registration_mode/main.cc)


配置文件：
- [examples_cpp_helloworld_app_mode_cfg.yaml](./install/linux/bin/cfg/examples_cpp_helloworld_app_mode_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_helloworld_app_registration_mode.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了一个 `HelloWorldModule`，会在`Initialize`时读取配置并打印出来，同时还会在其 `Initialize`、`Start`、`Shutdown`的阶段各打印一行日志；
- 此示例使用 App 模式，在 main 函数中启动 `AimRTCore` 实例，并将 `HelloWorldModule` 注册到其中；
- 可以在启动后观察控制台打印出来的日志，了解框架运行情况；


## helloworld app mode

一个基于 App 模式创建模块的方式启动的 cpp helloworld 示例，演示内容包括：
- 如何以 App 模式创建模块并启动；
- 如何使用 Configurator 功能；
- 如何使用 Log 功能；


核心代码：
- [main.cc](./app/helloworld_app/main.cc)


配置文件：
- [examples_cpp_helloworld_app_mode_cfg.yaml](./install/linux/bin/cfg/examples_cpp_helloworld_app_mode_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_helloworld_app_mode.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例使用 App 模式，在 main 函数中启动 `AimRTCore` 实例，并创建了一个 `HelloWorldModule` 模块句柄，使用该句柄读取配置并打印出来；
- 可以在启动后观察控制台打印出来的日志，了解框架运行情况；
