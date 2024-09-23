# helloworld examples

## helloworld app mode

一个基于 App 模式创建模块的方式启动的 python helloworld 示例，演示内容包括：
- 如何以 App 模式创建模块并启动；
- 如何使用 Configurator 功能；
- 如何使用 Log 功能；

核心代码：
- [examples_py_helloworld_app_mode.py](./examples_py_helloworld_app_mode.py)


配置文件：
- [examples_py_helloworld_app_mode_cfg.yaml](./cfg/examples_py_helloworld_app_mode_cfg.yaml)


运行方式（linux）：
- 安装 `aimrt_py` 包；
- 直接运行本目录下[start_examples_py_helloworld_app_mode.sh](./start_examples_py_helloworld_app_mode.sh)脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例使用 App 模式，在 main 函数中启动 `AimRTCore` 实例，并创建了一个 `HelloWorldModule` 模块句柄，使用该句柄读取配置并打印出来，同时在 start 之后启动了一个线程循环打印日志；
- 可以在启动后观察控制台打印出来的日志，了解框架运行情况；


## helloworld registration mode

一个基于 App 模式注册模块的方式启动的 python helloworld 示例，演示内容包括：
- 如何继承 ModuleBase 创建一个模块；
- 如何以 App 模式注册模块并启动；
- 如何使用 Configurator 功能；
- 如何使用 Log 功能；
- 如何使用 Executor 功能；

核心代码：
- [helloworld_module.py](./helloworld_module.py)
- [examples_py_helloworld_registration_mode.py](./examples_py_helloworld_registration_mode.py)


配置文件：
- [examples_py_helloworld_registration_mode_cfg.yaml](./cfg/examples_py_helloworld_registration_mode_cfg.yaml)


运行方式（linux）：
- 安装 `aimrt_py` 包；
- 直接运行本目录下[start_examples_py_helloworld_registration_mode.sh](./start_examples_py_helloworld_registration_mode.sh)脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了一个 `HelloWorldModule`，会在`Initialize`时读取配置并打印出来，同时还会尝试获取名为`work_thread_pool`的执行器，在 Start 阶段之后向该执行器中投递任务；
- 此示例使用 App 模式，在 main 函数中启动 `AimRTCore` 实例，并将 `HelloWorldModule` 注册到其中；
- 可以在启动后观察控制台打印出来的日志，了解框架运行情况；
