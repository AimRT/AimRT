# parameter example

一个基于 App 模式创建模块的方式启动的 python parameter 示例，演示内容包括：
- 如何以 App 模式创建模块并启动；
- 如何使用 Log 功能；
- 如何使用 Parameter 功能；

核心代码：
- [examples_py_parameter_app.py](./examples_py_parameter_app.py)


配置文件：
- [examples_py_parameter_app_cfg.yaml](./cfg/examples_py_parameter_app_cfg.yaml)


运行方式（linux）：
- [安装 `aimrt_py` 包](../../../../document/sphinx-cn/tutorials/quick_start/installation_py.md)；
- 直接运行本目录下[start_examples_py_parameter_app.sh](./start_examples_py_parameter_app.sh)脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例使用 App 模式，在 main 函数中启动 `AimRTCore` 实例，并创建了一个 `ParameterModule` 模块句柄，使用该句柄在两个线程中分别调用 `SetParameter` 和 `GetParameter` 接口，分别进行设置参数和读取参数的操作；
- 可以在启动后观察控制台打印出来的日志，了解框架运行情况；
