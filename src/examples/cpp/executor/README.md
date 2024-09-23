# executor examples


## executor

一个最基本的 cpp executor 示例，演示内容包括：
- 如何获取执行器；
- 如何投递任务到执行器中执行；
- 如何使用**线程安全型**执行器；
- 如何投递定时任务到执行器中；


核心代码：
- [executor_module.cc](./module/executor_module/executor_module.cc)
- [pkg_main.cc](./pkg/executor_pkg/pkg_main.cc)


配置文件：
- [examples_cpp_executor_cfg.yaml](./install/linux/bin/cfg/examples_cpp_executor_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_executor.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了一个 `ExecutorModule`，会在`Initialize`时获取以下三种执行器：
  - 名称为`work_executor`的普通执行器；
  - 名称为`thread_safe_executor`的线程安全型执行器；
  - 名称为`time_schedule_executor`的支持定时任务的执行器；
- `ExecutorModule`模块在 `Start` 阶段会依次使用获取的执行器运行具体的逻辑任务：
  - `work_executor`：投递一个简单的任务到其中执行；
  - `thread_safe_executor`：向该执行器中一次性投递 10000 个任务，用于递增一个整数 n，并在最终打印出 n 的值，由于执行器是线程安全，故 n 最终值还是 10000；
  - `time_schedule_executor`：通过该执行器实现了一个间隔 1s 的定时循环；
- 此示例将 `ExecutorModule` 集成到 `executor_pkg` 中，并在配置文件中加载此 Pkg；


## executor co

一个基于协程接口使用 executor 功能的示例，演示内容包括：
- 如何以协程的方式使用执行器；

核心代码：
- [executor_co_module.cc](./module/executor_co_module/executor_co_module.cc)
- [pkg_main.cc](./pkg/executor_pkg/pkg_main.cc)


配置文件：
- [examples_cpp_executor_co_cfg.yaml](./install/linux/bin/cfg/examples_cpp_executor_co_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_executor_co.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了一个 `ExecutorCoModule`，其中与**executor**演示的功能示例基本一致，只不过使用了协程接口来实现；


## executor co loop

一个基于协程接口使用 executor 功能实现定时循环的示例，演示内容包括：
- 如何以协程的方式使用执行器实现定时循环；


核心代码：
- [executor_co_loop_module.cc](./module/executor_co_loop_module/executor_co_loop_module.cc)
- [pkg_main.cc](./pkg/executor_pkg/pkg_main.cc)


配置文件：
- [examples_cpp_executor_co_loop_cfg.yaml](./install/linux/bin/cfg/examples_cpp_executor_co_loop_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_executor_co_loop.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了一个 `ExecutorCoLoopModule`，会在 `Start`的阶段以 1s 的定时间隔段循打印日志；
- 此示例将 `ExecutorCoLoopModule` 集成到 `executor_pkg` 中，并在配置文件中加载此 Pkg；


## executor real time

一个 executor 实时性相关功能的示例，演示内容包括：
- 如何通过配置文件设置执行器的线程调度策略和优先级、绑核策略等；
- 本示例仅在 linux 上有效；

核心代码：
- [real_time_module.cc](./module/real_time_module/real_time_module.cc)
- [pkg_main.cc](./pkg/executor_pkg/pkg_main.cc)


配置文件：
- [examples_cpp_executor_real_time_cfg.yaml](./install/linux/bin/cfg/examples_cpp_executor_real_time_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_executor_real_time.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了一个 `RealTimeModule`，会在`Initialize`时获取以下几种执行器：
  - `sched_fifo_thread`：配置了 SCHED_FIFO 调度策略，并绑了 2 个核的单线程 `asio_thread` 类型执行器；
  - `sched_other_thread`：配置了 SCHED_OTHER 调度策略，并绑了 2 个核的单线程 `asio_thread` 类型执行器；
  - `sched_rr_thread`：配置了 SCHED_RR 调度策略，并绑了 3 个核的单线程 `asio_thread` 类型执行器；
- `RealTimeModule` 在 `Start`的阶会使用这三种执行器进行定时调度，并评估其实时性，通过日志打印其循环次数、调度策略，当前使用的CPU等信息：
- 此示例将 `RealTimeModule` 集成到 `executor_pkg` 中，并在配置文件中加载此 Pkg；
