# logger examples


## logger

一个最基本的 cpp logger 示例，演示内容包括：
- 如何在 AimRT 中打印 Log 到控制台；
- 如何使用不同的日志级别；


核心代码：
- [logger_module.cc](./module/logger_module/logger_module.cc)
- [pkg_main.cc](./pkg/logger_pkg/pkg_main.cc)


配置文件：
- [examples_cpp_logger_cfg.yaml](./install/linux/bin/cfg/examples_cpp_logger_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_logger.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了一个 `LoggerModule`，会在其 `Start` 的阶段循环打印各种等级的日志；
- 此示例将 `LoggerModule` 集成到 `logger_pkg` 中，并在配置文件中加载此 Pkg；

  
## logger rotate file

一个最基本的 cpp logger 示例，演示内容包括：
-  如何使用 rotate_file 类型 Log 后端并了解其配置项；

核心代码：
- [logger_module.cc](./module/logger_module/logger_module.cc)
- [pkg_main.cc](./pkg/logger_pkg/pkg_main.cc)


配置文件：
- [examples_cpp_logger_rotate_file_cfg.yaml](./install/linux/bin/cfg/examples_cpp_logger_rotate_file_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_logger_rotate_file.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；



说明：
- 此示例创建了一个 `LoggerModule`，会在其 `Start` 的阶段循环打印各种等级的日志；
- 此示例会在配置文件指定的目录中 "./log" 生成的日志文件 "examples_cpp_logger_rotate_file.log" 文件，并将日志写入其中；
- 可以看到在该配置文件中多了如下配置：
```yaml
      - type: rotate_file
        options:
          path: ./log           # 日志保存目标目录
          filename: examples_cpp_logger_rotate_file.log # 日志名
          max_file_size_m: 4    # 定义了单个日志文件的最大大小，单位为 MB。这里指定为 4MB，意味着当日志文件达到或超过这个大小时，就会触发轮替
          max_file_num: 10      # 指定了要保留的最大日志文件数量。在这个配置中，最多会保留 10 个日志文件，包括当前正在使用的那个文件。
```

## logger specify executor

一个最基本的 cpp logger 示例，演示内容包括：
- 如何使用指定的执行器作为日志后端的执行线程；

核心代码：
- [logger_module.cc](./module/logger_module/logger_module.cc)
- [pkg_main.cc](./pkg/logger_pkg/pkg_main.cc)


配置文件：
- [examples_cpp_logger_specify_executor_cfg.yaml](./install/linux/bin/cfg/examples_cpp_logger_specify_executor_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES` 选项编译 AimRT；
- 直接运行 build 目录下`start_examples_cpp_logger_specify_executor.sh`脚本启动进程；
- 键入`ctrl-c`停止进程；


说明：
- 此示例创建了一个 `LoggerModule`，会在其 `Start`的阶段循环打印各种等级的日志；
- 此示例将 `LoggerModule` 集成到 `logger_pkg` 中，并在配置文件中加载此 Pkg；
- 模块的执行任务将会在配置文件指定的执器上运行；
- 可以看到在该配置文件中多了如下配置：
```yaml
  log:
    core_lvl: INFO # Trace/Debug/Info/Warn/Error/Fatal/Off
    backends:
      - type: console
        options:
          log_executor_name: log_executor  # 指定日志执行器名称，这里要和 executors 中列举的执行器列表匹配
  executor:
    executors: # 执行器列表
      - name: work_executor
        type: simple_thread
      - name: log_executor  # 执行器名称（可自定义）
        type: simple_thread # 执行器类型（类型具体参考 AimRT 使用手册执行器部分）
```

