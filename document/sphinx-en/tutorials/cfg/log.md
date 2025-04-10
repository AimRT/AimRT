

# aimrt.log

## Configuration Overview

The `aimrt.log` configuration item is used to configure logging. Detailed configuration items are as follows:

| Node                 | Type   | Optional | Default Value | Purpose                |
| -------------------- | ------ | -------- | ------------- | ---------------------- |
| core_lvl             | string | Yes      | "Info"        | Framework log level    |
| default_module_lvl   | string | Yes      | "Info"        | Default module log level |
| backends             | array  | Yes      | ""            | List of log backends    |
| backends[i].type     | string | Required | ""            | Log backend type        |
| backends[i].options  | map    | Yes      | -             | Backend-specific config |

Available log levels (case-insensitive):
- Trace
- Debug
- Info
- Warn
- Error
- Fatal
- Off

Configuration notes:
- `core_lvl` sets the log level for AimRT runtime kernel, typically set to Info.
- `default_module_lvl` defines the default log level for modules.
- `backends` array registers log backends:
  - `backends[i].type` specifies backend type. Official/plugin-provided backends are supported.
  - `backends[i].options` contains initialization parameters for specific backends.

Example:
```yaml
aimrt:
  log:
    core_lvl: INFO
    default_module_lvl: INFO
    backends:
      - type: console
      - type: rotate_file
        options:
          path: ./log
          filename: examples_cpp_executor_real_time.log
```

## console Control Backend

The `console` backend prints logs to console. Configuration items:

| Node               | Type   | Optional | Default Value                        | Purpose                     |
| ------------------ | ------ | -------- | ------------------------------------- | --------------------------- |
| color              | bool   | Yes      | true                                 | Enable color printing       |
| module_filter      | string | Yes      | "(.*)"                               | Module filter regex         |
| log_executor_name  | string | Yes      | ""                                   | Log executor name           |
| pattern            | string | Yes      | "[%c.%f][%l][%t][%n][%g:%R @%F]%v"   | Log format pattern          |

Usage notes:
- Only one `console` backend allowed per AimRT instance
- `color` may not work on some OS
- `module_filter` uses regex to filter modules for this backend
- Unconfigured `log_executor_name` uses guard thread
- Pattern format specifiers:

| Spec | Description                  | Example                          |
| ---- | ---------------------------- | -------------------------------- |
| %c   | Full datetime                | 2024-03-15 14:30:45              |
| %Y   | Year                         | 2024                             |
| %m   | Month                        | 03                               |
| %d   | Day                          | 15                               |
| %H   | Hour                         | 14                               |
| %M   | Minute                       | 30                               |
| %S   | Second                       | 45                               |
| %D   | Date only                    | 2024-03-15                       |
| %T   | Time only                    | 14:30:45                         |
| %f   | Microseconds                 | 123456                           |
| %A   | Weekday name                 | Sunday                           |
| %a   | Abbreviated weekday          | Sun                              |
| %l   | Log level                    | Info                             |
| %t   | Thread ID                    | 1234                             |
| %n   | Module name                  | test_module                      |
| %G   | Filename (basename)          | test_file.cpp                    |
| %g   | Full file path               | /XX/YY/ZZ/test_file.cpp          |
| %R   | Line number                  | 20                               |
| %F   | Function name                | TestFunc                         |
| %v   | Log message content          | "This is a log message"          |
| %other | Display literal character | `%q` shows `q`, `%%` shows `%` |

Example:
```yaml
aimrt:
  executor:
    executors:
      - name: test_log_executor
        type: tbb_thread
        options:
          thread_num: 1
  log:
    core_lvl: INFO
    default_module_lvl: INFO
    backends:
      - type: console
        options:
          color: true
          module_filter: "(.*)"
          log_executor_name: test_log_executor.log
```


## rotate_file 滚动文件日志后端

The `rotate_file` log backend is an official log backend provided by AimRT, used for printing logs to files. All its configuration items are as follows:

| Node               | Type         | Optional | Default Value                        | Description                                     |
| ------------------ | ------------ | -------- | ------------------------------------- | ----------------------------------------------- |
| path               | string       | Required | "./log"                               | Directory for storing log files                 |
| filename           | string       | Required | "aimrt.log"                           | Base name of log file                           |
| max_file_size_m    | unsigned int | Optional | 16                                    | Maximum size of log file, unit: Mb              |
| max_file_num       | unsigned int | Optional | 100                                   | Maximum number of log files. 0 means no limit   |
| module_filter      | string       | Optional | "(.*)"                                | Module filter                                   |
| log_executor_name  | string       | Optional | ""                                    | Log executor. Default uses main thread          |
| pattern            | string       | Optional | "[%c.%f][%l][%t][%n][%g:%R @%F]%v"    | Log output format                               |
| enable_sync        | bool         | Optional | false                                 | Whether to enable periodic sync to disk         |
| sync_interval_ms   | unsigned int | Optional | 30000                                 | Interval for periodic sync, unit: ms            |
| sync_executor_name | string       | Conditional Required | "" | Executor for periodic sync. Required when `enable_sync` is true |

Usage notes:
- The `rotate_file` log backend allows multiple registrations, enabling different modules to log to different files.
- `path` configures the directory for log files. If the path doesn't exist, it will be created. If creation fails, an exception will be thrown.
- `filename` configures the base name of the log file.
- When a single log file exceeds the size configured in `max_file_size_m`, a new log file is created and the old one is renamed with a suffix like `_x`.
- When the number of log files exceeds `max_file_num`, the oldest log file is deleted. If set to 0, files will never be deleted.
- `module_filter` uses regular expressions to configure which modules' logs are processed by this backend. This is different from module log levels, which are global and affect all log backends, while this configuration only affects this backend.
- `log_executor_name` configures the log executor. The executor must be thread-safe. If not configured, it defaults to using a guard thread for logging.
- `pattern` uses `"%" + character` for formatted output. Refer to the `pattern` field description in the [console backend](#console-log-backend) configuration for details.
- `enable_sync` configures whether to enable periodic sync to disk. When enabled, data is periodically synced to disk at intervals configured by `sync_interval_ms` to ensure data integrity, but this may reduce performance.
- `sync_interval_ms` configures the interval for periodic sync, unit: ms. Choose an appropriate value balancing data integrity and performance.
- `sync_executor_name` configures the executor for periodic sync. The executor must support timer scheduling.


Here is a simple example:
```yaml
aimrt:
  executor:
    executors:
      - name: test_log_executor
        type: tbb_thread
        options:
          thread_num: 1
  log:
    core_lvl: INFO
    default_module_lvl: INFO
    backends:
      - type: rotate_file
        options:
          path: ./log
          filename: example.log
          max_file_size_m: 4
          max_file_num: 10
          module_filter: "(.*)"
          log_executor_name: test_log_executor.log
```