# aimrt.log

## Configuration Overview

The `aimrt.log` configuration item is used to configure logging. Detailed configuration items are described below:

| Node                | Type   | Optional | Default | Description              |
| ------------------- | ------ | -------- | ------- | ------------------------ |
| core_lvl            | string | Optional | "Info"  | Framework log level      |
| default_module_lvl  | string | Optional | "Info"  | Default module log level |
| enable_crash_log    | bool   | Optional | false   | Whether to output stack trace on program crash |
| backends            | array  | Optional | ""      | List of log backends     |
| backends[i].type    | string | Required | ""      | Log backend type         |
| backends[i].options | map    | Optional | -       | Specific backend config  |

Log level options include the following (case-insensitive):

- Trace
- Debug
- Info
- Warn
- Error
- Fatal
- Off

`aimrt.log` configuration notes:

- `core_lvl` sets the log level for the AimRT runtime kernel; kernel logs are usually set to Info.
- `default_module_lvl` is the default module log level.
- `enable_crash_log` is a boolean type used to choose whether to output a stack trace when the program crashes
- `backends` is an array used to register various log backends.
  - `backends[i].type` is the type of the log backend. AimRT officially provides several log backends, and some plugins also provide additional types. Some backends allow duplicate registration; see the corresponding backend documentation for details.
  - `backends[i].options` are initialization parameters passed by AimRT to each log backend. The format of this configuration is defined by each backend type; refer to the corresponding backend documentation.

Here is a simple example:

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

## console Console Log Backend

The `console` log backend is an officially provided AimRT backend that prints logs to the console，which works on the guard thread. All its configuration items are as follows:

| Node          | Type   | Optional | Default                           | Description               |
| ------------- | ------ | -------- | --------------------------------- | ------------------------- |
| color         | bool   | Optional | true                              | Whether to print in color |
| module_filter | string | Optional | "(.\*)"                           | Module filter             |
| pattern       | string | Optional | "[%c.%f][%l][t][%n][%g:%R @%F]%v" | Log output format         |

Usage notes:

- The `console` backend does not allow duplicate registration; only one instance is allowed per AimRT process.
- `color` configures whether to use colored output. This may not be supported on some operating systems.
- `module_filter` supports regular expressions to determine which module logs are processed by this backend. This differs from module log levels, which are global, precedence-based, and affect all backends, whereas this filter only affects the current backend.
- `log_executor_name` specifies the log executor. It must be thread-safe; if not provided, the guard thread is used.
- `pattern` uses `"%" + character` for formatting. Available formats are:

| Format | Description             | Example                             |
| ------ | ----------------------- | ----------------------------------- |
| %c     | Full date and time      | 2024-03-15 14:30:45                 |
| %Y     | Year                    | 2024                                |
| %m     | Month                   | 03                                  |
| %d     | Day                     | 15                                  |
| %H     | Hour                    | 14                                  |
| %M     | Minute                  | 30                                  |
| %S     | Second                  | 45                                  |
| %D     | Date only               | 2024-03-15                          |
| %T     | Time only               | 14:30:45                            |
| %f     | Microseconds            | 123456                              |
| %A     | Weekday                 | Sunday                              |
| %a     | Weekday (abbreviated)   | Sun                                 |
| %l     | Log level               | Info                                |
| %t     | Thread ID               | 1234                                |
| %n     | Module name             | test_module                         |
| %G     | Filename (last segment) | test_file.cpp                       |
| %g     | Full file path          | /XX/YY/ZZ/test_file.cpp             |
| %R     | Line number             | 20                                  |
| %F     | Function name           | TestFunc                            |
| %v     | Log message content     | "This is a log message"             |
| %other | Literal output          | e.g. `%q` prints `q`, `%%` prints % |

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
      - type: console
        options:
          color: true
          module_filter: "(.*)"
          log_executor_name: test_log_executor.log
```

## rotate_file Rotating File Log Backend

The `rotate_file` log backend is an officially provided AimRT backend that writes logs to files, which works on the guard thread. All its configuration items are as follows:

| Node                  | Type         | Optional    | Default                           | Description                                                  |
| --------------------- | ------------ | ----------- | --------------------------------- | ------------------------------------------------------------ |
| path                  | string       | Required    | "./log"                           | Directory for log files                                      |
| filename              | string       | Required    | "aimrt.log"                       | Base name for log files                                      |
| max_file_size_m       | unsigned int | Optional    | 16                                | Maximum file size in Mb                                      |
| max_file_num          | unsigned int | Optional    | 100                               | Maximum number of files. 0 means unlimited                   |
| module_filter         | string       | Optional    | "(.\*)"                           | Module filter                                                |
| pattern               | string       | Optional    | "[%c.%f][%l][t][%n][%g:%R @%F]%v" | Log output format                                            |
| enable_sync           | bool         | Optional    | false                             | Enable periodic flush                                        |
| sync_interval_ms      | unsigned int | Optional    | 30000                             | Periodic flush interval in ms                                |
| sync_executor_name    | string       | Conditional | ""                                | Executor for periodic flush. Required if enable_sync is true |
| suffix_with_timestamp | bool         | Optional    | true                              | Append timestamp suffix to log file name                     |

Usage notes:

- The `rotate_file` backend allows duplicate registration, enabling different modules to log to different files.
- `path` sets the directory for log files. If the directory does not exist, it will be created; creation failure throws an exception.
- `filename` sets the base name for log files.
- When a log file exceeds `max_file_size_m`, a new file is created and the old one is renamed with an `_x` suffix.
- When the number of log files exceeds `max_file_num`, the oldest file is deleted. Setting to 0 disables deletion.
- `module_filter` supports regular expressions to determine which module logs are processed by this backend. This differs from module log levels, which are global, precedence-based, and affect all backends, whereas this filter only affects the current backend.
- `log_executor_name` specifies the log executor. It must be thread-safe; if not provided, the guard thread is used.
- `pattern` uses `"%" + character` for formatting; see the `pattern` section under [console console](#console-console-log-backend).
- `enable_sync` enables periodic flushing. When enabled, data is flushed to disk every `sync_interval_ms` to ensure data integrity, but performance is reduced.
- `sync_interval_ms` sets the flush interval in ms. Balance between data integrity and performance.
- `sync_executor_name` specifies the executor for periodic flushing. The executor must support timer scheduling.
- The `suffix_with_timestamp` configuration controls whether to append a timestamp to the log file name, which represents the log file rotation time.

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
