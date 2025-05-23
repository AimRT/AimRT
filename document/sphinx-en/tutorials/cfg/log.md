# aimrt.log

## Configuration Overview

The `aimrt.log` configuration is used to set up logging. The detailed configuration items are described below:

| Node                | Type   | Optional | Default Value | Purpose               |
| ------------------- | ------ | -------- | ------------- | --------------------- |
| core_lvl            | string | Optional | "Info"        | Framework log level   |
| default_module_lvl  | string | Optional | "Info"        | Default module log level |
| backends            | array  | Optional | ""            | List of log backends  |
| backends[i].type    | string | Required | ""            | Log backend type      |
| backends[i].options | map    | Optional | -             | Configuration for specific log backend |

The available log levels (case-insensitive) include:
- Trace
- Debug
- Info
- Warn
- Error
- Fatal
- Off

Configuration notes for `aimrt.log`:
- `core_lvl` represents the log level for the AimRT runtime kernel. Kernel logs are generally set to Info level.
- `default_module_lvl` is the default log level for modules.
- `backends` is an array used to register various log backends.
  - `backends[i].type` specifies the type of log backend. AimRT officially provides several log backend types, and some plugins may offer additional ones. Some backends allow duplicate registration - refer to the corresponding backend documentation for details.
  - `backends[i].options` contains initialization parameters passed by AimRT to each log backend. The format of these configurations is defined by each log backend type - refer to the corresponding backend documentation.

Here's a simple example:
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

## Console Log Backend

The `console` log backend is an official AimRT log backend that prints logs to the console. All its configuration items are:

| Node              | Type   | Optional | Default Value                         | Purpose                     |
| ----------------- | ------ | -------- | ------------------------------------- | --------------------------- |
| color             | bool   | Optional | true                                  | Whether to use color output |
| module_filter     | string | Optional | "(.*)"                                | Module filter               |
| log_executor_name | string | Optional | ""                                    | Log executor (defaults to main thread) |
| pattern           | string | Optional | "[%c.%f][%l][%t][%n][%g:%R @%F]%v"    | Log output format           |

Usage notes:
- The `console` log backend does not allow duplicate registration - only one instance is permitted per AimRT instance.
- `color` configures colored output, which may not be supported on some operating systems.
- `module_filter` uses regular expressions to specify which module logs can be processed by this backend. Unlike module log levels (which are global and affect all backends), this configuration only affects this specific backend.
- `log_executor_name` specifies the log executor, which must be thread-safe. If not configured, it defaults to using the guard thread for log printing.
- `pattern` formats output using `"%" + character` sequences. Available formats:

 | Format | Explanation                  | Example                          |
 | ------ | ---------------------------- | -------------------------------- |
 | %c     | Full date and time           | 2024-03-15 14:30:45             |
 | %Y     | Year                         | 2024                            |
 | %m     | Month                        | 03                              |
 | %d     | Day                          | 15                              |
 | %H     | Hour                         | 14                              |
 | %M     | Minute                       | 30                              |
 | %S     | Second                       | 45                              |
 | %D     | Date only                    | 2024-03-15                      |
 | %T     | Time only                    | 14:30:45                        |
 | %f     | Microseconds                 | 123456                          |
 | %A     | Weekday name                 | Sunday                          |
 | %a     | Abbreviated weekday          | Sun                             |
 | %l     | Log level                    | Info                            |
 | %t     | Thread ID                    | 1234                            |
 | %n     | Module name                  | test_module                     |
 | %G     | Filename (last component)    | test_file.cpp                   |
 | %g     | Filename (full path)         | /XX/YY/ZZ/test_file.cpp         |
 | %R     | Line number                  | 20                              |
 | %F     | Function name                | TestFunc                        |
 | %v     | Log message content          | "This is a log message"         |
 | %other | Displays the character       | `%q` shows `q`, `%%` shows `%`  |

Here's a simple example:
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

## Rotate File Log Backend

The `rotate_file` log backend is an official AimRT log backend that prints logs to files. All its configuration items are:

| Node               | Type         | Optional | Default Value                         | Purpose                                            |
| ------------------ | ------------ | -------- | ------------------------------------- | -------------------------------------------------- |
| path               | string       | Required | "./log"                               | Log file directory                                 |
| filename           | string       | Required | "aimrt.log"                           | Base filename for log files                        |
| max_file_size_m    | unsigned int | Optional | 16                                    | Maximum log file size (MB)                         |
| max_file_num       | unsigned int | Optional | 100                                   | Maximum number of log files (0 means unlimited)    |
| module_filter      | string       | Optional | "(.*)"                                | Module filter                                      |
| log_executor_name  | string       | Optional | ""                                    | Log executor (defaults to main thread)             |
| pattern            | string       | Optional | "[%c.%f][%l][%t][%n][%g:%R @%F]%v"    | Log output format                                  |
| enable_sync        | bool         | Optional | false                                 | Whether to enable periodic disk sync               |
| sync_interval_ms   | unsigned int | Optional | 30000                                 | Sync interval (ms)                                 |
| sync_executor_name | string       | Conditionally Required | ""                                    | Sync executor (required when `enable_sync` is true) |

Usage notes:
- The `rotate_file` log backend allows duplicate registration, enabling different modules to log to different files.
- `path` specifies the directory for log files. If the path doesn't exist, it will be created. Creation failure throws an exception.
- `filename` specifies the base name for log files.
- When a log file exceeds `max_file_size_m`, a new file is created and the old one is renamed with an `_x` suffix.
- When the number of log files exceeds `max_file_num`, the oldest file is deleted. Setting this to 0 means files are never deleted.
- `module_filter` uses regular expressions to specify which module logs can be processed by this backend.
- `log_executor_name` specifies the log executor, which must be thread-safe. If not configured, it defaults to using the guard thread.
- `pattern` formats output using `"%" + character` sequences (see [Console Log Backend](#console-log-backend) for format details).
- `enable_sync` enables periodic disk syncing to ensure data integrity (with performance impact).
- `sync_interval_ms` sets the sync interval (ms). Balance between data integrity and performance.
- `sync_executor_name` specifies the executor for periodic syncing, which must support timer scheduling.

Here's a simple example:
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