# aimrt.log


## 配置项概述

`aimrt.log`配置项用于配置日志。其中的细节配置项说明如下：

| 节点                | 类型   | 是否可选 | 默认值 | 作用               |
| ------------------- | ------ | -------- | ------ | ------------------ |
| core_lvl            | string | 可选     | "Info" | 框架日志等级       |
| default_module_lvl  | string | 可选     | "Info" | 默认的模块日志等级 |
| backends            | array  | 可选     | ""     | 日志后端列表       |
| backends[i].type    | string | 必选     | ""     | 日志后端类型       |
| backends[i].options | map    | 可选     | -      | 具体日志后端的配置 |

其中，日志等级可选项包括以下几种（不区分大小写）：
- Trace
- Debug
- Info
- Warn
- Error
- Fatal
- Off


`aimrt.log`的配置说明如下：
- `core_lvl`表示 AimRT 运行时内核的日志等级，内核日志一般设为 Info 级别即可。
- `default_module_lvl`默认的模块日志等级。
- `backends`是一个数组，用于注册各个日志后端。
  - `backends[i].type`是日志后端的类型。AimRT 官方提供了几种日志后端，部分插件也提供了一些日志后端类型。部分后端允许重复注册，详情请参考对应后端类型的文档。
  - `backends[i].options`是 AimRT 传递给各个日志后端的初始化参数，这部分配置格式由各个日志后端类型定义，请参考对应日志后端类型的文档。



以下是一个简单的示例：
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


## console 控制台日志后端


`console`日志后端是 AimRT 官方提供的一种日志后端，用于将日志打印到控制台上。其所有的配置项如下：


| 节点              | 类型   | 是否可选 | 默认值                                | 作用                       |
| ----------------- | ------ | -------- | ------------------------------------- | -------------------------- |
| color             | bool   | 可选     | true                                  | 是否要彩色打印             |
| module_filter     | string | 可选     | "(.*)"                                | 模块过滤器                 |
| log_executor_name | string | 可选     | ""                                    | 日志执行器。默认使用主线程 |
| pattern           | string | 可选     | "[%c.%f][%l][%t][%n][%g:%R @%F]%v"    | 日志的输出格式             |


使用注意点如下：
- `console`日志后端不允许重复注册，一个 AimRT 实例中只允许注册一个。
- `color`配置了是否要彩色打印。此项配置有可能在一些操作系统不支持。
- `module_filter`支持以正则表达式的形式，来配置哪些模块的日志可以通过本后端处理。这与模块日志等级不同，模块日志等级是全局的、先决的、影响所有日志后端的，而这里的配置只影响本后端。
- `log_executor_name`配置了日志执行器。要求日志执行器是线程安全的，如果没有配置，则默认使用 guard 线程打印日志。
- `pattern` 通过 `"%" + 字符` 的形式来进行格式化输出，具体可输出格式如下：
  
 | 格式  | 解释                     | 举例                            |
 | ----- | ------------------------ | ------------------------------- |
 | %c    | 完整日期和时间           | 2024-03-15 14:30:45             |
 | %Y    | 年份                     | 2024                            |
 | %m    | 月份                     | 03                              |
 | %d    | 日期                     | 15                              |
 | %H    | 小时                     | 14                              |
 | %M    | 分钟                     | 30                              |
 | %S    | 秒                       | 45                              |
 | %D    | 仅日期                   | 2024-03-15                      |
 | %T    | 仅时间                   | 14:30:45                        |
 | %f    | 微秒                     | 123456                          |
 | %A    | 星期几                   | Sunday                          |
 | %a    | 星期几（简写）           | Sun                             |
 | %l    | 日志级别                 | Info                            |
 | %t    | 线程ID                   | 1234                            |
 | %n    | 模块名                   | test_module                     |
 | %G    | 文件名（只显示最后一级） | test_file.cpp                   |
 | %g    | 文件名（完整路径）       | /XX/YY/ZZ/test_file.cpp         |
 | %R    | 行号                     | 20                              |
 | %F    | 函数名                   | TestFunc                        |
 | %v    | 日志消息内容             | "This is a log message"         |
 | %其他 | 只会把其他内容显示       | 如`%q` 只显示 `q`, `%%`只显示%` |


以下是一个简单的示例：
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

`rotate_file`日志后端是 AimRT 官方提供的一种日志后端，用于将日志打印到文件中。其所有的配置项如下：


| 节点               | 类型         | 是否可选 | 默认值                                | 作用                                            |
| ------------------ | ------------ | -------- | ------------------------------------- | ----------------------------------------------- |
| path               | string       | 必选     | "./log"                               | 日志文件存放目录                                |
| filename           | string       | 必选     | "aimrt.log"                           | 日志文件基础名称                                |
| max_file_size_m    | unsigned int | 可选     | 16                                    | 日志文件最大尺寸，单位：Mb                      |
| max_file_num       | unsigned int | 可选     | 100                                   | 日志文件最大数量。0表示无上限                   |
| module_filter      | string       | 可选     | "(.*)"                                | 模块过滤器                                      |
| log_executor_name  | string       | 可选     | ""                                    | 日志执行器。默认使用主线程                      |
| pattern            | string       | 可选     | "[%c.%f][%l][%t][%n][%g:%R @%F]%v"    | 日志的输出格式                                  |
| enable_sync        | bool         | 可选     | false                                 | 是否启用定期落盘                                |
| sync_interval_ms   | unsigned int | 可选     | 30000                                 | 定期主动落盘的时间间隔，单位：ms                |
| sync_executor_name | string       | 条件必选 | ""                                    | 定期落盘的执行器。 `enable_sync 为 true 时必选` |


使用注意点如下：
- `rotate_file`日志后端允许重复注册，业务可以基于这个特点，将不同模块的日志打印到不同文件中。
- `path`配置了日志文件的存放路径。如果路径不存在，则会创建一个。如果创建失败，会抛异常。
- `filename`配置了日志文件基础名称。
- 当单个日志文件尺寸超过`max_file_size_m`配置的大小后，就会新建一个日志文件，同时将老的日志文件重命名，加上`_x`这样的后缀。
- 当日志文件数量超过`max_file_num`配置的值后，就会将最老的日志文件删除。如果配置为 0，则表示永远不会删除。
- `module_filter`支持以正则表达式的形式，来配置哪些模块的日志可以通过本后端处理。这与模块日志等级不同，模块日志等级是全局的、先决的、影响所有日志后端的，而这里的配置只影响本后端。
- `log_executor_name`配置了日志执行器。要求日志执行器是线程安全的，如果没有配置，则默认使用 guard 线程打印日志。
- `pattern` 通过 `"%" + 字符` 的形式来进行格式化输出，具体可参考[console 控制台](#console-控制台日志后端)配置中的 `pattern` 字段说明。
- `enable_sync`配置了是否启用定期落盘， 开启后数据以`sync_interval_ms`配置的时间间隔定期落盘到磁盘，以保证数据完整性， 但会降低运行性能。
- `sync_interval_ms`配置定期落盘的时间间隔，单位：ms。 请在数据完整性和性能之间设置合适大小。
- `sync_executor_name`配置定期落盘的执行器。用于定期落盘的执行器必须支持 timer scheduling。


以下是一个简单的示例：
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
