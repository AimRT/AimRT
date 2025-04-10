# Common Information

AimRT 进程在启动时需要一个配置文件，来定义各个组件的运行时表现。


## Yaml

AimRT 采用 Yaml 作为配置文件格式。YAML 是一种人类可读的数据序列化语言，通常用于编写配置文件。关于 Yaml 的语法细节，请参考互联网上的文档或[Yaml 官方网站](https://yaml.org/)。


## 不区分业务开发语言

AimRT 的配置文件不区分业务代码的开发语言，无论是 Python 还是 Cpp 都使用同一套配置标准。


## AimRT 框架配置的基本结构

AimRT 的配置文件中，在`aimrt`根节点下包含各个基本组件的配置节点，基本书写风格是小写字母+下划线，目前主要有以下这些基本组件可以配置，且所有的组件配置都是可选的：


| 节点            |   作用 |  文档 |
| ----            | ---- | ---- |
| configurator    |  配置工具的配置 | [configurator](./configurator.md) |
| plugin          |  插件配置 | [plugin](./plugin.md) |
| main_thread     |  主线程配置 | [main_thread](./main_thread.md) |
| guard_thread    |  守护线程配置 | [guard_thread](./guard_thread.md) |
| executor        |  执行器配置 | [executor](./executor.md) |
| log             |  日志配置 | [log](./log.md) |
| rpc             |  RPC 配置 | [rpc](./rpc.md) |
| channel         |  Channel 配置 | [channel](./channel.md) |
| module          |  模块配置 | [module](./module.md) |


以下是一个简单的示例，先给读者一个感性的印象。关于各个组件的详细配置方法，请参考后续章节：
```yaml
aimrt:
  configurator:
    temp_cfg_path: ./cfg/tmp
  plugin:
    plugins:
      - name: xxx_plugin
        path: ./libaimrt_xxx_plugin.so
  main_thread:
    name: main_thread
  guard_thread:
    name: guard_thread
  executor:
    executors:
      - name: work_executor
        type: asio_thread
  log:
    core_lvl: INFO
    backends:
      - type: console
  rpc:
    backends:
      - type: local
      - type: mqtt
    clients_options:
      - func_name: "(.*)"
        enable_backends: [local]
    servers_options:
      - func_name: "(.*)"
        enable_backends: [local]
  channel:
    backends:
      - type: local
      - type: mqtt
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [local]
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [local]
  module:
    pkgs:
      - path: /path/to/libxxx_pkg.so
    modules:
      - name: FooModule
        enable: True
        log_lvl: INFO
        cfg_file_path: /path/to/foo_module_cfg.yaml
      - name: BarModule
        log_lvl: WARN
```


## 业务配置

除了框架的配置，AimRT 还支持用户将业务模块的配置也以 Yaml 的形式写在同一个配置文件中，以模块名称为节点名，示例如下：
```yaml
aimrt:
  # ...

# Module custom configuration, with module name as node name
FooModule:
  key_1: val_1
  key_2: val_2

BarModule:
  xxx_array:
    - val1
    - val2
  xxx_map:
    key_1: val_1
    key_2: val_2

```

当然，如果用户不想要把业务模块配置与 AimRT 框架配置写在一个文件中，甚至不想要以 Yaml 格式来写配置，AimRT 也可以支持。具体的使用方式请参考[configurator](./configurator.md)的文档。


## 环境变量替换功能

AimRT 的配置文件支持替换环境变量。在解析配置文件前，AimRT 会将配置文件中形如`${XXX_ENV}`的字符串替换为环境变量`XXX_ENV`的值。注意，如果没有此环境变量，则会替换为字符串`null`。


## 配置文件 Dump 功能

如果使用者不确定自己的配置是否正确，可以使用 AimRT 的配置 Dump 功能，将 AimRT 解析后的完整配置文件 Dump 下来，看看和开发者自己的预期是否相符。具体可以参考[CPP运行时接口](../interface_cpp/runtime.md)中关于启动参数的章节。


## 线程绑核配置

AimRT 配置中很多地方都会有线程绑核的配置，这些配置基本都是一样的，在此处做一个集中说明。一般来说这些配置包含两个选项：

| 节点                | 类型                | 是否可选 | 默认值 | 作用 |
| ----                | ----                | ----    | ----  | ---- |
| thread_sched_policy | string              | 可选    | ""    | 线程调度策略 |
| thread_bind_cpu     | unsigned int array  | 可选    | []    | 绑核配置 |

使用注意点如下：
- `thread_sched_policy`配置了线程调度策略，通过调用操作系统的 API 来实现。目前仅在 Linux 下支持，在其他操作系统上此配置无效。
  - 在 Linux 下通过调用`pthread_setschedparam`这个 API 来配置。支持的方式包括：`SCHED_OTHER`、`SCHED_FIFO:xx`、`SCHED_RR:xx`。`xx`为该模式下的权重值。详细的解释请参考[pthread_setschedparam官方文档](https://man7.org/linux/man-pages/man3/pthread_setschedparam.3.html)。
- `thread_bind_cpu`配置了绑核策略，通过调用操作系统的 API 来实现。目前仅在 Linux 下支持，在其他操作系统上此配置无效。
  - 在 Linux 下通过调用`pthread_setaffinity_np`这个 API 来配置，直接在数组中配置 CPU ID 即可。参考[pthread_setaffinity_np官方文档](https://man7.org/linux/man-pages/man3/pthread_setaffinity_np.3.html)。
