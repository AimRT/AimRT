# Common Information

When an AimRT process starts, it requires a configuration file to define the runtime behavior of each component.


## Yaml

AimRT uses Yaml as its configuration file format. YAML is a human-readable data serialization language commonly used for writing configuration files. For details on Yaml syntax, please refer to online documentation or the [Yaml official website](https://yaml.org/).


## Language-agnostic for Business Development

AimRT's configuration files are independent of the business code's development language; both Python and Cpp use the same configuration standard.


## Basic Structure of AimRT Framework Configuration

In AimRT configuration files, under the `aimrt` root node, there are configuration nodes for various basic components. The naming convention uses lowercase letters with underscores. Currently, the following basic components can be configured, and all component configurations are optional:


| Node            |   Purpose |  Documentation |
| ----            | ---- | ---- |
| configurator    |  Configuration tool settings | [configurator](./configurator.md) |
| plugin          |  Plugin configuration | [plugin](./plugin.md) |
| main_thread     |  Main thread configuration | [main_thread](./main_main_thread.md) |
| guard_thread    |  Guard thread configuration | [guard_thread](./guard_thread.md) |
| executor        |  Executor configuration | [executor](./executor.md) |
| log             |  Log configuration | [log](./log.md) |
| rpc             |  RPC configuration | [rpc](./rpc.md) |
| channel         |  Channel configuration | [channel](./channel.md) |
| module          |  Module configuration | [module](./module.md) |


Below is a simple example to give readers an intuitive impression. For detailed configuration methods of each component, please refer to subsequent chapters:

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



## Business Configuration

In addition to framework configuration, AimRT also supports users writing business module configurations in Yaml format within the same configuration file, using the module name as the node name. Example below:

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


Of course, if users do not want to write business module configurations together with AimRT framework configurations in one file, or even do not want to use Yaml format for configuration, AimRT can also support this. For specific usage methods, please refer to the documentation in [configurator](./configurator.md).


## Environment Variable Substitution

AimRT configuration files support environment variable substitution. Before parsing the configuration file, AimRT will replace strings in the format `${XXX_ENV}` in the configuration file with the value of the environment variable `XXX_ENV`. Note that if this environment variable does not exist, it will be replaced with the string `null`.


## Configuration File Dump Feature

If users are unsure whether their configuration is correct, they can use AimRT's configuration dump feature to dump the complete configuration file parsed by AimRT and check if it matches the developer's expectations. For details, please refer to the chapter on startup parameters in [CPP Runtime Interface](../interface_cpp/runtime.md).


## Thread Core Binding Configuration

Many places in AimRT configuration will have thread core binding configurations, which are basically the same and will be explained here collectively. Generally, these configurations include two options:

| Node                | Type                | Optional | Default | Purpose |
| ----                | ----                | ----    | ----  | ---- |
| thread_sched_policy | string              | Optional    | ""    | Thread scheduling policy |
| thread_bind_cpu     | unsigned int array  | Optional    | []    | Core binding configuration |

Usage notes:
- `thread_sched_policy` configures the thread scheduling policy, implemented by calling operating system APIs. Currently only supported on Linux, this configuration is invalid on other operating systems.
  - On Linux, it is configured by calling the `pthread_setschedparam` API. Supported modes include: `SCHED_OTHER`, `SCHED_FIFO:xx`, `SCHED_RR:xx`. `xx` is the priority value under this mode. For detailed explanations, please refer to the [pthread_setschedparam official documentation](https://man7.org/linux/man-pages/man3/pthread_setschedparam.3.html).
- `thread_bind_cpu` configures the core binding policy, implemented by calling operating system APIs. Currently only supported on Linux, this configuration is invalid on other operating systems.
  - On Linux, it is configured by calling the `pthread_setaffinity_np` API, simply configure the CPU ID in the array. Refer to [pthread_setaffinity_np official documentation](https://man7.org/linux/man-pages/man3/pthread_setaffinity_np.3.html).