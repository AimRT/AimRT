# Common Information

The AimRT process requires a configuration file at startup to define the runtime behavior of various components.

## Yaml

AimRT uses YAML as the configuration file format. YAML is a human-readable data serialization language commonly used for writing configuration files. For details on YAML syntax, please refer to online documentation or the [YAML official website](https://yaml.org/).

## Language-Agnostic for Business Development

AimRT's configuration file does not distinguish between the development languages of business code. Whether it's Python or Cpp, the same configuration standard is used.

## Basic Structure of AimRT Framework Configuration

In AimRT's configuration file, under the `aimrt` root node, there are configuration nodes for various basic components. The writing style primarily uses lowercase letters with underscores. Currently, the following basic components can be configured, and all component configurations are optional:

| Node            | Purpose | Documentation |
| ----            | ---- | ---- |
| configurator    | Configuration tool settings | [configurator](./configurator.md) |
| plugin          | Plugin configuration | [plugin](./plugin.md) |
| main_thread     | Main thread configuration | [main_thread](./main_thread.md) |
| guard_thread    | Guard thread configuration | [guard_thread](./guard_thread.md) |
| executor        | Executor configuration | [executor](./executor.md) |
| log             | Logging configuration | [log](./log.md) |
| rpc             | RPC configuration | [rpc](./rpc.md) |
| channel         | Channel configuration | [channel](./channel.md) |
| module          | Module configuration | [module](./module.md) |

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

In addition to framework configuration, AimRT also supports users writing business module configurations in the same configuration file in YAML format, using the module name as the node name. Example:
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

Of course, if users prefer not to combine business module configurations with AimRT framework configurations in a single file, or even not to use YAML format for configurations, AimRT also supports this. For specific usage, please refer to the [configurator](./configurator.md) documentation.

## Environment Variable Substitution Feature

AimRT's configuration file supports environment variable substitution. Before parsing the configuration file, AimRT will replace strings in the format `${XXX_ENV}` with the value of the environment variable `XXX_ENV`. Note that if the environment variable does not exist, it will be replaced with the string `null`.

## Configuration File Dump Feature

If users are unsure whether their configuration is correct, they can use AimRT's configuration dump feature to output the complete configuration file after parsing by AimRT, to check if it matches their expectations. For details, refer to the startup parameters section in the [CPP Runtime Interface](../interface_cpp/runtime.md) documentation.

## Thread Core Binding Configuration

Many places in AimRT's configuration involve thread core binding settings, which are generally the same. Here is a consolidated explanation. Typically, these configurations include two options:

| Node                | Type                | Optional | Default Value | Purpose |
| ----                | ----                | ----    | ----  | ---- |
| thread_sched_policy | string              | Yes    | ""    | Thread scheduling policy |
| thread_bind_cpu     | unsigned int array  | Yes    | []    | Core binding configuration |

Usage notes:
- `thread_sched_policy` configures the thread scheduling policy by calling the operating system's API. Currently, this is only supported on Linux and is ineffective on other operating systems.
  - On Linux, this is configured by calling the `pthread_setschedparam` API. Supported modes include: `SCHED_OTHER`, `SCHED_FIFO:xx`, `SCHED_RR:xx`. `xx` represents the priority value in that mode. For detailed explanations, refer to the [pthread_setschedparam official documentation](https://man7.org/linux/man-pages/man3/pthread_setschedparam.3.html).
- `thread_bind_cpu` configures the core binding policy by calling the operating system's API. Currently, this is only supported on Linux and is ineffective on other operating systems.
  - On Linux, this is configured by calling the `pthread_setaffinity_np` API. Simply specify the CPU IDs in the array. Refer to the [pthread_setaffinity_np official documentation](https://man7.org/linux/man-pages/man3/pthread_setaffinity_np.3.html).