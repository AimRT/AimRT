

# Common Information

The AimRT process requires a configuration file at startup to define runtime behaviors of various components.

## Yaml

AimRT uses YAML as its configuration file format. YAML is a human-readable data serialization language commonly used for writing configuration files. For details about YAML syntax, please refer to online documentation or the [YAML official website](https://yaml.org/).

## Language Agnostic Configuration

AimRT's configuration file is independent of the business code's development language. The same configuration standard applies whether using Python or Cpp.

## Basic Structure of AimRT Framework Configuration

In AimRT's configuration file, the `aimrt` root node contains configuration nodes for various core components. The writing style follows lowercase letters with underscores. Currently available configurable components (all optional) include:

| Node            | Function | Documentation |
| ----            | ---- | ---- |
| configurator    | Configuration tool settings | [configurator](./configurator.md) |
| plugin          | Plugin configuration | [plugin](./plugin.md) |
| main_thread     | Main thread settings | [main_thread](./main_thread.md) |
| guard_thread    | Guard thread settings | [guard_thread](./guard_thread.md) |
| executor        | Executor configuration | [executor](./executor.md) |
| log             | Logging configuration | [log](./log.md) |
| rpc             | RPC settings | [rpc](./rpc.md) |
| channel         | Channel configuration | [channel](./channel.md) |
| module          | Module settings | [module](./module.md) |

Below is a simple example to provide initial understanding. Detailed configuration methods for each component can be found in subsequent chapters:
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

In addition to framework configuration, AimRT allows users to write business module configurations in the same YAML file using module names as node names. Example:
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

For users preferring separate configuration files or non-YAML formats, AimRT also provides support. Refer to [configurator](./configurator.md) documentation for details.

## Environment Variable Substitution

AimRT configuration files support environment variable substitution. Before parsing, strings formatted as `${XXX_ENV}` will be replaced with the value of environment variable `XXX_ENV`. Note: If the environment variable doesn't exist, it will be replaced with string `null`.

## Configuration Dump Feature

To verify configuration correctness, users can utilize AimRT's dump feature to output the fully parsed configuration file. Refer to [CPP Runtime Interface](../interface_cpp/runtime.md) documentation for startup parameter details.

## CPU Core Binding Configuration

Many configurations in AimRT involve thread-core binding settings. These configurations typically contain two options:

| Node                | Type                | Optional | Default | Function |
| ----                | ----                | ----    | ----  | ---- |
| thread_sched_policy | string              | Yes     | ""    | Thread scheduling policy |
| thread_bind_cpu     | unsigned int array  | Yes     | []    | Core binding configuration |

Usage notes:
- `thread_sched_policy` configures thread scheduling policies using OS APIs. Currently only supported on Linux.
  - Implemented via `pthread_setschedparam` on Linux. Supported values: `SCHED_OTHER`, `SCHED_FIFO:xx`, `SCHED_RR:xx` (where `xx` is priority). Details: [pthread_setschedparam docs](https://man7.org/linux/man-pages/man3/pthread_setschedparam.3.html).
- `thread_bind_cpu` configures core binding via OS APIs. Currently only supported on Linux.
  - Implemented via `pthread_setaffinity_np` on Linux. Configure CPU IDs directly in the array. Reference: [pthread_setaffinity_np docs](https://man7.org/linux/man-pages/man3/pthread_setaffinity_np.3.html).