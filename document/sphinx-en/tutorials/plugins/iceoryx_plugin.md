# Iceoryx Plugin


## Related Links

Reference Example:
- {{ '[iceoryx_plugin]({}/src/examples/plugins/iceoryx_plugin)'.format(code_site_root_path_url) }}


## Plugin Overview

**iceoryx_plugin** is a high-performance communication plugin based on [iceoryx](https://github.com/eclipse-iceoryx/iceoryx). It provides a zero-copy shared memory approach for inter-process communication, enabling low-latency data transmission. It is suitable for scenarios with high requirements on latency and frequency in native communication.

Compiling **iceoryx_plugin** requires **libacl** as a dependency. On Ubuntu, it can be installed with the following command:
```shell
sudo apt install libacl1-dev
```

The **iceoryx_plugin** plugin communicates via [iceoryx](https://github.com/eclipse-iceoryx/iceoryx). When using it, you must first start the Roudi daemon provided by iceoryx. AimRT also compiles Roudi by default during compilation. Users can start it by entering the following command in the terminal:
```bash
./iox-roudi --config-file=/path/to/you/cfg/iox_cfg.toml
```

Users can input configuration options in the command line, which is optional. If no input is provided, default configurations will be used. For detailed Roudi configuration, refer to [iceoryx overview.md](https://github.com/eclipse-iceoryx/iceoryx/blob/main/doc/website/getting-started/overview.md). It's important to note that the shared memory pool can be configured via a toml configuration file. Before use, ensure the allocated shared memory pool size matches the actual hardware resources. Here's an example:
```toml
# Adapt this config to your needs and rename it to e.g. roudi_config.toml
# Reference to https://github.com/eclipse-iceoryx/iceoryx/blob/main/iceoryx_posh/etc/iceoryx/roudi_config_example.toml

[general]
version = 1

[[segment]]

[[segment.mempool]]
size = 128
count = 10000

[[segment.mempool]]
size = 1024
count = 5000

[[segment.mempool]]
size = 16384
count = 1000

[[segment.mempool]]
size = 131072
count = 200
```

**iceoryx_plugin** provides the following components:
- `iceoryx` type Channel backend


Configuration options for **iceoryx_plugin** are as follows:

|      Node      |     Type     | Optional | Default |                     Purpose                     |
| :-----------: | :----------: | :------: | :----: | :----------------------------------------------: |
| shm_init_size | unsigned int |   Yes    |  1024  | Initial leased shared memory size (unit: bytes) |


Here's a simple example:
```yaml
aimrt:
  plugin:
    plugins:
      - name: iceoryx_plugin
        path: ./libaimrt_iceoryx_plugin.so
        options:
          shm_init_size: 2048
```


Notes on configuring **iceoryx_plugin**:
- `shm_init_size` indicates the initially allocated shared memory size, defaulting to 1k bytes. Note that during actual operation, data size may exceed the allocated shared memory. AimRT employs a dynamic expansion mechanism that doubles the size until it meets data requirements. Subsequent shared memory requests will use the expanded size.



## iceoryx Type Channel Backend


The `iceoryx` type Channel backend is a Channel backend provided by **iceoryx_plugin**, used for publishing and subscribing messages via iceoryx's shared memory approach. Its configuration options are as follows:

| Node Parameter               | Type   | Optional | Default | Purpose                          |
| ---------------------------- | ------ | -------- | ------ | ------------------------------- |
| listener_thread_name         | string | Yes      | ""     | Subscriber listener thread name |
| listener_thread_sched_policy | string | Yes      | ""     | Subscriber thread scheduling policy |
| listener_thread_bind_cpu     | array  | Yes      | []     | List of CPU cores bound to subscriber thread |

The above configuration items only take effect when there are subscribers. They are invalid if no subscribers exist in the process.


Here's a simple publisher example:
```yaml
aimrt:
  plugin:
    plugins:
      - name: iceoryx_plugin
        path: ./libaimrt_iceoryx_plugin.so
        options:
          shm_init_size: 2048
  channel:
    backends:
      - type: iceoryx
        options:
          listener_thread_name: "iceoryx_listener"
          listener_thread_sched_policy: "SCHED_FIFO:50"
          listener_thread_bind_cpu: [5,6,7]
    pub_topics_options:
      - topic_name: "(.*)" 
        enable_backends: [iceoryx]

```

Here's a simple subscriber example:
```yaml
aimrt:
  plugin:
    plugins:
      - name: iceoryx_plugin
        path: ./libaimrt_iceoryx_plugin.so
        options:
          shm_init_size: 2048
  executor:
  channel:
    backends:
      - type: iceoryx
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [iceoryx]
```


In the data transmission chain from AimRT publisher to subscriber, the Iceoryx packet format consists of 4 segments:
- Packet length (4 bytes)
- Serialization type (typically `pb` or `json`)
- Context section
  - Number of contexts (1 byte, max 255 contexts)
  - context_1 key (2-byte length + data section)
  - context_2 key (2-byte length + data section)
  - ...
- Data

```
| msg len [4 byte]
| n(0~255) [1 byte] | content type [n byte]
| context num [1 byte]
| context_1 key size [2 byte] | context_1 key data [key_1_size byte]
| context_1 val size [2 byte] | context_1 val data [val_1_size byte]
| context_2 key size [2 byte] | context_2 key data [key_2_size byte]
| context_2 val size [2 byte] | context_2 val data [val_2_size byte]
| ...
| msg data [len - 1 - n byte] |
```