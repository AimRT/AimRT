# Iceoryx Plugin

## Related Links

Reference example:

- {{ '[iceoryx_plugin]({}/src/examples/plugins/iceoryx_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**iceoryx_plugin** is a high-performance communication plugin based on [iceoryx](https://github.com/eclipse-iceoryx/iceoryx), providing a zero-copy shared-memory approach for inter-process communication. This enables low-latency data transmission and is suitable for Agibot communication scenarios that have strict requirements on latency and frequency.

Compiling **iceoryx_plugin** depends on **libacl**, which can be installed on Ubuntu with:


```shell
sudo apt install libacl1-dev
```


The **iceoryx_plugin** communicates via [iceoryx](https://github.com/eclipse-iceoryx/iceoryx). Before use, you must first start the daemon Roudi provided by iceoryx; AimRT also builds Roudi by default during compilation. Users can start it in a terminal with:


```bash
./iox-roudi --config-file=/path/to/you/cfg/iox_cfg.toml
```


Users can input configuration options on the command line; this is optional—if omitted, the default configuration is used. For details on configuring Roudi, refer to [iceoryx overview.md](https://github.com/eclipse-iceoryx/iceoryx/blob/main/doc/website/getting-started/overview.md). It is important to note that the configuration options allow the shared-memory pool to be tuned via a TOML file. Before use, be sure the allocated shared-memory pool size matches the actual hardware resources. An example:


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

- `iceoryx`-type Channel backend

**iceoryx_plugin** configuration items:

|     Node      |     Type     | Optional |     Default      |                         Purpose                          |
| :-----------: | :----------: | :------: | :--------------: | :------------------------------------------------------: |
| shm_init_size | unsigned int |   Yes    |       1024       | Initial shared-memory size leased from the pool (bytes) |
|  runtime_id   |    string    |   Yes    | 'iceoryx' + <pid> | Unique identifier for the current iceoryx runtime node |

A simple example:


```yaml
aimrt:
  plugin:
    plugins:
      - name: iceoryx_plugin
        path: ./libaimrt_iceoryx_plugin.so
        options:
          shm_init_size: 2048
          runtime_id: "test_node"
```


Configuration notes for **iceoryx_plugin**:

- `shm_init_size` is the initial shared-memory size allocated, default 1 kB. Note that during actual operation the data size may exceed the allocated memory. AimRT uses a dynamic expansion mechanism that doubles the size until the data requirement is met; subsequent allocations will use the expanded size.
- `runtime_id` is the unique identifier for the iceoryx runtime node. The same runtime_id cannot be used on the same machine. By default, runtime_id is 'iceoryx' + <pid>, where <pid> is the current process ID.

## iceoryx-type Channel Backend

The `iceoryx`-type Channel backend is a Channel backend provided by **iceoryx_plugin**, used to publish and subscribe to messages via iceoryx’s shared-memory mechanism. All configuration items are:

| Parameter                    | Type   | Optional | Default | Purpose                                      |
| ---------------------------- | ------ | -------- | ------- | -------------------------------------------- |
| listener_thread_name         | string | Yes      | ""      | Subscriber listener thread name              |
| listener_thread_sched_policy | string | Yes      | ""      | Subscriber listener thread scheduling policy |
| listener_thread_bind_cpu     | array  | Yes      | []      | CPU core list bound to subscriber listener   |

These configuration entries only take effect when there are subscribers; they are ignored when no subscribers exist in the process.

A simple publisher example:


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
          listener_thread_bind_cpu: [5, 6, 7]
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [iceoryx]
```


A simple subscriber example:


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


In the AimRT publisher-to-subscriber data path, the Iceoryx packet format is divided into four segments:

- Packet length, 4 bytes
- Serialization type, usually `pb` or `json`
- Context area
  - Context count, 1 byte, up to 255 contexts
  - context_1 key, 2-byte length + data
  - context_2 key, 2-byte length + data
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
