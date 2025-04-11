

# Iceoryx Plugin

## Related Links

Reference example:
- {{ '[iceoryx_plugin]({}/src/examples/plugins/iceoryx_plugin)'.format(code_site_root_path_url) }}


## Plugin Overview

**iceoryx_plugin** is a high-performance communication plugin implemented based on [iceoryx](https://github.com/eclipse-iceoryx/iceoryx). It provides a zero-copy shared memory approach for inter-process communication, enabling low-latency data transmission. It is suitable for onboard communication scenarios with strict requirements on latency and frequency.

Compiling **iceoryx_plugin** requires dependency on **libacl**, which can be installed on Ubuntu using the following command:
```bash
sudo apt-get install libacl1-dev
```

The **iceoryx_plugin** plugin communicates based on [iceoryx](https://github.com/eclipse-iceoryx/iceoryx). When using it, you must first start the iceoryx daemon Roudi. AimRT compiles Roudi by default during compilation. Users can start it by entering the following command in the terminal:
```bash
./build/AimRT/iceoryx/install/bin/iox-roudi
```

Users can enter configuration options in the command line (optional). If not specified, default configurations will be used. For Roudi configuration details, please refer to [iceoryx overview.md](https://github.com/eclipse-iceoryx/iceoryx/blob/main/doc/website/getting-started/overview.md). Importantly, configuration options allow shared memory pool configuration through TOML files. Users must ensure the allocated shared memory pool size matches actual hardware resources. Here's an example:
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

Configuration items for **iceoryx_plugin** are as follows:

| Node            | Type         | Optional | Default | Description |
| :----:          | :----:       | :----:   | :----:  | :----:      |
| shm_init_size   | unsigned int | Yes      | 1024    | Initial shared memory size leased from the shared memory pool, unit: byte |

Here's a simple example:
```yaml
channels:
  - name: example_channel
    backend: iceoryx
    config:
      shm_init_size: 2048
```


Regarding the configuration of **iceoryx_plugin**, the following points should be noted:
- `shm_init_size` represents the initial allocated shared memory size, defaulting to 1k bytes. Note that during actual operation, the data size may exceed the allocated shared memory. AimRT employs a dynamic expansion mechanism that grows at a rate of 2x until it meets the data requirements. Subsequent shared memory requests will be made according to the expanded size.

## iceoryx Type Channel Backend

The `iceoryx` type Channel backend is a Channel backend provided by **iceoryx_plugin**, used for publishing and subscribing messages through iceoryx's shared memory mechanism. The current version has no configurable items.

Here is a simple publisher example:
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
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [iceoryx]

```

Here is a simple subscriber example:
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


In the chain of data from the publisher to the subscriber in AimRT, the Iceoryx data packet format is divided into 4 parts:
- Data packet length, 4 bytes
- Serialization type, usually `pb` or `json`
- Context area
  - Number of contexts, 1 byte, maximum 255 contexts
  - context_1 key, 2 byte length + data section
  - context_2 key, 2 byte length + data section
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
