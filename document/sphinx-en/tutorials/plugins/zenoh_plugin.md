# Zenoh Plugin

## Related Links

Reference example:
- {{ '[zenoh_plugin]({}/src/examples/plugins/zenoh_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**zenoh_plugin** is a lightweight, efficient, real-time data transmission plugin designed to provide low-latency, high-throughput data transmission and processing capabilities for distributed systems. The zenoh plugin is particularly suitable for the following business scenarios:
- Communication systems with `service discovery` mechanisms;
- Flexible network topologies;
- Low-latency, high-throughput network communication and data transmission;
- Both SHM and non-SHM transmission modes;

This plugin provides the following components for AimRT:
- `zenoh` type Rpc backend
- `zenoh` type Channel backend

The configuration items for the plugin are as follows:

|         Node        |  Type  | Optional | Default |            Function            |
| :----------------: | :----: | :------: | :----: | :-----------------------------: |
|   shm_pool_size    |  int   |   Yes   | 10 MB  |   Shared memory pool size, unit: B   |
| shm_init_loan_size |  int   |   Yes   |  1 KB  | Initial shared memory loan size, unit: B |
|  native_cfg_path   | string |   Yes   |   ""   |   Native configuration file provided by zenoh   |
|    limit_domain    | string |   Yes   |   ""   |     Restrict the communication domain of the plugin      |

Regarding the configuration of **zenoh_plugin**, the following points should be noted:
- `shm_pool_size` indicates the size of the shared memory pool, unit: B, default value is 10 MB, which can be adjusted according to actual needs. If shared memory is not used, this configuration item can be ignored. If the remaining shared memory is insufficient to meet data transmission requirements, it will automatically switch to non-shared memory transmission mode.
- `shm_init_loan_size` indicates the initial loan size from the shared memory pool, unit: B, default value is 1 KB, which can be adjusted according to actual needs. If shared memory is not used, this configuration item can be ignored.
- `native_cfg_path` indicates the path to the native configuration file provided by zenoh. This file can be configured to flexibly set up zenoh's network structure. If left blank, the default configuration provided by zenoh will be used. For specific configuration details, please refer to zenoh's official documentation on [configuration](https://zenoh.io/docs/manual/configuration/). You can also directly modify the {{ '[zenoh_native_config.json5]({}/src/examples/plugins/zenoh_plugin/install/linux/bin/cfg/zenoh_native_config.json5)'.format(code_site_root_path_url) }} file for custom configuration. Here are some commonly used configuration items:

|            Configuration Item            |                                 Function                                 | Configuration Value in zenoh_native_config |
| :---------------------------: | :-------------------------------------------------------------------: | :---------------------------: |
|  scouting.multicast. enabled  |           Whether to enable multicast, allowing zenoh nodes to automatically discover each other           |             true              |
|  scouting.multicast. address  |                             Configure multicast address                              |       224.0.0.224:7446        |
| scouting.multicast. interface |                          Configure the network interface to use                           |             auto              |
|       listen.endpoints        |                          Addresses to actively listen on                           |               -               |
| transport.unicast.lowlatency  | Whether to enable minimum latency, which helps improve transmission speed (note: cannot be enabled simultaneously with QoS) |             false             |
| transport.unicast.qos.enabled |           Whether to enable Quality of Service (note: cannot be enabled simultaneously with lowlatency)            |             true              |

- `limit_domain` indicates the communication domain of the plugin, which is compatible with zenoh's powerful Key & Key Expression. If left blank, the plugin's default communication domain (i.e., the message topic) will be used. Only domains that `match` can communicate. The specific format is as follows:

```shell

#请不要以"/"开始，中间以"/"分隔，结尾不要带"/" （与zenoh官方书写方式一致），如：

xxx/yyy/zzz

```
  The simplest matching is when both domains are identical. Additionally, zenoh provides more flexible matching mechanisms. For details, please refer to zenoh's official documentation on [key](https://zenoh.io/docs/manual/abstractions/).

Here is a simple example:

```yaml
aimrt:
  plugin:
    plugins:
      - name: zenoh_plugin
        path: ./libaimrt_zenoh_plugin.so
        options:
          native_cfg_path: ./cfg/zenoh_native_config.json5
```
## zenoh Type Rpc Backend
The `zenoh` type Rpc backend is a type of Rpc backend provided in **zenoh_plugin**, primarily used to build a request-response model. All its configuration items are as follows:

| Node                           | Type   | Optional | Default | Description                           |
| ------------------------------ | ------ | -------- | ------- | ------------------------------------- |
| timeout_executor               | string | Optional | ""      | Executor for RPC timeout on Client side |
| clients_options                | array  | Optional | []      | Rules for Client-side RPC requests    |
| clients_options[i].func_name   | string | Required | ""      | RPC Func name, supports regex         |
| clients_options[i].shm_enabled | bool   | Optional | false   | Whether RPC Func uses shared memory communication |
| servers_options                | array  | Optional | []      | Rules for Server-side RPC request handling |
| servers_options[i].func_name   | string | Required | ""      | RPC Func name, supports regex         |
| servers_options[i].shm_enabled | bool   | Optional | false   | Whether RPC Func uses shared memory communication |


Here is a simple client example:

```yaml
aimrt:
  plugin:
    plugins:
      - name: zenoh_plugin
        path: ./libaimrt_zenoh_plugin.so
        options:
          native_cfg_path: ./cfg/zenoh_native_config.json5
          shm_pool_size: 10240
  executor:
    executors:
      - name: timeout_handle
        type: time_wheel
        options:
          bind_executor: work_thread_pool
  rpc:
    backends:
      - type: zenoh
        options:
          timeout_executor: timeout_handle
          clients_options:
            - func_name: "(.*)"
              shm_enabled: false
    clients_options:
      - func_name: "(.*)"
        enable_backends: [zenoh]

```

Here is a simple server example:

```yaml

aimrt:
  plugin:
    plugins:
      - name: zenoh_plugin
        path: ./libaimrt_zenoh_plugin.so
        options:
          native_cfg_path: ./cfg/zenoh_native_config.json5
  rpc:
    backends:
      - type: zenoh
        options:
          timeout_executor: timeout_handle
          servers_options:
            - func_name: "(.*)"
              shm_enabled: true
    servers_options:
      - func_name: "(.*)"
        enable_backends: [zenoh]

```
In the above examples, both the Client and Server use a service discovery mechanism, meaning endpoints in the same network can automatically discover each other and establish connections.

During the entire RPC process, the underlying Zenoh Topic names follow this format:
- Server side
  - Topic for subscribing to Req:
    - `req/aimrt_rpc/${func_name}/${limit_domain}>`
  - Topic for publishing Rsp: `rsp/aimrt_rpc/${func_name}/${limit_domain}`
- Client side
  - Topic for publishing Req:
    - `req/aimrt_rpc/${func_name}/${limit_domain}`
  - Topic for subscribing to Rsp: `rsp_/imrt_rpc/${func_name}/${limit_domain}`

`${func_name}` is the URL-encoded AimRT RPC method name.

For example, for a client request with a func name of `/aimrt.protocols.example.ExampleService/GetBarData` and no limit_domain configured, the `final topic name` would be: `req/aimrt_rpc/%2Faimrt.protocols.example.ExampleService%2FGetBarData`.

The Zenoh packet format from Client -> Server consists of 5 segments:
- Serialization type, usually `pb` or `json`
- The Zenoh topic name the client wants the server to reply to. The client must subscribe to this Zenoh topic.
- msg id, 4 bytes, which the server will include unchanged in the rsp packet for the client to match the rsp to the req.
- Context section
  - Number of contexts, 1 byte (max 255 contexts)
  - context_1 key, 2-byte length + data section
  - context_2 key, 2-byte length + data section
  - ...
- msg data

```
| n(0~255) [1 byte] | content type [n byte]
| m(0~255) [1 byte] | rsp topic name [m byte]
| msg id [4 byte]
| context num [1 byte]
| context_1 key size [2 byte] | context_1 key data [key_1_size byte]
| context_1 val size [2 byte] | context_1 val data [val_1_size byte]
| context_2 key size [2 byte] | context_2 key data [key_2_size byte]
| context_2 val size [2 byte] | context_2 val data [val_2_size byte]
| ...
| msg data [remaining byte]
```

The Zenoh packet format from Server -> Client consists of 4 segments:
- Serialization type, usually `pb` or `json`
- msg id, 4 bytes, same as the req's msg id
- status code, 4 bytes, framework error code. If this is non-zero, it indicates a server error, and the data section will be empty.
- msg data

```
| n(0~255) [1 byte] | content type [n byte]
| msg id [4 byte]
| status code [4 byte]
| msg data [remaining byte]
```
## zenoh Type Channel Backend

The `zenoh` type Channel backend is a Channel backend provided in **zenoh_plugin**, primarily used to build a publish-subscribe model. All its configuration items are as follows:

| Node                              | Type   | Optional | Default | Description                     |
| --------------------------------- | ------ | -------- | ------ | ------------------------------- |
| pub_topics_options                | array  | Optional | []     | Rules for publishing Topics     |
| pub_topics_options[i].topic_name  | string | Required | ""     | Topic name, supports regex      |
| pub_topics_options[i].shm_enabled | bool   | Required | false  | Whether the Publisher uses shared memory communication |


Here is a simple example for the publisher side:
```yaml
aimrt:
  plugin:
    plugins:
      - name: zenoh_plugin
        path: ./libaimrt_zenoh_plugin.so
        options:
          shm_pool_size: 1024
  channel:
    backends:
      - type: zenoh
        options:
          pub_topics_options:
            - topic_name: "(.*)"
              shm_enabled: false
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [zenoh]
```

Here is a simple example for the subscriber side:
```yaml
aimrt:
  plugin:
    plugins:
      - name: zenoh_plugin
        path: ./libaimrt_zenoh_plugin.so
        options:
          native_cfg_path: ./cfg/zenoh_native_config.json5
channel:
    backends:
      - type: zenoh
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [zenoh]
```

Both examples above use zenoh's service discovery mechanism, where two endpoints in the same network can automatically discover each other and establish a connection.

In this process, the underlying Topic name format is: `channel/${topic_name}/${message_type}${limit_domain}`. Here, `${topic_name}` is the AimRT Topic name, `${message_type}` is the URL-encoded AimRT message name, and `${limit_domain}` is the plugin's limit domain. This Topic is set as Zenoh's final key expression (Keyxpr), which is Zenoh's resource identifier. Only subscribers and publishers with matching key expressions can communicate.

For example, if the AimRT Topic name is `test_topic`, the message type is `pb:aimrt.protocols.example.ExampleEventMsg`, and the limit domain is `room1/A2`, then the final Zenoh Topic name will be: `channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsgroom1/A2`. If both the subscriber and publisher have the Topic set to `channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsgroom1/A2`, they can communicate.

On the data transmission path from the AimRT publisher to the subscriber, the Zenoh packet format consists of three segments:
- Serialization type, usually `pb` or `json`
- Context section
  - Number of contexts, 1 byte (maximum 255 contexts)
  - context_1 key, 2-byte length + data section
  - context_2 key, 2-byte length + data section
  - ...
- Data

The packet format is as follows:
```
| n(0~255) [1 byte] | content type [n byte]
| context num [1 byte]
| context_1 key size [2 byte] | context_1 key data [key_1_size byte]
| context_1 val size [2 byte] | context_1 val data [val_1_size byte]
| context_2 key size [2 byte] | context_2 key data [key_2_size byte]
| context_2 val size [2 byte] | context_2 val data [val_2_size byte]
| ...
| msg data [len - 1 - n byte] |
```