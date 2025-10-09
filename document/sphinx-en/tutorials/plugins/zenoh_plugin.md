# Zenoh Plugin


## Related Links

Reference example:
- {{ '[zenoh_plugin]({}/src/examples/plugins/zenoh_plugin)'.format(code_site_root_path_url) }}


## Plugin Overview

**zenoh_plugin** is a lightweight, efficient, real-time data transmission plugin designed to provide low-latency, high-throughput data transmission and processing capabilities for distributed systems. The zenoh plugin is particularly suitable when the following business scenarios are required:
- Communication systems with a `service discovery` mechanism;
- Flexible network topologies;
- Low-latency, high-throughput network communication and data transmission;
- Both SHM and non-SHM transmission modes;

This plugin provides the following components for AimRT:
- `zenoh`-type Rpc backend
- `zenoh`-type Channel backend


Plugin configuration items are as follows:

|        Node        |  Type  | Optional | Default |              Function               |
| :----------------: | :----: | :------: | :----: | :-----------------------------: |
|   shm_pool_size    |  int   |   Optional   | 10 MB  |   Size of the shared memory pool, unit: B   |
| shm_init_loan_size |  int   |   Optional   |  1 KB  | Initial loan size from shared memory, unit: B |
|  native_cfg_path   | string |   Optional   |   ""   |   Path to zenoh's native configuration file   |
|    limit_domain    | string |   Optional   |   ""   |     Restrict the communication domain of the plugin      |


Regarding the configuration of **zenoh_plugin**, the following points should be noted:
- `shm_pool_size` indicates the size of the shared memory pool, unit: B, default is 10 MB, which can be adjusted according to actual needs. If shared memory is not used, this configuration item can be ignored. If the remaining shared memory is insufficient to meet data transmission requirements, it will automatically switch to non-shared memory transmission mode.
- `shm_init_loan_size` indicates the initial loan size from the shared memory pool, unit: B, default is 1 KB, which can be adjusted according to actual needs. If shared memory is not used, this configuration item can be ignored.
- `native_cfg_path` indicates the path to the native configuration file provided by zenoh. You can flexibly configure zenoh's network structure through this file. If not specified, the default configuration provided by zenoh will be used. For specific configuration content, please refer to zenoh's official documentation on [configuration](https://zenoh.io/docs/manual/configuration/). You can also directly modify the {{ '[zenoh_native_config.json5]({}/src/examples/plugins/zenoh_plugin/install/linux/bin/cfg/zenoh_native_config.json5)'.format(code_site_root_path_url) }} file to customize the configuration. Here are some commonly used configuration items:

|            Configuration Item             |                                 Function                                  | Configuration Value in zenoh_native_config |
| :---------------------------: | :-------------------------------------------------------------------: | :---------------------------: |
|  scouting.multicast. enabled  |           Whether to enable multicast, allowing multiple zenoh nodes to discover each other automatically           |             true              |
|  scouting.multicast. address  |                             Configure multicast address                              |       224.0.0.224:7446        |
| scouting.multicast. interface |                          Configure the network interface used                           |             auto              |
|       listen.endpoints        |                          Addresses to actively listen on                           |               -               |
| transport.unicast.lowlatency  | Whether to enable lowest latency, which helps improve transmission speed. Note that it cannot be enabled simultaneously with qos |             false             |
| transport.unicast.qos.enabled |           Whether to enable quality of service. Note that it cannot be enabled simultaneously with lowlatency            |             true              |


- limit_domain indicates the communication domain of the plugin, compatible with zenoh's powerful Key & Key Expression. If not specified, the plugin's default communication domain (i.e., the message's topic) will be used. Only domains that `match` can communicate. The specific format is as follows:


```shell

#请不要以"/"开始，中间以"/"分隔，结尾不要带"/" （与zenoh官方书写方式一致），如：

xxx/yyy/zzz

```

  The simplest match is when the two domains are identical. In addition, zenoh's official documentation provides more flexible matching mechanisms. For details, please refer to zenoh's official explanation of [key](https://zenoh.io/docs/manual/abstractions/).

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
The `zenoh` type Rpc backend is an Rpc backend provided in **zenoh_plugin**, mainly used to build a request-response model. All its configuration items are as follows:

| Node                           | Type   | Optional | Default | Purpose                                 |
| ------------------------------ | ------ | -------- | ------- | --------------------------------------- |
| timeout_executor               | string | Optional | ""      | Executor when the Client initiates RPC timeout |
| clients_options                | array  | Optional | []      | Rules when the Client initiates RPC requests |
| clients_options[i].func_name   | string | Required | ""      | RPC Func name, supports regular expressions |
| clients_options[i].shm_enabled | bool   | Optional | false   | Whether the RPC Func uses shared memory communication |
| servers_options                | array  | Optional | []      | Rules for the server when handling RPC requests |
| servers_options[i].func_name   | string | Required | ""      | RPC Func name, supports regular expressions |
| servers_options[i].shm_enabled | bool   | Optional | false   | Whether the RPC Func uses shared memory communication |


Below is a simple client example:


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


Below is a simple server example:


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

In the above examples, both the Client and Server use service discovery, meaning two endpoints in the same network can automatically discover each other and establish a connection.

Throughout the RPC process, the underlying Zenoh Topic name format is as follows:
- Server side
  - Topic for subscribing to Req:
    - `req/aimrt_rpc/${func_name}/${limit_domain}>`
  - Topic for publishing Rsp: `rsp/aimrt_rpc/${func_name}/${limit_domain}`
- Client side
  - Topic for publishing Req:
    - `req/aimrt_rpc/${func_name}/ ${limit_domain}`
  - Topic for subscribing to Rsp: `rsp_/imrt_rpc/${func_name}/${limit_domain}`

`${func_name}` is the URL-encoded AimRT RPC method name.


For example, for a client request, if the func name is `/aimrt.protocols.example.ExampleService/GetBarData` and limit_domain is not configured, then the `final topic name` is: req/aimrt_rpc/%2Faimrt.protocols.example.ExampleService%2FGetBarData.



The Zenoh data packet format from Client -> Server is divided into 5 segments:
- Serialization type, generally `pb` or `json`
- Zenoh topic name that the client wants the server to reply rsp to. The client itself needs to subscribe to this zenoh topic
- msg id, 4 bytes, the server will encapsulate it unchanged into the rsp packet for the client to locate which req the rsp corresponds to
- context section
  - context count, 1 byte, maximum 255 contexts
  - context_1 key, 2 bytes length + data section
  - context_2 key, 2 bytes length + data section
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


The Zenoh data packet format from Server -> Client is divided into 4 segments:
- Serialization type, generally `pb` or `json`
- msg id, 4 bytes, the msg id from req
- status code, 4 bytes, framework error code, if this part is non-zero, it indicates an error occurred on the server side, and the data section will be empty
- msg data


```
| n(0~255) [1 byte] | content type [n byte]
| msg id [4 byte]
| status code [4 byte]
| msg data [remaining byte]
```
## zenoh Type Channel Backend

The `zenoh` type Channel backend is a Channel backend provided in **zenoh_plugin**, mainly used to build publish-subscribe models. All its configuration items are as follows:

| Node                              | Type   | Optional | Default | Purpose                            |
| --------------------------------- | ------ | -------- | ------- | ---------------------------------- |
| pub_topics_options                | array  | Optional | []      | Rules when publishing Topics       |
| pub_topics_options[i].topic_name  | string | Required | ""      | Topic name, supports regular expressions |
| pub_topics_options[i].shm_enabled | bool   | Required | false   | Whether the publisher uses shared memory communication |


Below is a simple publisher example:

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


Below is a simple subscriber example:

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


Both examples above use zeonh's service discovery mechanism, where two endpoints in the same network can automatically discover each other and establish connections.

In this process, the underlying Topic name format is: `channel/${topic_name}/${message_type}${limit_domain}`. Among them, `${topic_name}` is the AimRT Topic name, `${message_type}` is the URL-encoded AimRT message name, and `${limit_domain}` is the plugin's restriction domain. This Topic is set as Zenoh's final key expression (Keyxpr), which is the resource identifier provided by Zenoh. Only subscribers and publishers with matching key expressions can communicate.

For example, if the AimRT Topic name is `test_topic`, the message type is `pb:aimrt.protocols.example.ExampleEventMsg`, and the restriction domain is `room1/A2`, then the final Zenoh topic name will be: `channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsgroom1/A2`. If both the subscriber and publisher have the Topic `channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsgroom1/A2`, they can communicate.

In the entire chain from AimRT publisher to subscriber, the Zenoh data packet format is divided into 3 segments:
- Serialization type, generally `pb` or `json`
- context area
  - context count, 1 byte, maximum 255 contexts
  - context_1 key, 2 bytes length + data area
  - context_2 key, 2 bytes length + data area
  - ...
- data

The data packet format is as follows:

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
