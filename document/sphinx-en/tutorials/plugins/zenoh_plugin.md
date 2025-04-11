

# Zenoh Plugin

## Related Links

Reference examples:
- {{ '[zenoh_plugin]({}/src/examples/plugins/zenoh_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**zenoh_plugin** is a lightweight, efficient, real-time data transmission plugin designed to provide low-latency, high-throughput data transmission and processing capabilities for distributed systems. The zenoh plugin is particularly suitable for the following scenarios:
- Communication systems requiring `service discovery` mechanism
- Flexible network topologies
- Low-latency, high-throughput network communication and data transmission
- Both SHM and non-SHM transmission modes

This plugin provides the following components for AimRT:
- `zenoh` type Rpc backend
- `zenoh` type Channel backend

Plugin configuration items are as follows:

|        Node        |  Type  | Optional | Default |             Function              |
| :----------------: | :----: | :------: | :-----: | :-------------------------------: |
|   shm_pool_size    |  int   | Optional | 10 MB   | Shared memory pool size (Unit: B)  |
| shm_init_loan_size |  int   | Optional | 1 KB    | Initial shared memory loan (Unit: B) |
|  native_cfg_path   | string | Optional |   ""    | Native configuration file path for zenoh |
|    limit_domain    | string | Optional |   ""    | Communication domain restriction   |

Important notes about **zenoh_plugin** configuration:
- `shm_pool_size` indicates the size of shared memory pool in bytes. Default is 10 MB. Adjust according to actual needs. This can be ignored if not using shared memory. Insufficient remaining shared memory will automatically switch to non-shared memory mode.
- `shm_init_loan_size` indicates initial borrowing size from shared memory pool in bytes. Default is 1 KB. Adjust according to actual needs. This can be ignored if not using shared memory.
- `native_cfg_path` specifies the path to zenoh's native configuration file. Refer to zenoh's official [configuration](https://zenoh.io/docs/manual/configuration/) documentation. You can modify {{ '[zenoh_native_config.json5]({}/src/examples/plugins/zenoh_plugin/install/linux/bin/cfg/zenoh_native_config.json5)'.format(code_site_root_path_url) }} for custom configurations. Common configuration items:

|          Configuration Item          |                         Function                          | zenoh_native_config Value |
| :-----------------------------------: | :------------------------------------------------------: | :------------------------: |
|  scouting.multicast.enabled  | Enable multicast for automatic node discovery |           true            |
|  scouting.multicast.address  |                  Multicast address                   |    224.0.0.224:7446      |
| scouting.multicast.interface |                   Network interface                   |           auto            |
|       listen.endpoints        |                 Active listening addresses                 |             -             |
| transport.unicast.lowlatency | Enable low-latency mode (improves throughput, incompatible with QoS) |          false           |
| transport.unicast.qos.enabled | Enable Quality of Service (QoS, incompatible with low-latency) |          true           |

- `limit_domain` specifies communication domain constraints using zenoh's powerful Key & Key Expression system. When empty, uses default plugin domain (message topic). Only `matching` domains can communicate. Format examples:

```shell

#请不要以"/"开始，中间以"/"分隔，结尾不要带"/" （与zenoh官方书写方式一致），如：

xxx/yyy/zzz

```
The simplest matching is identical domains. Zenoh provides more flexible matching mechanisms - refer to zenoh's official [key](https://zenoh.io/docs/manual/abstractions/) documentation.

Sample configuration:

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

The `zenoh` type Rpc backend is provided in **zenoh_plugin**, primarily used to build request-response models. All its configuration items are as follows:

| Node                           | Type   | Optional | Default | Description                          |
| ------------------------------ | ------ | -------- | ------- | ------------------------------------ |
| timeout_executor               | string | Optional | ""      | Executor for RPC timeout on client   |
| clients_options                | array  | Optional | []      | Rules for client RPC requests        |
| clients_options[i].func_name   | string | Mandatory| ""      | RPC Func name (supports regex)       |
| clients_options[i].shm_enabled | bool   | Optional | false   | Whether to use SHM communication     |
| servers_options                | array  | Optional | []      | Rules for server RPC processing     |
| servers_options[i].func_name   | string | Mandatory| ""      | RPC Func name (supports regex)       |
| servers_options[i].shm_enabled | bool   | Optional | false   | Whether to use SHM communication     |


Client-side example:

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

Server-side example:

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

In these examples, both client and server use service discovery mechanism to automatically detect and connect with peers in the same network.

Underlying Zenoh Topic naming conventions:
- Server side
  - Request subscription topic:  
    `req/aimrt_rpc/${func_name}/${limit_domain}`
  - Response publication topic: `rsp/aimrt_rpc/${func_name}/${limit_domain}`
- Client side
  - Request publication topic:  
    `req/aimrt_rpc/${func_name}/${limit_domain}`
  - Response subscription topic: `rsp/aimrt_rpc/${func_name}/${limit_domain}`

`${func_name}` represents URL-encoded AimRT RPC method names.

Example: For a client request with func name `/aimrt.protocols.example.ExampleService/GetBarData` and no limit_domain configuration, the final topic becomes:  
`req/aimrt_rpc/%2Faimrt.protocols.example.ExampleService%2FGetBarData`.

---

### Client -> Server Zenoh Packet Format (5 segments)
1. Serialization type (`pb` or `json`)
2. Response topic name (client must subscribe to this topic)
3. 4-byte message ID (echoed in server response)
4. Context section:
   - 1-byte context count (max 255)
   - Context entries:
     - 2-byte key length + key data
5. Message payload

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

---

### Server -> Client Zenoh Packet Format (4 segments)
1. Serialization type (`pb` or `json`)
2. 4-byte message ID (from client request)
3. 4-byte status code (non-zero indicates error, no data segment)
4. Message payload

```
| n(0~255) [1 byte] | content type [n byte]
| msg id [4 byte]
| status code [4 byte]
| msg data [remaining byte]
```

## zenoh Type Channel Backend

The `zenoh` type Channel backend is a Channel backend provided in **zenoh_plugin**, mainly used to build a publish-subscribe model. All its configuration items are as follows:

| Node                              | Type   | Optional | Default Value | Description                            |
| --------------------------------- | ------ | -------- | ------------- | -------------------------------------- |
| pub_topics_options                | array  | Optional | []            | Rules when publishing Topics           |
| pub_topics_options[i].topic_name  | string | Required | ""            | Topic name supporting regular expression |
| pub_topics_options[i].shm_enabled | bool   | Required | false         | Whether to use shared memory communication on publisher side |


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

Both examples use zenoh's service discovery mechanism, where endpoints in the same network can automatically discover each other and establish connections.

In this process, the underlying Topic name format is: `channel/${topic_name}/${message_type}${limit_domain}`. Here:
- `${topic_name}` represents the AimRT Topic name
- `${message_type}` is the URL-encoded AimRT message name
- `${limit_domain}` is the plugin's limitation domain

This Topic is configured as Zenoh's final Key Expression (Keyxpr), which serves as Zenoh's resource identifier. Only subscribers and publishers with matching key expressions can communicate.

For example:
- AimRT Topic name: `test_topic`
- Message type: `pb:aimrt.protocols.example.ExampleEventMsg`
- Limitation domain: `room1/A2`

Final Zenoh topic becomes: `channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsgroom1/A2`. Subscribers and publishers using this identical topic can communicate.

In the data transmission chain from AimRT publisher to subscriber, Zenoh packet format consists of 3 segments:
1. Serialization type (typically `pb` or `json`)
2. Context section
   - Context count (1 byte, max 255 contexts)
   - Context_1 key (2-byte length + data)
   - Context_2 key (2-byte length + data)
   - ...
3. Payload data

Packet format is as follows:
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