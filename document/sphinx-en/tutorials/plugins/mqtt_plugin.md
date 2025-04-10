

# MQTT Plugin

## Relevant Links

Reference Example:
- {{ '[mqtt_plugin]({}/src/examples/plugins/mqtt_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**mqtt_plugin** is a network transport plugin implemented based on the MQTT protocol, providing the following components:
- `mqtt` type RPC backend
- `mqtt` type Channel backend

The plugin configuration items are as follows:

| Node                  | Type   | Optional | Default | Description                          |
| --------------------- | ------ | -------- | ------- | ------------------------------------ |
| broker_addr           | string | Mandatory | ""     | Address of MQTT broker               |
| client_id             | string | Mandatory | ""     | MQTT client ID for this node         |
| max_pkg_size_k        | int    | Optional | 1024    | Maximum packet size, unit: KB        |
| reconnect_interval_ms | int    | Optional | 1000    | Broker reconnection interval, unit: ms |
| truststore            | string | Optional | ""     | CA certificate path                  |
| client_cert           | string | Optional | ""     | Client certificate path              |
| client_key            | string | Optional | ""     | Client private key path              |
| client_key_password   | string | Optional | ""     | Password for client private key      |

Configuration notes for **mqtt_plugin**:
- `broker_addr` represents the address of the MQTT broker. Users must ensure an MQTT broker is running at this address, otherwise startup will fail.
- `client_id` represents the client ID used when connecting to the MQTT broker.
- `max_pkg_size_k` specifies the maximum packet size for data transmission, default 1 MB. Note that the broker must also support this size.
- `reconnect_interval_ms` specifies the broker reconnection interval, default 1 second.
- `truststore` indicates the CA certificate path for the broker, e.g., `/etc/emqx/certs/cacert.pem`. This option takes effect when the protocol in `broker_addr` is configured as `ssl` or `mqtts`, used to specify the CA certificate path. Configuration of only this option indicates one-way authentication.
- `client_cert` indicates the client certificate path, e.g., `/etc/emqx/certs/client-cert.pem`. Used for mutual authentication with `client_key`. Ignored if broker_addr uses non-encrypted protocols.
- `client_key` indicates the client private key path, e.g., `/etc/emqx/certs/client-key.pem`. Used for mutual authentication with `client_cert`. Ignored if broker_addr uses non-encrypted protocols.
- `client_key_password` specifies the password for the client private key. Required if the private key is password-protected. Ignored if broker_addr uses non-encrypted protocols.

**mqtt_plugin** is implemented based on [paho.mqtt.c](https://github.com/eclipse/paho.mqtt.c). When using it, please note that the Channel subscription callbacks, RPC Server handlers, and RPC Client returns all execute in threads provided by **paho.mqtt.c**. If users block these threads in callbacks, it may prevent continued message reception/transmission. As mentioned in the Module interface documentation: generally, if callback tasks are lightweight, they can be processed directly in the callback; but for heavy tasks, it's better to schedule them to dedicated executors.

Here's a simple example:
```yaml
aimrt:
  plugin:
    plugins:
      - name: mqtt_plugin
        path: ./libaimrt_mqtt_plugin.so
        options:
          broker_addr: tcp://127.0.0.1:1883
          client_id: example_mqtt_client
          max_pkg_size_k: 1024
```

## MQTT Type RPC Backend

The `mqtt` type RPC backend is an RPC backend provided by **mqtt_plugin** for invoking and processing AimRT RPC requests through MQTT. All its configuration items are as follows:

| Node                              | Type   | Optional | Default | Description                                                                 |
| --------------------------------- | ------ | -------- | ------- | --------------------------------------------------------------------------- |
| timeout_executor                  | string | Optional | ""      | Executor for handling RPC timeout on client side                           |
| clients_options                   | array  | Optional | []      | Rules for client-side RPC requests                                         |
| clients_options[i].func_name      | string | Required | ""      | RPC function name (supports regular expressions)                          |
| clients_options[i].server_mqtt_id | string | Optional | ""      | Target server MQTT ID for RPC function invocation                         |
| clients_options[i].qos            | int    | Optional | 2       | MQTT QoS level for client (valid values: 0/1/2)                           |
| servers_options                   | array  | Optional | []      | Rules for server-side RPC request processing                              |
| servers_options[i].func_name      | string | Required | ""      | RPC function name (supports regular expressions)                          |
| servers_options[i].allow_share    | bool   | Optional | true    | Whether to allow shared subscriptions for this RPC service                |
| servers_options[i].qos            | int    | Optional | 2       | MQTT QoS level for server (valid values: 0/1/2)                           |

A simple client configuration example:
```yaml
aimrt:
  plugin:
    plugins:
      - name: mqtt_plugin
        path: ./libaimrt_mqtt_plugin.so
        options:
          broker_addr: tcp://127.0.0.1:1883
          client_id: example_client
          max_pkg_size_k: 1024
  executor:
    executors:
      - name: timeout_handle
        type: time_wheel
  rpc:
    backends:
      - type: mqtt
        options:
          timeout_executor: timeout_handle
          clients_options:
            - func_name: "(.*)"
              qos: 0
    clients_options:
      - func_name: "(.*)"
        enable_backends: [mqtt]
```

A simple server configuration example:
```yaml
aimrt:
  plugin:
    plugins:
      - name: mqtt_plugin
        path: ./libaimrt_mqtt_plugin.so
        options:
          broker_addr: tcp://127.0.0.1:1883
          client_id: example_server
          max_pkg_size_k: 1024
  rpc:
    backends:
      - type: mqtt
        options:
          servers_options:
            - func_name: "(.*)"
              allow_share: true
              qos: 0
    servers_options:
      - func_name: "(.*)"
        enable_backends: [mqtt]
```

In these examples, both client and server connect to an MQTT broker at `tcp://127.0.0.1:1883`. The client configuration ensures all RPC requests are processed through the MQTT backend to complete the RPC invocation loop.

When multiple servers register the same RPC service, clients will randomly select a server. To specify a target server, set ToAddr in client context as:
```cpp
auto ctx_ptr = proxy->NewContextSharedPtr();
// mqtt://{{target server mqtt id}}
ctx_ptr->SetToAddr("mqtt://target_server_mqtt_id");

auto status = proxy->Foo(ctx_ptr, req, rsp);
```

The underlying MQTT topic format follows these patterns:
- Server Side
  - Request subscription topics (both subscribed):
    - `$share/aimrt/aimrt_rpc_req/${func_name}`
    - `aimrt_rpc_req/${server_id}/${func_name}`
  - Response publication topic: `aimrt_rpc_rsp/${client_id}/${func_name}`
- Client Side
  - Request publication topics (choose one):
    - `aimrt_rpc_req/${func_name}`
    - `aimrt_rpc_req/${server_id}/${func_name}`
  - Response subscription topic: `aimrt_rpc_rsp/${client_id}/${func_name}`

Where `${client_id}` and `${server_id}` must be globally unique within the MQTT broker environment (typically using broker-registered IDs). `${func_name}` represents URL-encoded AimRT RPC method names. Servers use shared subscriptions to ensure exclusive request processing, requiring MQTT 5.0-compatible brokers.

Example: For client ID `example_client` and RPC method `/aimrt.protocols.example.ExampleService/GetBarData`, `${client_id}` becomes `example_client` and `${func_name}` becomes `%2Faimrt.protocols.example.ExampleService%2FGetBarData`.

Client -> Server Mqtt Packet Format
Divided into 5 segments:
- Serialization type, usually `pb` or `json`
- Mqtt topic name for server response. Client must subscribe to this topic
- Message ID (4 bytes), will be included unchanged in server response
- Context section
  - Context count (1 byte, max 255)
  - Context_1 key: 2-byte length + data
  - Context_2 key: 2-byte length + data
  - ...
- Message payload

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

Server -> Client Mqtt Packet Format
Divided into 4 segments:
- Serialization type, usually `pb` or `json`
- Message ID (4 bytes, same as request)
- Status code (4 bytes):
  - Non-zero indicates framework error (no data payload)
  - Zero indicates success
- Message payload

```
| n(0~255) [1 byte] | content type [n byte]
| msg id [4 byte]
| status code [4 byte]
| msg data [remaining byte]
```

## MQTT Type Channel Backend

The `mqtt` type Channel backend is a Channel backend implementation provided by the **mqtt_plugin**, used for publishing and subscribing messages via MQTT. All its configuration items are as follows:

| Node                          | Type   | Optional | Default | Description                              |
| ----------------------------- | ------ | -------- | ------- | ---------------------------------------- |
| pub_topics_options            | array  | Yes      | []      | Rules for publishing Topics              |
| pub_topics_options[i].topic_name | string | Required | ""     | Topic name supporting regular expressions |
| pub_topics_options[i].qos     | int    | Required | 2       | MQTT QoS for publisher (0/1/2)           |
| sub_topics_options            | array  | Yes      | []      | Rules for subscribing Topics             |
| sub_topics_options[i].topic_name | string | Required | ""     | Topic name supporting regular expressions |
| sub_topics_options[i].qos     | int    | Required | 2       | MQTT QoS for subscriber (0/1/2)         |

Here's a simple publisher example:
```yaml
aimrt:
  plugin:
    plugins:
      - name: mqtt_plugin
        path: ./libaimrt_mqtt_plugin.so
        options:
          broker_addr: tcp://127.0.0.1:1883
          client_id: example_publisher
          max_pkg_size_k: 1024
  channel:
    backends:
      - type: mqtt
        options:
          pub_topics_options:
            - topic_name: "(.*)"
              qos: 2
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [mqtt]
```

Here's a simple subscriber example:
```yaml
aimrt:
  plugin:
    plugins:
      - name: mqtt_plugin
        path: ./libaimrt_mqtt_plugin.so
        options:
          broker_addr: tcp://127.0.0.1:1883
          client_id: example_subscriber
          max_pkg_size_k: 1024
  channel:
    backends:
      - type: mqtt
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [mqtt]
```

In the above examples, both publisher and subscriber connect to an MQTT broker at `tcp://127.0.0.1:1883`. The publisher is configured to process all messages through the MQTT backend, while the subscriber is configured to trigger callbacks for all messages received via the MQTT backend, thus establishing complete pub-sub communication.

The underlying MQTT Topic name format is: `/channel/${topic_name}/${message_type}`. Where:
- `${topic_name}` is the AimRT Topic name
- `${message_type}` is the URL-encoded AimRT message name

For example:
- AimRT Topic name: `test_topic`
- Message type: `pb:aimrt.protocols.example.ExampleEventMsg`
- Resulting MQTT Topic: `/channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsg`

The MQTT packet format for AimRT publisher-subscriber communication consists of 3 segments:
1. Serialization type (typically `pb` or `json`)
2. Context section:
   - Context count (1 byte, max 255)
   - Context_1 key (2-byte length + data)
   - Context_2 key (2-byte length + data)
   - ...
3. Payload data

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