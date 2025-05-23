# MQTT Plugin

## Related Links

Reference example:
- {{ '[mqtt_plugin]({}/src/examples/plugins/mqtt_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**mqtt_plugin** is a network transmission plugin implemented based on the MQTT protocol. This plugin provides the following components:
- `mqtt` type RPC backend
- `mqtt` type Channel backend

The plugin configuration items are as follows:

| Node                  | Type   | Optional | Default | Description                          |
| --------------------- | ------ | -------- | ------- | ------------------------------------ |
| broker_addr           | string | Required | ""      | Address of the MQTT broker           |
| client_id             | string | Required | ""      | MQTT client ID of this node          |
| max_pkg_size_k        | int    | Optional | 1024    | Maximum packet size in KB            |
| reconnect_interval_ms | int    | Optional | 1000    | Reconnection interval to broker in ms|
| truststore            | string | Optional | ""      | CA certificate path                  |
| client_cert           | string | Optional | ""      | Client certificate path              |
| client_key            | string | Optional | ""      | Client private key path              |
| client_key_password   | string | Optional | ""      | Password for client private key      |

Regarding the configuration of **mqtt_plugin**, the following points should be noted:
- `broker_addr` represents the address of the MQTT broker. Users must ensure an MQTT broker is running at this address; otherwise, startup will fail.
- `client_id` represents the client ID used when this node connects to the MQTT broker.
- `max_pkg_size_k` indicates the maximum packet size for data transmission, defaulting to 1 MB. Note that the broker must also support this size.
- `reconnect_interval_ms` specifies the reconnection interval to the broker, defaulting to 1 second.
- `truststore` indicates the CA certificate path for the broker, e.g., `/etc/emqx/certs/cacert.pem`. This option takes effect when the protocol of `broker_addr` is configured as `ssl` or `mqtts`, specifying the CA certificate path. Otherwise, this option is automatically ignored. Note that configuring only this option is considered one-way authentication.
- `client_cert` indicates the client certificate path, e.g., `/etc/emqx/certs/client-cert.pem`. Used for mutual authentication in conjunction with `client_key`. If broker_addr uses a non-encrypted protocol, this option is ignored.
- `client_key` indicates the client private key path, e.g., `/etc/emqx/certs/client-key.pem`. Used for mutual authentication in conjunction with `client_cert`. If broker_addr uses a non-encrypted protocol, this option is ignored.
- `client_key_password` specifies the password for the client private key. If the private key is password-protected, this option must be set. If broker_addr uses a non-encrypted protocol, this option is ignored.

**mqtt_plugin** is encapsulated based on [paho.mqtt.c](https://github.com/eclipse/paho.mqtt.c). When using it, the Channel subscription callback, RPC Server processing method, and RPC Client return all utilize threads provided by **paho.mqtt.c**. If users block the thread in the callback, it may prevent further message reception/transmission. As mentioned in the Module interface documentation, generally, if the task in the callback is very lightweight, it can be processed directly in the callback. However, if the task is heavy, it is better to schedule it to a dedicated executor for processing.

Here is a simple example:
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

The `mqtt` type RPC backend is an RPC backend provided by the **mqtt_plugin**, used to invoke and handle AimRT RPC requests via MQTT. All its configuration items are as follows:

| Node                              | Type   | Optional | Default | Description                                                                 |
| --------------------------------- | ------ | -------- | ------- | --------------------------------------------------------------------------- |
| timeout_executor                  | string | Optional | ""      | Executor for handling RPC timeout on the client side                        |
| clients_options                   | array  | Optional | []      | Rules for client-side RPC requests                                          |
| clients_options[i].func_name      | string | Required | ""      | RPC function name, supports regular expressions                             |
| clients_options[i].server_mqtt_id | string | Optional | ""      | MQTT server ID to which the RPC function request is sent                    |
| clients_options[i].qos            | int    | Optional | 2       | MQTT QoS level for RPC client, valid values: 0/1/2                          |
| servers_options                   | array  | Optional | []      | Rules for server-side RPC request handling                                  |
| servers_options[i].func_name      | string | Required | ""      | RPC function name, supports regular expressions                             |
| servers_options[i].allow_share    | bool   | Optional | true    | Whether the RPC service allows shared subscriptions                         |
| servers_options[i].qos            | int    | Optional | 2       | MQTT QoS level for RPC server, valid values: 0/1/2                          |

Here is a simple client example:
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

And here is a simple server example:
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

In the above examples, both the client and server connect to an MQTT broker at `tcp://127.0.0.1:1883`. The client is configured to process all RPC requests through the MQTT backend, thereby completing the RPC call loop.

If multiple servers register the same RPC service, the client will randomly select one server to send the request to. To specify a particular server, you can set the `ToAddr` in the client's context as follows:
```cpp
auto ctx_ptr = proxy->NewContextSharedPtr();
// mqtt://{{target server mqtt id}}
ctx_ptr->SetToAddr("mqtt://target_server_mqtt_id");

auto status = proxy->Foo(ctx_ptr, req, rsp);
```

During the RPC process, the underlying MQTT topic name formats are as follows:
- Server Side
  - Topics subscribed for requests (both will be subscribed):
    - `$share/aimrt/aimrt_rpc_req/${func_name}`
    - `aimrt_rpc_req/${server_id}/${func_name}`
  - Topic for publishing responses: `aimrt_rpc_rsp/${client_id}/${func_name}`
- Client Side
  - Topics for publishing requests (choose one):
    - `aimrt_rpc_req/${func_name}`
    - `aimrt_rpc_req/${server_id}/${func_name}`
  - Topic for subscribing to responses: `aimrt_rpc_rsp/${client_id}/${func_name}`

Here, `${client_id}` and `${server_id}` must be globally unique within the same MQTT broker environment, typically using the IDs registered with the MQTT broker. `${func_name}` is the URL-encoded name of the AimRT RPC method. Servers use shared subscriptions to ensure only one server processes the request. This feature requires an MQTT 5.0-compatible broker.

For example, if a client registers with the MQTT broker using the ID `example_client` and the function name is `/aimrt.protocols.example.ExampleService/GetBarData`, then `${client_id}` is `example_client` and `${func_name}` is `%2Faimrt.protocols.example.ExampleService%2FGetBarData`.The MQTT packet format from Client -> Server consists of 5 segments:
- Serialization type, typically `pb` or `json`
- The MQTT topic name where the client expects the server to reply with an rsp. The client needs to subscribe to this MQTT topic itself
- Message ID, 4 bytes, which the server will encapsulate unchanged into the rsp packet for the client to identify which req corresponds to which rsp
- Context section
  - Number of contexts, 1 byte, maximum 255 contexts
  - context_1 key, 2-byte length + data section
  - context_2 key, 2-byte length + data section
  - ...
- Message data

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

The MQTT packet format from Server -> Client consists of 4 segments:
- Serialization type, typically `pb` or `json`
- Message ID, 4 bytes, the msg ID from the req
- Status code, 4 bytes, framework error code. If this value is non-zero, it indicates an error occurred on the server side, and the data section will be empty
- Message data

```
| n(0~255) [1 byte] | content type [n byte]
| msg id [4 byte]
| status code [4 byte]
| msg data [remaining byte]
```
## MQTT Type Channel Backend

The `mqtt` type Channel backend is a Channel backend provided in the **mqtt_plugin**, used for publishing and subscribing to messages via MQTT. All its configuration items are as follows:

| Node                             | Type   | Optional | Default | Description                           |
| -------------------------------- | ------ | -------- | ------- | ------------------------------------- |
| pub_topics_options               | array  | Optional | []      | Rules for publishing Topics           |
| pub_topics_options[i].topic_name | string | Required | ""      | Topic name, supports regular expressions |
| pub_topics_options[i].qos        | int    | Required | 2       | MQTT QoS for the publish side, valid values: 0/1/2 |
| sub_topics_options               | array  | Optional | []      | Rules for subscribing to Topics       |
| sub_topics_options[i].topic_name | string | Required | ""      | Topic name, supports regular expressions |
| sub_topics_options[i].qos        | int    | Required | 2       | MQTT QoS for the subscribe side, valid values: 0/1/2 |

Here is a simple example for the publish side:
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

Here is a simple example for the subscribe side:
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

In the above examples, both the publish and subscribe sides are connected to an MQTT broker at the address `tcp://127.0.0.1:1883`. The publish side is configured to process all messages through the MQTT backend, and the subscribe side is configured to trigger callbacks for all messages from the MQTT backend, thereby establishing the message publish-subscribe pipeline.

In this process, the underlying MQTT Topic name format is: `/channel/${topic_name}/${message_type}`. Here, `${topic_name}` is the AimRT Topic name, and `${message_type}` is the URL-encoded AimRT message name.

For example, if the AimRT Topic name is `test_topic` and the message type is `pb:aimrt.protocols.example.ExampleEventMsg`, the final MQTT topic name will be: `/channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsg`.

On the pipeline where AimRT publishes data from the publish side to the subscribe side, the MQTT packet format is divided into three segments:
- Serialization type, typically `pb` or `json`
- Context section
  - Number of contexts, 1 byte, maximum 255 contexts
  - context_1 key, 2-byte length + data section
  - context_2 key, 2-byte length + data section
  - ...
- Data

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