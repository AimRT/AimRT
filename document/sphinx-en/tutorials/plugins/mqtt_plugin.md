# Mqtt Plugin


## Related Links

Reference example:
- {{ '[mqtt_plugin]({}/src/examples/plugins/mqtt_plugin)'.format(code_site_root_path_url) }}


## Plugin Overview

**mqtt_plugin** is a network transport plugin implemented based on the mqtt protocol. This plugin provides the following components:
- `mqtt` type RPC backend
- `mqtt` type Channel backend


The plugin configuration items are as follows:

| Node                  | Type   | Optional | Default | Purpose                              |
| --------------------- | ------ | -------- | ------- | ------------------------------------ |
| broker_addr           | string | Required | ""      | Address of the mqtt broker           |
| client_id             | string | Required | ""      | This node's mqtt client id           |
| max_pkg_size_k        | int    | Optional | 1024    | Maximum packet size, unit: KB        |
| reconnect_interval_ms | int    | Optional | 1000    | Reconnection interval to broker, unit: ms |
| truststore            | string | Optional | ""      | Path to CA certificate               |
| client_cert           | string | Optional | ""      | Path to client certificate           |
| client_key            | string | Optional | ""      | Path to client private key           |
| client_key_password   | string | Optional | ""      | Password set for client private key  |


Regarding the configuration of **mqtt_plugin**, the following points should be noted:
- `broker_addr` indicates the address of the mqtt broker. Users must ensure that an mqtt broker is running at this address, otherwise startup will fail.
- `client_id` indicates the client id used when this node connects to the mqtt broker.
- `max_pkg_size_k` indicates the maximum packet size during data transmission, default 1 MB. Note that the broker must also support this size.
- `reconnect_interval_ms` indicates the reconnection interval to the broker, default 1 second.
- `truststore` indicates the path to the broker's CA certificate, e.g. `/etc/emqx/certs/cacert.pem`. This option takes effect when the protocol of `broker_addr` is configured as `ssl` or `mqtts`, used to specify the CA certificate path; otherwise, this option is automatically ignored. Please note that configuring only this option is considered one-way authentication.
- `client_cert` indicates the path to the client certificate, e.g. `/etc/emqx/certs/client-cert.pem`. Used when mutual authentication is required, in conjunction with `client_key`. If broker_addr uses an unencrypted protocol, this option will be ignored.
- `client_key` indicates the path to the client private key, e.g. `/etc/emqx/certs/client-key.pem`. Used when mutual authentication is required, in conjunction with `client_cert`. If broker_addr uses an unencrypted protocol, this option will be ignored.
- `client_key_password` indicates the password set for the client private key. If the private key is password-protected, this option needs to be set. If broker_addr uses an unencrypted protocol, this option will be ignored.

The **mqtt_plugin** plugin is encapsulated based on [paho.mqtt.c](https://github.com/eclipse/paho.mqtt.c). When in use, the threads provided by **paho.mqtt.c** are used for Channel subscription callbacks, RPC Server processing methods, and RPC Client returns. When users block threads in callbacks, it may prevent continued message reception/transmission. As mentioned in the Module interface documentation, generally, if the task in the callback is very lightweight, it can be processed directly in the callback; but if the task in the callback is relatively heavy, it's best to schedule it to another dedicated executor for processing.


Below is a simple example:

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
## mqtt Type RPC Backend


The `mqtt` type RPC backend is an RPC backend provided by **mqtt_plugin**, used to invoke and handle AimRT RPC requests via MQTT. All its configuration items are as follows:


| Node                              | Type   | Optional | Default | Description                                                                 |
| --------------------------------- | ------ | -------- | ------- | --------------------------------------------------------------------------- |
| timeout_executor                  | string | Optional | ""      | Executor used on the client side when an RPC times out                      |
| clients_options                   | array  | Optional | []      | Rules for the client side when initiating RPC requests                      |
| clients_options[i].func_name      | string | Required | ""      | RPC Func name, supports regular expressions                                 |
| clients_options[i].server_mqtt_id | string | Optional | ""      | MQTT server id requested when the RPC Func is invoked                       |
| clients_options[i].qos            | int    | Optional | 2       | MQTT QoS on the RPC client side, allowed values: 0/1/2                      |
| servers_options                   | array  | Optional | []      | Rules for the server side when processing RPC requests                      |
| servers_options[i].func_name      | string | Required | ""      | RPC Func name, supports regular expressions                                 |
| servers_options[i].allow_share    | bool   | Optional | true    | Whether this RPC service allows shared subscriptions; if not, the service can only be invoked via a specified server id |
| servers_options[i].qos            | int    | Optional | 2       | MQTT QoS on the RPC server side, allowed values: 0/1/2                      |

Below is a simple client example:

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


Below is a simple server example:

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


In the examples above, both the client and server connect to an MQTT broker at `tcp://127.0.0.1:1883`, and the client is configured to handle all RPC requests via the mqtt backend, thus completing the RPC call loop.

If multiple servers register the same RPC service, the client will randomly select one server to send the request. To specify a particular server, you can set ToAddr in the client's ctx as follows:

```cpp
auto ctx_ptr = proxy->NewContextSharedPtr();
// mqtt://{{target server mqtt id}}
ctx_ptr->SetToAddr("mqtt://target_server_mqtt_id");

auto status = proxy->Foo(ctx_ptr, req, rsp);
```



Throughout the RPC process, the underlying MQTT topic names are formatted as follows:
- Server side
  - Topics subscribed for Req (both are subscribed):
    - `$share/aimrt/aimrt_rpc_req/${func_name}`
    - `aimrt_rpc_req/${server_id}/${func_name}`
  - Topic published for Rsp: `aimrt_rpc_rsp/${client_id}/${func_name}`
- Client side
  - Topics published for Req (choose one):
    - `aimrt_rpc_req/${func_name}`
    - `aimrt_rpc_req/${server_id}/${func_name}`
  - Topic subscribed for Rsp: `aimrt_rpc_rsp/${client_id}/${func_name}`

`${client_id}` and `${server_id}` must be globally unique within the same MQTT broker environment for the client and server, typically using the id registered with the MQTT broker. `${func_name}` is the URL-encoded AimRT RPC method name. The server subscribes using shared subscriptions to ensure only one server handles the request. This feature requires an MQTT 5.0-compliant broker.


For example, if the client registers with the MQTT broker using id `example_client` and the func name is `/aimrt.protocols.example.ExampleService/GetBarData`, then `${client_id}` is `example_client` and `${func_name}` is `%2Faimrt.protocols.example.ExampleService%2FGetBarData`.The overall Mqtt packet format from Client -> Server is divided into 5 segments:
- Serialization type, usually `pb` or `json`
- The mqtt topic name that the client wants the server to reply rsp to. The client itself needs to subscribe to this mqtt topic
- msg id, 4 bytes, the server will encapsulate it unchanged into the rsp packet for the client to locate which req the rsp corresponds to
- context area
  - number of contexts, 1 byte, maximum 255 contexts
  - context_1 key, 2-byte length + data area
  - context_2 key, 2-byte length + data area
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


The overall Mqtt packet format from Server -> Client is divided into 4 segments:
- Serialization type, usually `pb` or `json`
- msg id, 4 bytes, the msg id from the req
- status code, 4 bytes, framework error code, if this part is non-zero, it means an error occurred on the server side, and the data segment will have no content
- msg data


```
| n(0~255) [1 byte] | content type [n byte]
| msg id [4 byte]
| status code [4 byte]
| msg data [remaining byte]
```
## mqtt Type Channel Backend


The `mqtt` type Channel backend is a Channel backend provided by **mqtt_plugin**, used to publish and subscribe to messages via mqtt. All its configuration items are as follows:


| Node                             | Type   | Optional | Default | Purpose                                   |
| -------------------------------- | ------ | -------- | ------- | ----------------------------------------- |
| pub_topics_options               | array  | Optional | []      | Rules when publishing Topic               |
| pub_topics_options[i].topic_name | string | Required | ""      | Topic name, supports regular expressions  |
| pub_topics_options[i].qos        | int    | Required | 2       | Publish side mqtt qos, range: 0/1/2       |
| sub_topics_options               | array  | Optional | []      | Rules when publishing Topic               |
| sub_topics_options[i].topic_name | string | Required | ""      | Topic name, supports regular expressions  |
| sub_topics_options[i].qos        | int    | Required | 2       | Subscribe side mqtt qos, range: 0/1/2     |


Below is a simple publisher example:

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


And here is a simple subscriber example:

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


In the above examples, both the publisher and subscriber connect to an Mqtt broker at address `tcp://127.0.0.1:1883`. The publisher is configured to process all messages through the mqtt backend, and the subscriber is configured to trigger callbacks for all messages from the mqtt backend, thereby establishing the message publish-subscribe link.


During this process, the underlying Mqtt Topic name format is: `/channel/${topic_name}/${message_type}`. Here, `${topic_name}` is the AimRT Topic name, and `${message_type}` is the url-encoded AimRT message name.

For example, if the AimRT Topic name is `test_topic` and the message type is `pb:aimrt.protocols.example.ExampleEventMsg`, then the final Mqtt topic name will be: `/channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsg`.


In the chain from AimRT publisher to subscriber, the Mqtt packet format is divided into 3 segments:
- Serialization type, generally `pb` or `json`
- context section
  - context count, 1 byte, maximum 255 contexts
  - context_1 key, 2-byte length + data section
  - context_2 key, 2-byte length + data section
  - ...
- data


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
