# Net Plugin

## Related Links

Reference example:
- {{ '[net_plugin]({}/src/examples/plugins/net_plugin)'.format(code_site_root_path_url) }}


## Plugin Overview

**net_plugin** is a network transport plugin implemented based on the boost asio/beast library. This plugin provides the following components:
- `http` type RPC backend
- `http` type Channel backend
- `tcp` type Channel backend
- `udp` type Channel backend

The plugin configuration items are as follows:

| Node                      | Type      | Optional | Default   | Purpose |
| ----                      | ----      | ----     | ----      | ---- |
| thread_num                | int       | Required | 2         | Number of threads the net plugin needs to use |
| http_options              | map       | Optional | -         | http related options |
| http_options.listen_ip    | string    | Optional | "0.0.0.0" | http listening IP |
| http_options.listen_port  | int       | Required | -         | http listening port, the port must not be occupied |
| tcp_options               | map       | Optional | -         | tcp related options |
| tcp_options.listen_ip     | string    | Optional | "0.0.0.0" | tcp listening IP |
| tcp_options.listen_port   | int       | Required | -         | tcp listening port, the port must not be occupied |
| udp_options               | map       | Optional | -         | udp related options |
| udp_options.listen_ip     | string    | Optional | "0.0.0.0" | udp listening IP |
| udp_options.listen_port   | int       | Required | -         | udp listening port, the port must not be occupied |
| udp_options.max_pkg_size  | int       | Optional | 1024      | Maximum udp packet size, theoretically cannot exceed 65515 |



Regarding the configuration of **net_plugin**, the usage notes are as follows:
- `thread_num` indicates the number of threads used by the net plugin.
- `http_options` is optional and only needs to be configured when you need to use the http RPC server functionality or the Channel subscriber functionality.
  - `http_options.listen_ip` is used to configure the address the http service listens on, default is "0.0.0.0". If you only want to listen on a specific network interface, you can change it to the specified IP.
  - `http_options.listen_port` is used to configure the port the http service listens on. This is a required item, users must ensure the port is not occupied, otherwise the plugin will fail to initialize.
- `tcp_options` is optional and only needs to be configured when you need to use the tcp Channel subscriber functionality.
  - `tcp_options.listen_ip` is used to configure the address the tcp service listens on, default is "0.0.0.0". If you only want to listen on a specific network interface, you can change it to the specified IP.
  - `tcp_options.listen_port` is used to configure the port the tcp service listens on. This is a required item, users must ensure the port is not occupied, otherwise the plugin will fail to initialize.
- `udp_options` is optional and only needs to be configured when you need to use the udp Channel subscriber functionality.
  - `udp_options.listen_ip` is used to configure the address the udp service listens on, default is "0.0.0.0". If you only want to listen on a specific network interface, you can change it to the specified IP.
  - `udp_options.listen_port` is used to configure the port the udp service listens on. This is a required item, users must ensure the port is not occupied, otherwise the plugin will fail to initialize.
  - `udp_options.max_pkg_size` is used to configure the maximum udp packet size, theoretically cannot exceed 65515. After serialization, the published message must be smaller than this value, otherwise the publication will fail.


Additionally, when using **net_plugin**, the Channel subscription callback, RPC Server processing callback, and RPC Client return all use the own thread executor provided by **net_plugin**. When users block the thread in the callback, it may lead to exhaustion of the **net_plugin** thread pool, thus preventing continued message receiving/sending. As stated in the Module interface documentation, generally speaking, if the task in the callback is very lightweight, it can be processed directly in the callback; but if the task in the callback is heavy, it's best to schedule it to another dedicated task executor for processing.


Here is a simple example:

```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
          http_options:
            listen_ip: 127.0.0.1
            listen_port: 50080
          tcp_options:
            listen_ip: 127.0.0.1
            listen_port: 50081
          udp_options:
            listen_ip: 127.0.0.1
            listen_port: 50082
            max_pkg_size: 1024
```
## http Type RPC Backend


The `http` type RPC backend is an RPC backend provided in **net_plugin**, used to invoke and process RPC requests via HTTP. All its configuration items are as follows:


| Node                          | Type      | Optional | Default | Purpose |
| ----                          | ----      | ----     | ----    | ---- |
| clients_options               | array     | Optional | []      | Rules for the client when initiating RPC requests |
| clients_options[i].func_name  | string    | Required | ""      | RPC Func name, supports regular expressions |
| clients_options[i].server_url | string    | Required | ""      | URL to which the RPC Func sends the request |


Below is a simple client example:

```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
  rpc:
    backends:
      - type: http
        options:
          clients_options:
            - func_name: "(.*)"
              server_url: http://127.0.0.1:50080
    clients_options:
      - func_name: "(.*)"
        enable_backends: [http]
```


Below is a simple server example:

```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
          http_options:
            listen_ip: 127.0.0.1
            listen_port: 50080
  rpc:
    backends:
      - type: http
    servers_options:
      - func_name: "(.*)"
        enable_backends: [http]
```



In the examples above, the Server listens on the local port 50080, and the Client configures all RPC requests to be sent via the http backend to the address `http://127.0.0.1:50080`, which is the address the Server listens on, thereby completing the RPC call loop.


When the Client initiates a call to the Server, the following format is used:
- Sent via HTTP POST, with the serialized request or response packet data filled in the Body;
- The serialization method of the Body content is defined by the `content-type` Header, for example:
  - `content-type:application/json`
  - `content-type:application/protobuf`
- URL encoding format: `http://{IP:PORT}/rpc/{FUNC_NAME}`:
  - `{IP:PORT}`: the peer's network address;
  - `{FUNC_NAME}`: the URL-encoded RPC func name, based on the name provided during RPC registration;


As long as this format is followed, users can initiate calls using tools like PostMan or Curl, using JSON as the serialization method for better readability. For example, the following command can send a message to a specified module to trigger the corresponding callback:

```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.example.ExampleService/GetFooData' \
    -d '{"msg": "test msg"}'
```


Based on this feature, the `http` type RPC backend is often used during debugging, development, or testing phases, allowing quick triggering of a Server handler via auxiliary tools or inspecting the communication content between upstream and downstream using packet capture tools.## http Type Channel Backend


The `http` type Channel backend is a backend provided by **net_plugin** for publishing and subscribing to messages via HTTP. All its configuration items are as follows:


| Node                                  | Type          | Optional | Default | Purpose |
| ----                                  | ----          | ----   | ----   | ---- |
| pub_topics_options                    | array         | Optional | []     | Rules when publishing Topics |
| pub_topics_options[i].topic_name      | string        | Required | ""     | Topic name, supports regular expressions |
| pub_topics_options[i].server_url_list | string array  | Required | []     | List of URLs to which the Topic should be sent |

Below is a simple publisher example:

```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
  channel:
    backends:
      - type: http
        options:
          pub_topics_options:
            - topic_name: "(.*)"
              server_url_list: ["127.0.0.1:50080"]
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [http]
```


Below is a simple subscriber example:

```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
          http_options:
            listen_ip: 127.0.0.1
            listen_port: 50080
  channel:
    backends:
      - type: http
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [http]
```


In the above examples, the subscriber listens on local port 50080, and the publisher is configured to send all Topic messages via the http backend to the address `127.0.0.1:50080`, which is the address the server is listening on. The subscriber is also configured to allow all messages to trigger callbacks from the http backend, thereby establishing the message publish-subscribe chain.

When the publisher sends messages to the subscriber, the following format is followed:
- Sent using HTTP POST, with the serialized message data in the Body;
- The serialization method of the Body content is defined by the `content-type` Header, for example:
  - `content-type:application/json`
  - `content-type:application/protobuf`
- URL encoding format: `http://{IP:PORT}/channel/{TOPIC_NAME}/{MSG_TYPE}`:
  - `{IP:PORT}`: Network address of the peer;
  - `{TOPIC_NAME}`: URL-encoded Topic name;
  - `{MSG_TYPE}`: URL-encoded message type name, based on the definition in TypeSupport;

As long as this format is followed, users can initiate calls using tools like PostMan or Curl, using JSON as the serialization method for better readability. For example, with the following command, you can send a message to a specified module to trigger the corresponding callback:

```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsg' \
    -d '{"msg": "test msg", "num": 123}'
```


Leveraging this feature, the `http` type Channel backend is often used during debugging, development, or testing phases, allowing quick triggering of a callback using auxiliary tools, or inspecting communication content between upstream and downstream via packet capture tools.



## tcp Type Channel Backend


The `tcp` type Channel backend is a backend provided by **net_plugin** for publishing and subscribing to messages via TCP. All its configuration items are as follows:


| Node                                  | Type          | Optional | Default | Purpose |
| ----                                  | ----          | ----   | ----   | ---- |
| pub_topics_options                    | array         | Optional | []     | Rules when publishing Topics |
| pub_topics_options[i].topic_name      | string        | Required | ""     | Topic name, supports regular expressions |
| pub_topics_options[i].server_url_list | string array  | Required | []     | List of URLs to which the Topic should be sent |

Below is a simple publisher example:

```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
  channel:
    backends:
      - type: tcp
        options:
          pub_topics_options:
            - topic_name: "(.*)"
              server_url_list: ["127.0.0.1:50080"]
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [tcp]
```


Below is a simple subscriber example:

```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
          tcp_options:
            listen_ip: 127.0.0.1
            listen_port: 50080
  channel:
    backends:
      - type: tcp
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [tcp]
```


In the above examples, the subscriber listens on local port 50080, and the publisher is configured to send all Topic messages via the tcp backend to the address `127.0.0.1:50080`, which is the address the server is listening on, thereby establishing the message publish-subscribe chain.

Note that when using the tcp backend to transmit messages, the data format is proprietary, but the format will remain version-compatible.## udp Type Channel Backend

The `udp` type Channel backend is a Channel backend provided by **net_plugin**, used to publish and subscribe to messages via UDP. All its configuration items are as follows:

| Node                                  | Type          | Optional | Default | Purpose |
| ----                                  | ----          | ----     | ----    | ---- |
| pub_topics_options                    | array         | Optional | []      | Rules when publishing Topics |
| pub_topics_options[i].topic_name      | string        | Required | ""      | Topic name, supports regular expressions |
| pub_topics_options[i].server_url_list | string array  | Required | []      | List of URLs to which the Topic should be sent |

Below is a simple publisher example:

```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
  channel:
    backends:
      - type: udp
        options:
          pub_topics_options:
            - topic_name: "(.*)"
              server_url_list: ["127.0.0.1:50080"]
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [udp]
```


Below is a simple subscriber example:

```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
          udp_options:
            listen_ip: 127.0.0.1
            listen_port: 50080
  channel:
    backends:
      - type: udp
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [udp]
```


In the above examples, the subscriber listens on local port 50080, and the publisher configures all Topic messages to be sent via the udp backend to the address `127.0.0.1:50080`, which is the address the server is listening on, thereby establishing the message publish-subscribe link.

Note that when using the udp backend to transmit messages, the data format is proprietary, but the data format will maintain version compatibility.