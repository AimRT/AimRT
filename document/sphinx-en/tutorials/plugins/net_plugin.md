# Net Plugin

## Related Links

Reference example:
- {{ '[net_plugin]({}/src/examples/plugins/net_plugin)'.format(code_site_root_path_url) }}


## Plugin Overview

**net_plugin** is a network transmission plugin implemented based on the boost asio/beast library. This plugin provides the following components:
- `http` type RPC backend
- `http` type Channel backend
- `tcp` type Channel backend
- `udp` type Channel backend

The plugin configuration items are as follows:

| Node                      | Type      | Optional | Default | Description |
| ----                      | ----      | ----     | ----    | ----        |
| thread_num                | int       | Required | 2       | Number of threads required by the net plugin |
| http_options              | map       | Optional | -       | HTTP related options |
| http_options.listen_ip    | string    | Optional | "0.0.0.0" | HTTP listening IP |
| http_options.listen_port  | int       | Required | -       | HTTP listening port (must not be occupied) |
| tcp_options               | map       | Optional | -       | TCP related options |
| tcp_options.listen_ip     | string    | Optional | "0.0.0.0" | TCP listening IP |
| tcp_options.listen_port   | int       | Required | -       | TCP listening port (must not be occupied) |
| udp_options               | map       | Optional | -       | UDP related options |
| udp_options.listen_ip     | string    | Optional | "0.0.0.0" | UDP listening IP |
| udp_options.listen_port   | int       | Required | -       | UDP listening port (must not be occupied) |
| udp_options.max_pkg_size  | int       | Optional | 1024    | Maximum UDP packet size (theoretically cannot exceed 65515) |



Regarding the configuration of **net_plugin**, the following points should be noted:
- `thread_num` indicates the number of threads used by the net plugin.
- `http_options` is optional, but HTTP-related features (such as HTTP Channel backend and HTTP RPC backend) can only be enabled when this node is configured.
  - `http_options.listen_ip` configures the listening address for HTTP service (default is "0.0.0.0"). To listen only on a specific NIC, change it to the specified IP.
  - `http_options.listen_port` configures the listening port for HTTP service (required). Users must ensure the port is not occupied; otherwise, plugin initialization will fail.
- `tcp_options` is optional, but TCP-related features (such as TCP Channel backend) can only be enabled when this node is configured.
  - `tcp_options.listen_ip` configures the listening address for TCP service (default is "0.0.0.0"). To listen only on a specific NIC, change it to the specified IP.
  - `tcp_options.listen_port` configures the listening port for TCP service (required). Users must ensure the port is not occupied; otherwise, plugin initialization will fail.
- `udp_options` is optional, but UDP-related features (such as UDP Channel backend) can only be enabled when this node is configured.
  - `udp_options.listen_ip` configures the listening address for UDP service (default is "0.0.0.0"). To listen only on a specific NIC, change it to the specified IP.
  - `udp_options.listen_port` configures the listening port for UDP service (required). Users must ensure the port is not occupied; otherwise, plugin initialization will fail.
  - `udp_options.max_pkg_size` configures the maximum UDP packet size (theoretically cannot exceed 65515). The serialized message size must be smaller than this value; otherwise, publishing will fail.


Additionally, when using **net_plugin**, the Channel subscription callback, RPC Server processing callback, and RPC Client return all use the plugin's own thread executor. If users block threads in callbacks, it may exhaust the **net_plugin** thread pool, preventing further message reception/transmission. As mentioned in the Module interface documentation, generally speaking:
- If the task in the callback is very lightweight, it can be processed directly in the callback.
- If the task is heavy, it's better to schedule it to another dedicated task executor for processing.


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
## HTTP Type RPC Backend

The `http` type RPC backend is an RPC backend provided in **net_plugin**, used to invoke and handle RPC requests via HTTP. All its configuration items are as follows:

| Node                          | Type      | Optional | Default | Purpose |
| ----                          | ----      | ----     | ----    | ----    |
| clients_options               | array     | Optional | []      | Rules for clients when initiating RPC requests |
| clients_options[i].func_name  | string    | Required | ""      | RPC Func name, supports regular expressions |
| clients_options[i].server_url | string    | Required | ""      | URL to request when the RPC Func is invoked |

Here is a simple client example:
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
            listen_port: 50081
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

And here is a simple server example:
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

In the above examples, the Server listens on the local port 50080, while the Client is configured to route all RPC requests through the http backend to `http://127.0.0.1:50080`, which is the address the server listens on, thereby completing the RPC call loop.

When the Client initiates a call to the Server, it follows this format:
- Uses HTTP POST method, with the request or response packet data serialized in the Body;
- The `content-type` Header defines the serialization method of the Body content, for example:
  - `content-type:application/json`
  - `content-type:application/protobuf`
- URL encoding format: `http://{IP:PORT}/rpc/{FUNC_NAME}`:
  - `{IP:PORT}`: The peer's network address;
  - `{FUNC_NAME}`: URL-encoded RPC func name, as specified during RPC registration;

As long as this format is followed, users can initiate calls using tools like PostMan or Curl. Using JSON as the serialization method enhances readability. For example, the following command can send a message to a specified module to trigger the corresponding callback:
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.example.ExampleService/GetFooData' \
    -d '{"msg": "test msg"}'
```

Based on this feature, the `http` type RPC backend is commonly used during debugging, development, or testing phases. It allows quick triggering of a Server processing function via auxiliary tools or inspecting communication content between upstream and downstream using packet capture tools.## HTTP Type Channel Backend

The `http` type Channel backend is a Channel backend provided by **net_plugin**, used to publish and subscribe messages via HTTP. All its configuration items are as follows:

| Node                                  | Type          | Optional | Default | Description |
| ----                                  | ----          | ----     | ----    | ----        |
| pub_topics_options                    | array         | Optional | []      | Rules for publishing Topics |
| pub_topics_options[i].topic_name      | string        | Required | ""      | Topic name, supports regular expressions |
| pub_topics_options[i].server_url_list | string array  | Required | []      | List of URLs to which the Topic needs to be sent |

Here is a simple publisher example:
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
            listen_port: 50081
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

Here is a simple subscriber example:
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

In the above examples, the subscriber listens on the local port 50080, while the publisher is configured to send all Topic messages via the HTTP backend to the address `127.0.0.1:50080`, which is the address the server listens on. The subscriber is also configured to allow all messages to trigger callbacks via the HTTP backend, thereby establishing the message publish-subscribe pipeline.

When the publisher sends messages to the subscriber, it follows the format below:
- Uses HTTP POST method, with the serialized message data in the Body;
- The `content-type` Header defines the serialization format of the Body content, for example:
  - `content-type:application/json`
  - `content-type:application/protobuf`
- URL encoding format: `http://{IP:PORT}/channel/{TOPIC_NAME}/{MSG_TYPE}`:
  - `{IP:PORT}`: The network address of the peer;
  - `{TOPIC_NAME}`: URL-encoded Topic name;
  - `{MSG_TYPE}`: URL-encoded message type name, as defined in TypeSupport;

As long as this format is followed, users can initiate calls using tools like PostMan or Curl, with JSON as the serialization method for better readability. For example, the following command can send a message to the specified module to trigger the corresponding callback:
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsg' \
    -d '{"msg": "test msg", "num": 123}'
```

Based on this feature, the `http` type Channel backend is often used in debugging, development, or testing phases. It allows quick triggering of specific callbacks using auxiliary tools or inspecting communication content between upstream and downstream via packet capture tools.

## TCP Type Channel Backend

The `tcp` type Channel backend is a Channel backend provided by **net_plugin**, used to publish and subscribe messages via TCP. All its configuration items are as follows:

| Node                                  | Type          | Optional | Default | Description |
| ----                                  | ----          | ----     | ----    | ----        |
| pub_topics_options                    | array         | Optional | []      | Rules for publishing Topics |
| pub_topics_options[i].topic_name      | string        | Required | ""      | Topic name, supports regular expressions |
| pub_topics_options[i].server_url_list | string array  | Required | []      | List of URLs to which the Topic needs to be sent |

Here is a simple publisher example:
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
            listen_port: 50081
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

Here is a simple subscriber example:
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

In the above examples, the subscriber listens on the local port 50080, while the publisher is configured to send all Topic messages via the TCP backend to the address `127.0.0.1:50080`, which is the address the server listens on, thereby establishing the message publish-subscribe pipeline.

Note that when using the TCP backend for message transmission, the data format is proprietary but will maintain version compatibility.## UDP Type Channel Backend

The `udp` type Channel backend is a Channel backend provided in **net_plugin**, used for publishing and subscribing to messages via UDP. All its configuration items are as follows:

| Node                                  | Type          | Optional | Default | Purpose |
| ----                                  | ----          | ----     | ----    | ----    |
| pub_topics_options                    | array         | Optional | []      | Rules when publishing Topics |
| pub_topics_options[i].topic_name      | string        | Required | ""      | Topic name, supports regular expressions |
| pub_topics_options[i].server_url_list | string array  | Required | []      | List of URLs to which the Topic needs to be sent |

Here is a simple example of a publisher:
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
            listen_port: 50081
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

Here is a simple example of a subscriber:
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

In the above examples, the subscriber listens on the local port 50080, while the publisher is configured to send all Topic messages via the UDP backend to the address `127.0.0.1:50080`, which is the address the server is listening on, thereby establishing the message publish-subscribe link.

Note that when using the UDP backend to transmit messages, the data format is proprietary, but the data format will maintain version compatibility.