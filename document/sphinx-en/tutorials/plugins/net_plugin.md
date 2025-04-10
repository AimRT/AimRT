

# Net Plugin

## Related Links

Reference Example:
- {{ '[net_plugin]({}/src/examples/plugins/net_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**net_plugin** is a network transmission plugin implemented based on boost asio/beast library, providing the following components:
- `http` type RPC backend
- `http` type Channel backend
- `tcp` type Channel backend
- `udp` type Channel backend

The plugin configuration items are as follows:

| Node                     | Type      | Optional | Default Value | Purpose |
| ----                     | ----      | ----     | ----          | ----    |
| thread_num               | int       | Required | 2             | Number of threads used by net plugin |
| http_options             | map       | Optional | -             | HTTP related options |
| http_options.listen_ip   | string    | Optional | "0.0.0.0"     | HTTP listening IP |
| http_options.listen_port | int       | Required | -             | HTTP listening port (must be available) |
| tcp_options              | map       | Optional | -             | TCP related options |
| tcp_options.listen_ip    | string    | Optional | "0.0.0.0"     | TCP listening IP |
| tcp_options.listen_port  | int       | Required | -             | TCP listening port (must be available) |
| udp_options              | map       | Optional | -             | UDP related options |
| udp_options.listen_ip    | string    | Optional | "0.0.0.0"     | UDP listening IP |
| udp_options.listen_port  | int       | Required | -             | UDP listening port (must be available) |
| udp_options.max_pkg_size | int       | Optional | 1024          | Maximum UDP packet size (theoretical maximum 65515) |

Configuration notes for **net_plugin**:
- `thread_num` indicates the number of threads used by net plugin.
- `http_options` is optional, but http-related features (http Channel backend, http RPC backend) are only enabled when configured:
  - `http_options.listen_ip` configures HTTP service listening address (default "0.0.0.0"). Use specific IP for dedicated NIC.
  - `http_options.listen_port` configures HTTP listening port (required). Ensure port availability to avoid initialization failure.
- `tcp_options` is optional, but tcp-related features (tcp Channel backend) are only enabled when configured:
  - `tcp_options.listen_ip` configures TCP listening address (default "0.0.0.0"). Use specific IP for dedicated NIC.
  - `tcp_options.listen_port` configures TCP listening port (required). Ensure port availability to avoid initialization failure.
- `udp_options` is optional, but udp-related features (udp Channel backend) are only enabled when configured:
  - `udp_options.listen_ip` configures UDP listening address (default "0.0.0.0"). Use specific IP for dedicated NIC.
  - `udp_options.listen_port` configures UDP listening port (required). Ensure port availability to avoid initialization failure.
  - `udp_options.max_pkg_size` configures maximum UDP packet size. Serialized messages must be smaller than this value.

Additional usage notes:
- Callbacks for Channel subscriptions, RPC Server processing, and RPC Client responses all use **net_plugin**'s internal thread executor. Blocking these threads may exhaust the thread pool and prevent message processing.
- As described in Module interface documentation:
  - Handle lightweight tasks directly in callbacks
  - Schedule heavy tasks to dedicated executors

Example configuration:
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

## HTTP-Type RPC Backend

The `http`-type RPC backend provided by **net_plugin** enables RPC invocation and processing through HTTP protocol. All configuration options are as follows:

| Node                          | Type      | Optional | Default | Description |
| ----                          | ----      | ----     | ----    | ----        |
| clients_options               | array     | Yes      | []      | Rules for client-initiated RPC requests |
| clients_options[i].func_name  | string    | Required | ""      | RPC function name supporting regular expressions |
| clients_options[i].server_url | string    | Required | ""      | Target URL for RPC function invocation |

Below is a simple client configuration example:
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

Here's a corresponding server configuration example:
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

In the examples above, the server listens on local port 50080, while the client configures all RPC requests to be routed through the http backend to `http://127.0.0.1:50080`, completing the RPC call loop.

Client invocation follows these specifications:
- Uses HTTP POST method with serialized request/response data in the body
- `Content-Type` header defines body serialization format, e.g.:
  - `Content-Type: application/json`
  - `Content-Type: application/protobuf`
- URL format: `http://{IP:PORT}/rpc/{FUNC_NAME}`:
  - `{IP:PORT}`: Target network address
  - `{FUNC_NAME}`: URL-encoded RPC function name (matches registered RPC name)

This format allows invocation using tools like PostMan or Curl. For improved readability with JSON serialization, use commands like:
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.example.ExampleService/GetFooData' \
    -d '{"msg": "test msg"}'
```

Based on this feature, the `http` type RPC backend is commonly used during debugging, development, or testing phases. It allows quick triggering of server processing functions through auxiliary tools, or inspection of communication content between upstream and downstream using packet capture tools.

## HTTP Type Channel Backend

The `http` type Channel backend is one of the Channel backends provided by **net_plugin**, used for publishing and subscribing messages via HTTP. All its configuration items are as follows:

| Node                                  | Type          | Optional | Default | Description |
| ----                                  | ----          | ----     | ----    | ----        |
| pub_topics_options                    | array         | Optional | []      | Rules for publishing Topics |
| pub_topics_options[i].topic_name      | string        | Required | ""      | Topic name, supports regular expressions |
| pub_topics_options[i].server_url_list | string array  | Required | []      | List of URLs to which the Topic should be sent |

Here is a simple example of a publisher configuration:
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

In the above examples, the subscriber listens on local port 50080, while the publisher is configured to send all Topic messages through the http backend to the address `127.0.0.1:50080`, which is the address the server listens on. The subscriber is also configured to trigger callbacks for all messages from the http backend, thus establishing the message publish-subscribe pipeline.

When the publisher sends messages to the subscriber, it follows this format:
- Use HTTP POST method with serialized message data in the Body
- Use the `content-type` Header to define the serialization format of the Body content, for example:
  - `content-type:application/json`
  - `content-type:application/protobuf`
- URL encoding format: `http://{IP:PORT}/channel/{TOPIC_NAME}/{MSG_TYPE}`:
  - `{IP:PORT}`: The network address of the peer;
  - `{TOPIC_NAME}`: URL-encoded Topic name;
  - `{MSG_TYPE}`: URL-encoded message type name, where the message type name is defined in TypeSupport;

By following this format, users can initiate calls using tools like PostMan or Curl, using JSON as the serialization format for better readability. For example, the following command can be used to send a message to a specified module to trigger the corresponding callback:
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsg' \
    -d '{"msg": "test msg", "num": 123}'
```

Based on this feature, the `http` type Channel backend is commonly used during debugging, development, or testing phases. It allows quick triggering of callbacks using auxiliary tools, or inspecting communication content between upstream and downstream using packet capture tools.

## TCP Type Channel Backend

The `tcp` type Channel backend is one of the Channel backends provided by **net_plugin**, used for publishing and subscribing messages through TCP. All its configuration items are as follows:

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

In the above examples:
- The subscriber listens on local port 50080
- The publisher configures all Topic messages to be sent via TCP backend to `127.0.0.1:50080` (the server listening address)
- This establishes the message pub-sub communication channel

Note: When using TCP backend for message transmission:
- The data format is proprietary but will maintain version compatibility
- Applications must ensure both communication ends use matching protocol versions

## UDP Type Channel Backend

The `udp` type Channel backend is a Channel backend provided by **net_plugin** for publishing and subscribing messages via UDP. All its configuration items are as follows:

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

In the above examples:
- The subscriber listens on local port 50080
- The publisher configures all Topic messages to be sent via UDP backend to `127.0.0.1:50080` (the server listening address)
- This establishes the message pub-sub communication channel

- The data format is proprietary but will maintain version compatibility
- Applications must ensure both communication ends use matching protocol versions