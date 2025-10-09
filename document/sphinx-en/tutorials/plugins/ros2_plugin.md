# ROS2 Plugin

## Related Links

Reference example:
- {{ '[ros2_plugin]({}/src/examples/plugins/ros2_plugin)'.format(code_site_root_path_url) }}


## Plugin Overview


**ros2_plugin** is a communication transport plugin implemented based on [ROS2 Humble](https://docs.ros.org/en/humble/index.html). This plugin provides the following components:
- `ros2`-type RPC backend
- `ros2`-type Channel backend

The plugin configuration items are as follows:

| Node                    | Type   | Optional | Default Value   | Purpose                                                                               |
| ----------------------- | ------ | -------- | --------------- | ---------------------------------------------------------------------------------- |
| node_name               | string | Required | ""              | ROS2 node name                                                                      |
| executor_type           | string | Optional | "MultiThreaded" | ROS2 executor type, optional values: "SingleThreaded", "StaticSingleThreaded", "MultiThreaded" |
| executor_thread_num     | int    | Optional | 2               | When executor_type == "MultiThreaded", indicates the number of threads for the ROS2 executor                   |
| auto_initialize_logging | bool   | Optional | true            | Whether to initialize ROS2's default SPLOG logging system                                              |

Regarding the configuration of **ros2_plugin**, the usage notes are as follows:
- `node_name` represents the ROS2 node name. From an external perspective, an AimRT node that has loaded the ROS2 plugin is a ROS2 node, and its node name is configured according to this item.
- `executor_type` represents the type of the ROS2 node executor. Currently, there are three choices: `SingleThreaded`, `StaticSingleThreaded`, and `MultiThreaded`. For specific meanings, please refer to the ROS2 Humble documentation.
- `executor_thread_num` only takes effect when the `executor_type` value is `MultiThreaded`, indicating the number of threads for ROS2.
- `auto_initialize_logging` indicates whether to initialize ROS2's default SPLOG logging system. If set to `true`, the ROS2 default SPLOG logging system will be used, and related logs will be stored in the directory determined by the environment variable ROS_LOG_DIR.


Additionally, when using **ros2_plugin**, the executor provided by **ros2_plugin** is used for Channel subscription callbacks, RPC Server processing, and RPC Client returns. When users block threads in callbacks, it may lead to exhaustion of the **ros2_plugin** thread pool, preventing further message reception/sending. As mentioned in the Module interface documentation, generally, if the task in the callback is very lightweight, it can be processed directly in the callback; however, if the task in the callback is relatively heavy, it is best to schedule it to another dedicated executor for processing.


Here is a simple example:

```yaml
aimrt:
  plugin:
    plugins:
      - name: ros2_plugin
        path: ./libaimrt_ros2_plugin.so
        options:
          node_name: example_ros2_node
          executor_type: MultiThreaded
          executor_thread_num: 4
```
## ros2 Type RPC Backend


The `ros2` type RPC backend is an RPC backend provided by **ros2_plugin**, used to invoke and handle AimRT RPC requests via the ROS2 RPC mechanism. All of its configuration items are as follows:

| Node                                             | Type   | Optional | Default   | Description                                                                                                                                                                                              |
| ------------------------------------------------ | ------ | -------- | --------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timeout_executor                                 | string | Optional | ""        | Executor used on the client side when an RPC timeout occurs                                                                                                                                                                  |
| clients_options                                  | array  | Optional | []        | Rules for initiating RPC requests on the client side                                                                                                                                                                       |
| clients_options[i].func_name                     | string | Required | ""        | RPC Func name, supports regular expressions                                                                                                                                                                     |
| clients_options[i].qos                           | map    | Optional | -         | QOS configuration                                                                                                                                                                                          |
| clients_options[i].qos.history                   | string | Optional | "default" | QOS history option<br/>keep_last: keep the most recent records (cache up to N records, configurable via queue length)<br/>keep_all: keep all records (cache all records, limited by the underlying middleware's maximum configurable resources)<br/>default: use system default |
| clients_options[i].qos.depth                     | int    | Optional | 10        | QOS queue depth option (only works with keep_last)                                                                                                                                                     |
| clients_options[i].qos.reliability               | string | Optional | "default" | QOS reliability option<br/>reliable: reliable (when messages are lost, they are resent repeatedly to ensure successful data transmission)<br/>best_effort: best effort (attempts to transmit data without guaranteeing successful delivery; may lose data when the network is unstable)<br/>default: system default  |
| clients_options[i].qos.durability                | string | Optional | "default" | QOS durability option<br/>transient_local: transient local (publisher retains data for late-joining subscribers)<br/>volatile: volatile (does not retain any data)<br/>default: system default                                              |
| clients_options[i].qos.deadline                  | int    | Optional | -1        | QOS expected maximum time between subsequent message publications to the topic option<br/>Fill in millisecond-level time interval, -1 means not set, use system default                                                                                             |
| clients_options[i].qos.lifespan                  | int    | Optional | -1        | QOS maximum time between message publication and reception (in milliseconds) option<br/>before the message is considered stale or expired (expired messages are silently discarded and never actually received)<br/>-1 keeps system default, not set                                         |
| clients_options[i].qos.liveliness                | string | Optional | "default" | QOS option for determining whether a publisher is alive<br/>automatic: automatic (ROS2 determines based on the time interval between message publication and reception)<br/>manual_by_topic: requires the publisher to declare periodically<br/>default: keep system default                                    |
| clients_options[i].qos.liveliness_lease_duration | int    | Optional | -1        | QOS liveliness lease duration in milliseconds option; if the publisher does not declare liveliness within this time, it is considered inactive<br/>-1 keeps system default, not set                                                                          |
| clients_options[i].remapping_rule                | string | Optional | ""        | Used to remap the func_name matched by the regular expression in clients_options[i].func_name according to new rules to generate a new ros2_func_name <br/>Supports replacement rules when writing: {j} represents the j-th captured group matched by the regular expression                  || servers_options                                  | array  | Optional | []        | Rules for the server when receiving and processing RPC requests                                                                                                                                                                   |
| servers_options[i].func_name                     | string | Required | ""        | RPC Func name, supports regular expressions                                                                                                                                                                     |
| servers_options[i].qos                           | map    | Optional | -         | QOS configuration                                                                                                                                                                                          |
| servers_options[i].qos.history                   | string | Optional | "default" | QOS history option<br/>keep_last: keep the most recent records (cache up to N records, configurable via queue length)<br/>keep_all: keep all records (cache all records, limited by the underlying middleware's maximum resources)<br/>default: use system default |
| servers_options[i].qos.depth                     | int    | Optional | 10        | QOS queue depth option (only works with keep_last)                                                                                                                                                     |
| servers_options[i].qos.reliability               | string | Optional | "default" | QOS reliability option<br/>reliable: reliable (re-sends messages if lost, retries until successful)<br/>best_effort: best effort (attempts delivery but does not guarantee success; may lose data on unstable networks)<br/>default: system default  |
| servers_options[i].qos.durability                | string | Optional | "default" | QOS durability option<br/>transient_local: transient local (publisher keeps data for late-joining subscribers)<br/>volatile: volatile (does not keep any data)<br/>default: system default                                              |
| servers_options[i].qos.deadline                  | int    | Optional | -1        | QOS expected maximum time between subsequent messages published to the topic<br/>Specify in milliseconds; use -1 to keep system default                                                                                             |
| servers_options[i].qos.lifespan                  | int    | Optional | -1        | QOS maximum time (in milliseconds) between message publication and reception<br/>after which the message is considered stale or expired (expired messages are silently dropped and never received)<br/>use -1 to keep system default                                         |
| servers_options[i].qos.liveliness                | string | Optional | "default" | QOS option for determining whether a publisher is alive<br/>automatic: automatic (ROS2 judges based on message publication/reception intervals)<br/>manual_by_topic: publisher must declare periodically<br/>default: keep system default                                    |
| servers_options[i].qos.liveliness_lease_duration | int    | Optional | -1        | QOS liveliness lease duration (in milliseconds); if the publisher does not declare liveliness within this time, it is considered inactive<br/>use -1 to keep system default                                                                                                        |
| servers_options[i].remapping_rule                | string | Optional | ""        | Remaps the func_name matched by the regex in servers_options[i].func_name according to the new rule to generate a new ros2_func_name <br/>Supports replacement rules: {j} represents the j-th regex capture group                  |


Here is a simple example of remap usage: for an AimRT func_name named `pb:/aimrt_server/GetFooData`, if no remap is needed, the final generated ros func_name is `/aimrt_5Fserver/GetFooData` (note that the `<msg_type>` before and including ":" is removed, and any characters that are not digits, letters, or '/' are encoded as HEX with '_' prefix). If remap is required, you can configure as follows:


```yaml
rpc:
  backends:
    - type: ros2
      options:
        servers_options:
          - func_name: "(.*)/(.*)/(.*)" #这里是填写匹配 AimRT func_name 的正则表达式
            remapping_rule: "{1}/{2}" # 这里填写重映射规则，用于生成新的ros2_func_name。 这里也可以简化写成 /{2}

```

With this configuration, the first `(.*)` captures `pb:`, the second `(.*)` captures `aimrt_server`, and the third `(.*)` captures `GetFooData`; the final generated ros func_name becomes `/GetFooData`. To simplify writing, you may omit `{1}` (the message type) in the `remapping_rule`; the system will automatically generate it to adapt the conversion between AimRT func_name and ros2 func_name. Below are some quick examples demonstrating richer usage, assuming the AimRT func_name is `pb:/aaa/bbb/ccc`:

| func_name               | remapping_rule  | ros2_func_name |
| ----------------------- | --------------- | -------------- |
| (.\*)/(.\*)/(.\*)/(.\*) |                 | /aaa/bbb/ccc   |
| (.\*)/(.\*)/(.\*)/(.\*) | {1}/{2}/ddd/{4} | /aaa/ddd/ccc   |
| (.\*)/(.\*)/(.\*)/(.\*) | /{2}/ddd/{4}    | /aaa/ddd/ccc   |
| (.\*)/(.\*)/(bbb)/(.\*) | {1}/{3}_{4}     | /bbb_5Fccc     |
| (.\*)/(.\*)/(bbb)/(.\*) | /{3}/eee        | /bbb/eee       |


Below is a simple client example:

```yaml
aimrt:
  plugin:
    plugins:
      - name: ros2_plugin
        path: ./libaimrt_ros2_plugin.so
        options:
          node_name: example_ros2_client_node
          executor_type: MultiThreaded
          executor_thread_num: 4
  rpc:
    backends:
      - type: ros2
        options:
          clients_options:
            - func_name: "(.*)"
              qos:
                history: keep_last
                depth: 10
                reliability: reliable
                durability: volatile
                deadline: -1
                lifespan: -1
                liveliness: automatic
                liveliness_lease_duration: -1
    clients_options:
      - func_name: "(.*)"
        enable_backends: [ros2]
```


Below is a simple server example:

```yaml
aimrt:
  plugin:
    plugins:
      - name: ros2_plugin
        path: ./libaimrt_ros2_plugin.so
        options:
          node_name: example_ros2_server_node
          executor_type: MultiThreaded
          executor_thread_num: 4
  rpc:
    backends:
      - type: ros2
        options:
          servers_options:
            - func_name: "(.*)"
              qos:
                history: keep_last
                depth: 10
                reliability: reliable
                durability: volatile
                deadline: -1
                lifespan: -1
                liveliness: automatic
                liveliness_lease_duration: -1
    servers_options:
      - func_name: "(.*)"
        enable_backends: [ros2]
```


In the examples above, the server side starts a ROS2 node named `example_ros2_server_node`, and the client side starts a ROS2 node named `example_ros2_client_node`. The client initiates an RPC call through the ROS2 backend, and the server receives the RPC request via the ROS2 backend and processes it.

When the client calls the server, if the protocol layer is the native ROS2 protocol, the communication will fully reuse the native ROS2 protocol, and native ROS2 nodes can seamlessly interface with AimRT nodes based on this protocol.

If the client does not use the ROS2 protocol when calling the server, the communication will be wrapped using the ROS2 protocol defined in {{ '[RosRpcWrapper.srv]({}/src/protocols/plugins/ros2_plugin_proto/srv/RosRpcWrapper.srv)'.format(code_site_root_path_url) }}. The content of this protocol is as follows:

```
string  serialization_type
string[]  context
byte[]  data
---
int64   code
string  serialization_type
byte[]  data
```


At this point, if a native ROS2 node needs to interface with an AimRT node, the developer of the native ROS2 node must serialize/deserialize the actual request/response packets in conjunction with the `data` field of this protocol's Req/Rsp.

Additionally, because ROS2 has special requirements for `service_name`, the `service_name` used when AimRT communicates with ROS2 is generated by AimRT Func according to a rule similar to URL encoding:
- All symbols except digits, letters, and '/' are encoded as their ASCII values in HEX, prefixed with '_'.

For example, if the AimRT Func name is `/aaa.bbb.ccc/ddd`, the encoded ROS2 service name becomes `/aaa_2Ebbb_2Eccc/ddd`. The exact value is also printed when the ros2_plugin starts.

Based on these features, the `ros2` type RPC backend can be used to bridge the RPC link with native ROS2 nodes, thereby achieving compatibility between AimRT and ROS2.## ros2 Type Channel Backend

The `ros2` type Channel backend is a Channel backend provided by **ros2_plugin**, used to publish and subscribe to AimRT Channel messages via ROS2 Topics. All its configurations are as follows:

| Node                                                | Type   | Optional | Default   | Purpose                                                                                                                                                                                              |
| --------------------------------------------------- | ------ | -------- | --------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| pub_topics_options                                  | array  | Optional | []        | Rules when publishing Topics                                                                                                                                                                               |
| pub_topics_options[i].topic_name                    | string | Required | ""        | Topic name, supports regular expressions                                                                                                                                                                        |
| pub_topics_options[i].use_serialized                | bool   | Optional | "false"   | Controls the format of published data, only effective for ros2msg                                                                           |
| pub_topics_options[i].qos                           | map    | Optional | -         | QOS configuration                                                                                                                                                                                          |
| pub_topics_options[i].qos.history                   | string | Optional | "default" | QOS history record option<br/>keep_last: keep the most recent records (cache up to N records, configurable via queue length option)<br/>keep_all: keep all records (cache all records, but limited by the maximum configurable resources of the underlying middleware)<br/>default: use system default |
| pub_topics_options[i].qos.depth                     | int    | Optional | 10        | QOS queue depth option (can only be used with Keep_last)                                                                                                                                                       |
| pub_topics_options[i].qos.reliability               | string | Optional | "default" | QOS reliability option<br/>reliable: reliable (when messages are lost, they will be resent repeatedly to ensure successful data transmission)<br/>best_effort: best effort (attempts to transmit data but does not guarantee successful transmission, may lose data when network is unstable)<br/>default: system default  |
| pub_topics_options[i].qos.durability                | string | Optional | "default" | QOS durability option<br/>transient_local: transient local (publisher retains data for late-joining subscribers)<br/>volatile: volatile (does not retain any data)<br/>default: system default                                              |
| pub_topics_options[i].qos.deadline                  | int    | Optional | -1        | QOS expected maximum time between subsequent message publications to the topic option<br/>needs to be filled in milliseconds, -1 means not set, use system default                                                                                             |
| pub_topics_options[i].qos.lifespan                  | int    | Optional | -1        | QOS maximum time between message publication and reception option<br/>before the message is considered stale or expired (expired messages are silently discarded and effectively never received<br/>-1 keeps system default not set                                         |
| pub_topics_options[i].qos.liveliness                | string | Optional | "default" | QOS how to determine if publisher is active option<br/>automatic: automatic (ROS2 determines based on time intervals between message publication and reception)<br/>manual_by_topic: requires publisher to periodically declare<br/>default: keep system default                                    |
| pub_topics_options[i].qos.liveliness_lease_duration | int    | Optional | -1        | QOS liveliness lease duration option in milliseconds, if the publisher does not declare activity within this time, it is considered inactive<br/>-1 keeps system default not set                                                                          |
| sub_topics_options                                  | array  | Optional | []        | Rules when subscribing to Topics                                                                                                                                                                               || sub_topics_options[i].topic_name                    | string | Required | ""        | Topic name, supports regular expressions                                                                                                                                                                        |
| sub_topics_options[i].use_serialized                | bool   | Optional | "false"   | Controls the data format received by the subscription callback, only valid for ros2msg                                                                                                               |
| sub_topics_options[i].qos                           | map    | Optional | -         | QOS configuration                                                                                                                                                                                          |
| sub_topics_options[i].qos.history                   | string | Optional | "default" | QOS history option<br/>keep_last: keep the most recent records (buffers up to N records, configurable via queue length)<br/>keep_all: keep all records (buffers all records, limited by underlying middleware's maximum resources)<br/>default: use system default |
| sub_topics_options[i].qos.depth                     | int    | Optional | 10        | QOS queue depth option (only works with keep_last)                                                                                                                                                       |
| sub_topics_options[i].qos.reliability               | string | Optional | "default" | QOS reliability option<br/>reliable: reliable (retransmits lost messages to ensure successful delivery)<br/>best_effort: best effort (attempts delivery without guarantee, may lose data when network is unstable)<br/>default: system default  |
| sub_topics_options[i].qos.durability                | string | Optional | "default" | QOS durability option<br/>transient_local: transient local (publisher keeps data for late-joining subscribers)<br/>volatile: volatile (does not keep any data)<br/>default: system default                                              |
| sub_topics_options[i].qos.deadline                  | int    | Optional | -1        | QOS expected maximum time between subsequent messages published to the topic<br/>Fill in milliseconds, -1 means not set, use system default                                                                                             |
| sub_topics_options[i].qos.lifespan                  | int    | Optional | -1        | QOS maximum time between message publication and reception (in milliseconds)<br/>Messages exceeding this are considered stale/expired (expired messages are silently dropped and never received)<br/>-1 keeps system default (not set)                                         |
| sub_topics_options[i].qos.liveliness                | string | Optional | "default" | QOS how to determine if publisher is alive<br/>automatic: automatic (ROS2 determines based on message publish/receive intervals)<br/>manual_by_topic: requires publisher to periodically assert<br/>default: keep system default                                    |
| sub_topics_options[i].qos.liveliness_lease_duration | int    | Optional | -1        | QOS liveliness lease duration (in milliseconds), if publisher doesn't assert liveliness within this time it's considered inactive<br/>-1 keeps system default (not set)                                                                        |


以下是一个简单的发布端的示例：

```yaml
aimrt:
  plugin:
    plugins:
      - name: ros2_plugin
        path: ./libaimrt_ros2_plugin.so
        options:
          node_name: example_ros2_pub_node
          executor_type: MultiThreaded
          executor_thread_num: 4
  channel:
    backends:
      - type: ros2
        options:
          pub_topics_options:
            - topic_name: "(.*)"
              use_serialized: false
              qos:
                history: keep_last
                depth: 10
                reliability: reliable
                durability: volatile
                deadline: -1
                lifespan: -1
                liveliness: automatic
                liveliness_lease_duration: -1
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [ros2]
```


以下则是一个简单的订阅端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: ros2_plugin
        path: ./libaimrt_ros2_plugin.so
        options:
          node_name: example_ros2_sub_node
          executor_type: MultiThreaded
          executor_thread_num: 4
  channel:
    backends:
      - type: ros2
        options:
          sub_topics_options:
            - topic_name: "(.*)"
              use_serialized: false
              qos:
                history: keep_last
                depth: 10
                reliability: reliable
```                durability: volatile
                deadline: -1
                lifespan: -1
                liveliness: automatic
                liveliness_lease_duration: -1
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [ros2]

```

以上示例中，发布端启动了一个 ROS2 节点`example_ros2_pub_node`，订阅端则启动了一个 ROS2 节点`example_ros2_sub_node`，发布端通过 ROS2 的后端发布消息，订阅端通过 ROS2 后端接收到消息并进行处理。

如果消息发布订阅时，协议层是原生 ROS2 协议，那么通信时将完全复用 ROS2 的原生协议，原生 ROS2 节点可以基于该协议无缝与 AimRT 节点对接。

如果消息发布订阅时，协议层没有使用 ROS2 协议，那么通信时将基于{{ '[RosMsgWrapper.msg]({}/src/protocols/plugins/ros2_plugin_proto/msg/RosMsgWrapper.msg)'.format(code_site_root_path_url) }}这个 ROS2 协议进行包装，该协议内容如下：
```

string  serialization_type
string[]  context
byte[]  data
```

If a native ROS2 node needs to communicate with an AimRT node at this point, the developer of the native ROS2 node must work with the `data` field of this protocol to serialize/deserialize the actual message.

In addition, when AimRT and ROS2 interoperate, the `Topic` name is generated according to the following rule: `${aimrt_topic}/${ros2_encode_aimrt_msg_type}`. Here, `${aimrt_topic}` is the AimRT Topic name, and `${ros2_encode_aimrt_msg_type}` is derived from the AimRT Msg name using a URL-encoding-like rule:
- All symbols except digits, letters, and '/' are encoded as their ASCII HEX values, prefixed with '_'.

For example, if the AimRT Topic name is `test_topic` and the AimRT Msg name is `pb:aaa.bbb.ccc`, the final ROS2 Topic value will be `test_topic/pb_3Aaaa_2Ebbb_2Eccc`. The exact value is also printed when the ros2_plugin starts.

Thanks to this feature, the `ros2`-type Channel backend can be used to open up the Channel link with native ROS2 nodes, thereby achieving AimRT compatibility with ROS2.

Developers can also refer to the example in {{ '[ros2_plugin]({}/src/examples/plugins/ros2_plugin)'.format(code_site_root_path_url) }} to communicate with native ros2 humble nodes.