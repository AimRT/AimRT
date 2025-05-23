# ROS2 Plugin

## Related Links

Reference Example:
- {{ '[ros2_plugin]({}/src/examples/plugins/ros2_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**ros2_plugin** is a communication transport plugin implemented based on [ROS2 Humble](https://docs.ros.org/en/humble/index.html). This plugin provides the following components:
- `ros2` type RPC backend
- `ros2` type Channel backend

The plugin configuration items are as follows:

| Node                    | Type   | Optional | Default Value   | Description                                                                        |
| ----------------------- | ------ | -------- | --------------- | ---------------------------------------------------------------------------------- |
| node_name               | string | Required | ""              | ROS2 node name                                                                     |
| executor_type           | string | Optional | "MultiThreaded" | ROS2 executor type, options: "SingleThreaded", "StaticSingleThreaded", "MultiThreaded" |
| executor_thread_num     | int    | Optional | 2               | When executor_type == "MultiThreaded", indicates the thread count of ROS2 executor |
| auto_initialize_logging | bool   | Optional | true            | Whether to initialize ROS2's default SPLOG logging system                          |

Regarding the configuration of **ros2_plugin**, the following points should be noted:
- `node_name` represents the ROS2 node name. From an external perspective, an AimRT node loaded with the ROS2 plugin appears as a ROS2 node, and its node name is configured through this item.
- `executor_type` indicates the type of ROS2 node executor. Currently, there are three options: `SingleThreaded`, `StaticSingleThreaded`, and `MultiThreaded`. For specific meanings, please refer to the ROS2 Humble documentation.
- `executor_thread_num` only takes effect when `executor_type` is set to `MultiThreaded`, indicating the thread count of ROS2.
- `auto_initialize_logging` determines whether to initialize ROS2's default SPLOG logging system. If set to `true`, ROS2's default SPLOG logging system will be used, and related logs will be stored in the directory specified by the environment variable ROS_LOG_DIR.

Additionally, when using **ros2_plugin**, the executor provided by **ros2_plugin** is used for Channel subscription callbacks, RPC Server processing, and RPC Client returns. If the callback blocks the thread, it may exhaust the **ros2_plugin** thread pool, preventing further message reception/sending. As mentioned in the Module interface documentation, generally, if the task in the callback is very lightweight, it can be processed directly in the callback. However, if the task is relatively heavy, it is better to schedule it to another dedicated task executor for processing.

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

The `ros2` type RPC backend is an RPC backend provided by **ros2_plugin**, used to invoke and process AimRT RPC requests through the ROS2 RPC approach. All its configuration items are as follows:

| Node                                              | Type   | Optional | Default   | Description                                                                                                                                                                                       |
| ------------------------------------------------- | ------ | -------- | --------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| timeout_executor                                  | string | Optional | ""        | Executor for RPC timeout scenarios on the Client side                                                                                                                                             |
| clients_options                                   | array  | Optional | []        | Rules for the client when initiating RPC requests                                                                                                                                                |
| clients_options[i].func_name                      | string | Required | ""        | RPC Func name, supports regular expressions                                                                                                                                                      |
| clients_options[i].qos                            | map    | Optional | -         | QOS configuration                                                                                                                                                                                |
| clients_options[i].qos.history                    | string | Optional | "default" | QOS history option<br/>keep_last: Keep the most recent records (caches up to N records, configurable via queue depth option)<br/>keep_all: Keep all records (caches all records, but limited by the maximum resources configurable by the underlying middleware)<br/>default: Use system default |
| clients_options[i].qos.depth                      | int    | Optional | 10        | QOS queue depth option (can only be used with Keep_last)                                                                                                                                         |
| clients_options[i].qos.reliability                | string | Optional | "default" | QOS reliability option<br/>reliable: Reliable (retransmits lost messages repeatedly to ensure successful data transfer)<br/>best_effort: Best effort (attempts to transfer data but does not guarantee successful transfer, may lose data if the network is unstable)<br/>default: System default |
| clients_options[i].qos.durability                 | string | Optional | "default" | QOS durability option<br/>transient_local: Transient local (publisher retains data for late-joining subscribers)<br/>volatile: Volatile (does not retain any data)<br/>default: System default                                   |
| clients_options[i].qos.deadline                   | int    | Optional | -1        | QOS expected maximum time interval (in milliseconds) between subsequent message publications to the topic<br/>Fill in -1 to not set, using system default                                          |
| clients_options[i].qos.lifespan                   | int    | Optional | -1        | QOS maximum time interval (in milliseconds) between message publication and reception<br/>Messages exceeding this interval are considered stale or expired (expired messages are silently discarded and never actually received)<br/>Fill in -1 to not set, using system default |
| clients_options[i].qos.liveliness                 | string | Optional | "default" | QOS option for determining publisher liveliness<br/>automatic: Automatic (ROS2 judges based on the time interval between message publication and reception)<br/>manual_by_topic: Requires the publisher to periodically declare<br/>default: Use system default |
| clients_options[i].qos.liveliness_lease_duration  | int    | Optional | -1        | QOS liveliness lease duration option (in milliseconds). If the publisher does not declare liveliness within this duration, it is considered inactive<br/>Fill in -1 to not set, using system default |
| clients_options[i].remapping_rule                 | string | Optional | ""        | Used to remap the func_name matched by the regular expression in clients_options[i].func_name according to new rules to generate a new ros2_func_name<br/>Supports replacement rules during writing: {j} represents the jth captured group matched by the regular expression || servers_options                                  | array  | Optional | []        | Rules for the server to handle RPC requests                                                                                                                                                      |
| servers_options[i].func_name                     | string | Required | ""        | RPC Func name, supports regular expressions                                                                                                                                                      |
| servers_options[i].qos                           | map    | Optional | -         | QOS configuration                                                                                                                                                                               |
| servers_options[i].qos.history                   | string | Optional | "default" | QOS history options<br/>keep_last: Keep most recent records (cache up to N records, configurable via queue depth)<br/>keep_all: Keep all records (cache all records, subject to underlying middleware's maximum configurable resources)<br/>default: Use system default |
| servers_options[i].qos.depth                     | int    | Optional | 10        | QOS queue depth option (only works with Keep_last)                                                                                                                                              |
| servers_options[i].qos.reliability               | string | Optional | "default" | QOS reliability options<br/>reliable: Reliable (retransmits lost messages to ensure successful delivery)<br/>best_effort: Best effort (attempts delivery but doesn't guarantee it, may lose data during network instability)<br/>default: System default |
| servers_options[i].qos.durability                | string | Optional | "default" | QOS durability options<br/>transient_local: Transient local (publisher retains data for late-joining subscribers)<br/>volatile: Volatile (retains no data)<br/>default: System default           |
| servers_options[i].qos.deadline                  | int    | Optional | -1        | QOS maximum expected time between successive messages on a topic (in milliseconds)<br/>-1 means no setting, uses system default                                                               |
| servers_options[i].qos.lifespan                  | int    | Optional | -1        | QOS maximum duration between message publication and acceptance (in milliseconds)<br/>Expired messages are silently discarded<br/>-1 means no setting, uses system default                     |
| servers_options[i].qos.liveliness                | string | Optional | "default" | QOS publisher liveliness determination method<br/>automatic: Automatic (ROS2 determines based on message publication/reception intervals)<br/>manual_by_topic: Requires periodic publisher declaration<br/>default: System default |
| servers_options[i].qos.liveliness_lease_duration | int    | Optional | -1        | QOS liveliness lease duration (in milliseconds)<br/>If publisher doesn't declare liveliness within this duration, it's considered inactive<br/>-1 means no setting, uses system default         |
| servers_options[i].remapping_rule                | string | Optional | ""        | Used to remap the func_name matched by servers_options[i].func_name regex to generate a new ros2_func_name<br/>Supports replacement rules in format: {j} represents the j-th regex capture group |

Here's a simple example of remap usage: For an AimRT func_name `pb:/aimrt_server/GetFooData`, if no remap is needed, the final generated ros func_name would be `/aimrt_5Fserver/GetFooData` (note that the "<msg_type>" before ":" is removed, and non-alphanumeric/'/' characters are HEX encoded with '_' prefix). If remapping is needed, configure as follows:

```yaml
rpc:
  backends:
    - type: ros2
      options:
        servers_options:
          - func_name: "(.*)/(.*)/(.*)" #这里是填写匹配 AimRT func_name 的正则表达式
            remapping_rule: "{1}/{2}" # 这里填写重映射规则，用于生成新的ros2_func_name。 这里也可以简化写成 /{2}

```
With this configuration, the first `(.*)` captures `pb:`, the second `(.*)` captures `aimrt_server`, and the third `(.*)` captures `GetFooData`. The final generated ros func_name becomes `/GetFooData`. To simplify writing, the `remapping_rule` option can omit the message type represented by `{1}`, as the system will automatically generate it to adapt the conversion between AimRT func_name and ros2 func_name. Below are some quick examples demonstrating richer usage, assuming the AimRT func_name is `pb:/aaa/bbb/ccc`:

| func_name               | remapping_rule  | ros2_func_name |
| ----------------------- | --------------- | -------------- |
| (.\*)/(.\*)/(.\*)/(.\*) |                 | /aaa/bbb/ccc   |
| (.\*)/(.\*)/(.\*)/(.\*) | {1}/{2}/ddd/{4} | /aaa/ddd/ccc   |
| (.\*)/(.\*)/(.\*)/(.\*) | /{2}/ddd/{4}    | /aaa/ddd/ccc   || (.\*)/(.\*)/(bbb)/(.\*) | {1}/{3}_{4}     | /bbb_5Fccc     |
| (.\*)/(.\*)/(bbb)/(.\*) | /{3}/eee        | /bbb/eee       |


Here is a simple client example:
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

Here is a simple server example:
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


In the above examples, the Server starts a ROS2 node `example_ros2_server_node`, while the Client starts a ROS2 node `example_ros2_client_node`. The Client initiates RPC calls through the ROS2 backend, and the Server receives and processes the RPC requests via the ROS2 backend.

When the Client initiates a call to the Server, if the protocol layer uses the native ROS2 protocol, the communication will fully reuse ROS2's native protocol. Native ROS2 nodes can seamlessly interface with AimRT nodes based on this protocol.

If the Client initiates a call to the Server without using the ROS2 protocol, the communication will be wrapped based on the ROS2 protocol {{ '[RosRpcWrapper.srv]({}/src/protocols/plugins/ros2_plugin_proto/srv/RosRpcWrapper.srv)'.format(code_site_root_path_url) }}, whose content is as follows:
```
string  serialization_type
string[]  context
byte[]  data
---
int64   code
string  serialization_type
byte[]  data
```

If native ROS2 nodes need to interface with AimRT nodes in this case, developers of native ROS2 nodes must use the Req/Rsp `data` field of this protocol to serialize/deserialize the actual request/response packets.

Additionally, since ROS2 has specific requirements for `service_name`, the `service_name` for AimRT-ROS2 interoperability is generated by AimRT Func following a URL-like encoding rule:
- Encode the ASCII codes of all symbols except numbers, letters, and '/' in HEX format, prefixed with '_'.

For example, if the AimRT Func name is `/aaa.bbb.ccc/ddd`, the encoded ROS2 service name becomes `/aaa_2Ebbb_2Eccc/ddd`. The specific value will also be printed when the ros2_plugin starts.

Based on these features, the `ros2`-type RPC backend can be used to establish RPC links with native ROS2 nodes, enabling AimRT's compatibility with ROS2.## ros2 Type Channel Backend

The `ros2` type Channel backend is a Channel backend provided in the **ros2_plugin**, used to publish and subscribe AimRT Channel messages via ROS2 Topics. All its configurations are as follows:

| Node                                                | Type   | Optional | Default    | Description                                                                                                                                                                                              |
| --------------------------------------------------- | ------ | -------- | --------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| pub_topics_options                                  | array  | Optional     | []        | Rules for publishing Topics                                                                                                                                                                               |
| pub_topics_options[i].topic_name                    | string | Required     | ""        | Topic name, supports regular expressions                                                                                                                                                                        |
| pub_topics_options[i].qos                           | map    | Optional     | -         | QOS configuration                                                                                                                                                                                          |
| pub_topics_options[i].qos.history                   | string | Optional     | "default" | QOS history option<br/>keep_last: Keep the most recent records (cache up to N records, configurable via queue depth option)<br/>keep_all: Keep all records (cache all records, but limited by the maximum resources configurable by the underlying middleware)<br/>default: Use system default |
| pub_topics_options[i].qos.depth                     | int    | Optional     | 10        | QOS queue depth option (can only be used with Keep_last)                                                                                                                                                       |
| pub_topics_options[i].qos.reliability               | string | Optional     | "default" | QOS reliability option<br/>reliable: Reliable (if a message is lost, it will be resent, with repeated retransmissions to ensure successful data transfer)<br/>best_effort: Best effort (attempts to transfer data but does not guarantee successful transmission, data may be lost if the network is unstable)<br/>default: System default  |
| pub_topics_options[i].qos.durability                | string | Optional     | "default" | QOS durability option<br/>transient_local: Transient local (publisher retains data for late-joining subscribers)<br/>volatile: Volatile (no data is retained)<br/>default: System default                                              |
| pub_topics_options[i].qos.deadline                  | int    | Optional     | -1        | QOS expected maximum time interval (in milliseconds) between subsequent message publications to the topic<br/>Fill in the millisecond-level time interval, -1 means not set, follow system default                                                                                             |
| pub_topics_options[i].qos.lifespan                  | int    | Optional     | -1        | QOS maximum time interval (in milliseconds) between message publication and reception<br/>Without considering the message as stale or expired (expired messages are silently discarded and never actually received)<br/>Fill -1 to keep system default (not set)                                         |
| pub_topics_options[i].qos.liveliness                | string | Optional     | "default" | QOS option for determining how to check if the publisher is alive<br/>automatic: Automatic (ROS2 determines based on the time interval between message publication and reception)<br/>manual_by_topic: Requires the publisher to periodically declare<br/>default: Keep system default                                    |
| pub_topics_options[i].qos.liveliness_lease_duration | int    | Optional     | -1        | QOS liveliness lease duration (in milliseconds) option. If the publisher does not declare liveliness within this time, it is considered inactive<br/>Fill -1 to keep system default (not set)                                                                          |
| sub_topics_options                                  | array  | Optional     | []        | Rules for subscribing Topics                                                                                                                                                                               |
| sub_topics_options[i].topic_name                    | string | Required     | ""        | Topic name, supports regular expressions                                                                                                                                                                        || sub_topics_options[i].qos                           | map    | Optional | -         | QOS Configuration                                                                                                                                                                                          |
| sub_topics_options[i].qos.history                   | string | Optional | "default" | QOS history option<br/>keep_last: Keep recent records (cache up to N records, configurable via queue depth option)<br/>keep_all: Keep all records (cache all records, but limited by underlying middleware's maximum configurable resources)<br/>default: Use system default |
| sub_topics_options[i].qos.depth                     | int    | Optional | 10        | QOS queue depth option (only works with Keep_last)                                                                                                                                                       |
| sub_topics_options[i].qos.reliability               | string | Optional | "default" | QOS reliability option<br/>reliable: Reliable (retransmits lost messages to ensure successful delivery)<br/>best_effort: Best effort (attempts delivery but doesn't guarantee success, may lose data under unstable networks)<br/>default: System default  |
| sub_topics_options[i].qos.durability                | string | Optional | "default" | QOS durability option<br/>transient_local: Transient local (publisher retains data for late-joining subscribers)<br/>volatile: Volatile (does not retain any data)<br/>default: System default                                              |
| sub_topics_options[i].qos.deadline                  | int    | Optional | -1        | QOS maximum expected time between message publications on a topic (in milliseconds)<br/>Set to -1 for no setting (use system default)                                                                                             |
| sub_topics_options[i].qos.lifespan                  | int    | Optional | -1        | QOS maximum time between message publication and reception (in milliseconds) before considering it stale/expired (expired messages are silently discarded)<br/>Set to -1 for system default (no setting)                                         |
| sub_topics_options[i].qos.liveliness                | string | Optional | "default" | QOS option for determining publisher liveliness<br/>automatic: Automatic (ROS2 determines based on message publication/reception intervals)<br/>manual_by_topic: Requires periodic publisher declaration<br/>default: System default                                    |
| sub_topics_options[i].qos.liveliness_lease_duration | int    | Optional | -1        | QOS liveliness lease duration (in milliseconds). If publisher doesn't declare liveliness within this period, it's considered inactive.<br/>Set to -1 for system default (no setting)                                                                        |

Here is a simple publisher example:
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

Here is a simple subscriber example:
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
              qos:
                history: keep_last
                depth: 10
                reliability: reliable
                durability: volatile
                deadline: -1
                lifespan: -1
                liveliness: automatic
                liveliness_lease_duration: -1
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [ros2]
```

In the above examples, the publisher starts a ROS2 node `example_ros2_pub_node`, while the subscriber starts a ROS2 node `example_ros2_sub_node`. The publisher sends messages via ROS2 backend, and the subscriber receives and processes these messages through ROS2 backend.

If the message publishing/subscribing uses native ROS2 protocol, the communication will fully reuse ROS2's native protocol, allowing seamless integration between native ROS2 nodes and AimRT nodes.If the protocol layer does not use the ROS2 protocol when publishing/subscribing messages, communication will be based on the ROS2 protocol wrapped by {{ '[RosMsgWrapper.msg]({}/src/protocols/plugins/ros2_plugin_proto/msg/RosMsgWrapper.msg)'.format(code_site_root_path_url) }}, with the following content:
```
string  serialization_type
string[]  context
byte[]  data
```

If native ROS2 nodes need to interface with AimRT nodes in this case, developers of the native ROS2 nodes must use the `data` field of this protocol to serialize/deserialize the actual messages.

Additionally, the `Topic` name for communication between AimRT and ROS2 is generated by the following rule: `${aimrt_topic}/${ros2_encode_aimrt_msg_type}`. Here, `${aimrt_topic}` is the AimRT Topic name, and `${ros2_encode_aimrt_msg_type}` is generated from the AimRT Msg name using a URL-like encoding rule:
- Encode all non-alphanumeric characters (except '/') as HEX values of their ASCII codes, prefixed with '_'.

For example, if the AimRT Topic name is `test_topic` and the AimRT Msg name is `pb:aaa.bbb.ccc`, the resulting ROS2 Topic value will be `test_topic/pb_3Aaaa_2Ebbb_2Eccc`. The specific value will also be printed when the ros2_plugin starts.

Based on this feature, the `ros2`-type Channel backend can be used to establish a Channel link with native ROS2 nodes, enabling AimRT's compatibility with ROS2.

Developers can also refer to the example in {{ '[ros2_plugin]({}/src/examples/plugins/ros2_plugin)'.format(code_site_root_path_url) }} to communicate with native ROS2 Humble nodes.