

# ROS2 Plugin

## Related Links

Reference Example:
- {{ '[ros2_plugin]({}/src/examples/plugins/ros2_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**ros2_plugin** is a communication transport plugin implemented based on [ROS2 Humble](https://docs.ros.org/en/humble/index.html), providing the following components:
- `ros2` type RPC backend
- `ros2` type Channel backend

The plugin configuration items are as follows:

| Node                   | Type   | Optional | Default Value   | Description                                                                         |
| ---------------------- | ------ | -------- | --------------- | ----------------------------------------------------------------------------------- |
| node_name              | string | Mandatory | ""              | ROS2 node name                                                                      |
| executor_type          | string | Optional | "MultiThreaded" | ROS2 executor type, options: "SingleThreaded", "StaticSingleThreaded", "MultiThreaded" |
| executor_thread_num    | int    | Optional | 2               | When executor_type == "MultiThreaded", indicates the thread count of ROS2 executor |
| auto_initialize_logging | bool   | Optional | true            | Whether to initialize ROS2's default SPLOG logging system                          |

Regarding the configuration of **ros2_plugin**, please note:
- `node_name` represents the ROS2 node name. From external perspective, an AimRT node loaded with ROS2 plugin appears as a ROS2 node with the name configured by this item.
- `executor_type` specifies the type of ROS2 node executor, currently with three options: `SingleThreaded`, `StaticSingleThreaded`, `MultiThreaded`. For specific meanings, please refer to ROS2 Humble documentation.
- `executor_thread_num` only takes effect when `executor_type` is `MultiThreaded`, indicating the thread count of ROS2 executor.
- `auto_initialize_logging` determines whether to initialize ROS2's default SPLOG logging system. When set to `true`, it will use ROS2's default SPLOG logging system, with logs stored in the directory specified by environment variable ROS_LOG_DIR.

Additionally, when using **ros2_plugin**, the executor provided by **ros2_plugin** is used for Channel subscription callbacks, RPC Server processing, and RPC Client returns. If users block threads in callbacks, it may lead to exhaustion of **ros2_plugin**'s thread pool, preventing further message reception/transmission. As described in the Module interface documentation, generally:
- If tasks in callbacks are lightweight, they can be processed directly in the callback
- If tasks are heavy, it's better to schedule them to other dedicated executors for processing

Here's a simple example:
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

## ROS2 Type RPC Backend

The `ros2` type RPC backend is an RPC backend provided in **ros2_plugin** for handling AimRT RPC requests through ROS2 RPC. All its configuration items are as follows:

| Node                                              | Type   | Optional | Default   | Description                                                                                                                                                                                     |
| ------------------------------------------------- | ------ | -------- | --------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timeout_executor                                  | string | Optional | ""        | Executor for handling RPC timeout scenarios on the client side                                                                                                                                 |
| clients_options                                   | array  | Optional | []        | Rules for client-initiated RPC requests                                                                                                                                                         |
| clients_options[i].func_name                      | string | Required | ""        | RPC function name supporting regular expressions                                                                                                                                               |
| clients_options[i].qos                            | map    | Optional | -         | QoS configuration                                                                                                                                                                              |
| clients_options[i].qos.history                    | string | Optional | "default" | QoS history options<br/>keep_last: Retain recent records (caches up to N records via queue depth)<br/>keep_all: Retain all records (limited by middleware resources)<br/>default: Use system default |
| clients_options[i].qos.depth                      | int    | Optional | 10        | QoS queue depth (only works with Keep_last)                                                                                                                                                     |
| clients_options[i].qos.reliability                | string | Optional | "default" | QoS reliability options<br/>reliable: Guaranteed delivery (retransmits lost messages)<br/>best_effort: Best-effort delivery (no retransmissions)<br/>default: Use system default               |
| clients_options[i].qos.durability                 | string | Optional | "default" | QoS durability options<br/>transient_local: Retain data for late-joining subscribers<br/>volatile: No data retention<br/>default: Use system default                                           |
| clients_options[i].qos.deadline                   | int    | Optional | -1        | Maximum expected time between message publications (milliseconds)<br/>-1 means use system default                                                                                             |
| clients_options[i].qos.lifespan                   | int    | Optional | -1        | Maximum time between message publication and reception (milliseconds)<br/>Expired messages are silently discarded<br/>-1 means use system default                                              |
| clients_options[i].qos.liveliness                 | string | Optional | "default" | QoS liveliness determination<br/>automatic: Automatic detection by ROS2<br/>manual_by_topic: Requires periodic publisher declaration<br/>default: Use system default                           |
| clients_options[i].qos.liveliness_lease_duration  | int    | Optional | -1        | Liveliness lease duration (milliseconds)<br/>Publishers exceeding this duration are considered inactive<br/>-1 means use system default                                                        |
| clients_options[i].remapping_rule                 | string | Optional | ""        | Rule for remapping regex-matched func_name to new ros2_func_name<br/>Supports replacement patterns: {j} represents the j-th regex capture group                                              |
| servers_options                                  | array  | Optional | []        | Rules for the server to handle RPC requests                                                                                                                                                     |
| servers_options[i].func_name                     | string | Required | ""        | RPC Func name (supports regular expressions)                                                                                                                                                     |
| servers_options[i].qos                           | map    | Optional | -         | QOS configuration                                                                                                                                                                                |
| servers_options[i].qos.history                   | string | Optional | "default" | QOS history options<br/>keep_last: Keep recent records (caches up to N records, configurable via queue depth)<br/>keep_all: Keep all records (limited by underlying middleware)<br/>default: Use system default |
| servers_options[i].qos.depth                     | int    | Optional | 10        | QOS queue depth (only works with Keep_last)                                                                                                                                                      |
| servers_options[i].qos.reliability               | string | Optional | "default" | QOS reliability options<br/>reliable: Guaranteed delivery (retransmits on loss)<br/>best_effort: Best-effort delivery (may lose data)<br/>default: System default                                |
| servers_options[i].qos.durability                | string | Optional | "default" | QOS durability options<br/>transient_local: Keep data for late-joining subscribers<br/>volatile: No data retention<br/>default: System default                                                  |
| servers_options[i].qos.deadline                  | int    | Optional | -1        | Maximum expected time between messages (ms)<br/>-1 means use system default                                                                                                                      |
| servers_options[i].qos.lifespan                  | int    | Optional | -1        | Maximum time between publication and reception (ms)<br/>-1 means use system default                                                                                                             |
| servers_options[i].qos.liveliness                | string | Optional | "default" | Publisher liveliness detection<br/>automatic: Auto-detect by message intervals<br/>manual_by_topic: Requires periodic declaration<br/>default: System default                                    |
| servers_options[i].qos.liveliness_lease_duration | int    | Optional | -1        | Liveliness lease duration (ms)<br/>-1 means use system default                                                                                                                                   |
| servers_options[i].remapping_rule                | string | Optional | ""        | Rule to remap regex-matched func_name to new ros2_func_name<br/>Supports replacement patterns: {j} represents the j-th captured group                                                            |


Here's a simple example of remapping: For an AimRT func_name `pb:/aimrt_server/GetFooData`, without remapping, the generated ros func_name would be `/aimrt_5Fserver/GetFooData` (non-alphanumeric characters are HEX encoded with '_' prefix). With remapping configuration:



```yaml
rpc:
  backends:
    - type: ros2
      options:
        servers_options:
          - func_name: "(.*)/(.*)/(.*)" #这里是填写匹配 AimRT func_name 的正则表达式
            remapping_rule: "{1}/{2}" # 这里填写重映射规则，用于生成新的ros2_func_name。 这里也可以简化写成 /{2}

```

With this configuration, the first `(.*)` captures `pb:`, the second `(.*)` captures `aimrt_server`, and the third `(.*)` captures `GetFooData`. The resulting ros func_name is `/GetFooData`. To simplify the writing, the `remapping_rule` option can omit `{1}` which represents the message type, as the system will automatically generate it to adapt the conversion between AimRT func_name and ros2 func_name. Here are some quick examples to demonstrate more advanced usage, assuming the AimRT func_name is `pb:/aaa/bbb/ccc`:

| func_name               | remapping_rule  | ros2_func_name |
| ----------------------- | --------------- | -------------- |
| (.\*)/(.\*)/(.\*)/(.\*) |                 | /aaa/bbb/ccc   |
| (.\*)/(.\*)/(.\*)/(.\*) | {1}/{2}/ddd/{4} | /aaa/ddd/ccc   |
| (.\*)/(.\*)/(.\*)/(.\*) | /{2}/ddd/{4}    | /aaa/ddd/ccc   |
| (.\*)/(.\*)/(bbb)/(.\*) | {1}/{3}_{4}     | /bbb_5Fccc     |
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


In the above example, the server starts a ROS2 node `example_ros2_server_node`, while the client starts a ROS2 node `example_ros2_client_node`. The client initiates RPC calls through the ROS2 backend, and the server receives and processes RPC requests through the ROS2 backend.

When the client initiates a call to the server, if the protocol layer uses the native ROS2 protocol, the communication will fully reuse the native ROS2 protocol. Native ROS2 nodes can seamlessly interface with AimRT nodes based on this protocol.

If the client initiates a call to the server without using the ROS2 protocol, the communication will be wrapped using the {{ '[RosRpcWrapper.srv]({}/src/protocols/plugins/ros2_plugin_proto/srv/RosRpcWrapper.srv)'.format(code_site_root_path_url) }} ROS2 protocol, which has the following structure:


## ROS2 Type Channel Backend

The `ros2` type Channel backend is a Channel backend provided by **ros2_plugin**, which is used to publish and subscribe AimRT Channel messages through ROS2 Topics. All its configurations are as follows:

| Node                                                | Type   | Optional | Default Value | Description                                                                                                                                                                                      |
| --------------------------------------------------- | ------ | -------- | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| pub_topics_options                                  | array  | Yes      | []            | Rules for publishing Topics                                                                                                                                                                      |
| pub_topics_options[i].topic_name                    | string | Required | ""            | Topic name supporting regular expressions                                                                                                                                                        |
| pub_topics_options[i].qos                           | map    | Yes      | -             | QOS configuration                                                                                                                                                                                |
| pub_topics_options[i].qos.history                   | string | Yes      | "keep_last"   | QOS history policy<br/>keep_last: Retain recent records (cache up to N records via queue depth)<br/>keep_all: Retain all records (limited by middleware resources)<br/>default: Use system default |
| pub_topics_options[i].qos.depth                     | int    | Yes      | 1             | QOS queue depth option (only works with keep_last)                                                                                                                                              |
| pub_topics_options[i].qos.reliability               | string | Yes      | "best_effort" | QOS reliability policy<br/>reliable: Guaranteed delivery<br/>best_effort: Best-effort delivery<br/>default: System default                                                                        |
| pub_topics_options[i].qos.durability                | string | Yes      | "volatile"    | QOS durability policy<br/>transient_local: Retain data for late-joining subscribers<br/>volatile: No data retention<br/>default: System default                                                 |
| pub_topics_options[i].qos.deadline                  | int    | Yes      | -1            | Maximum expected time interval (milliseconds) between messages<br/>-1 means use system default                                                                                                    |
| pub_topics_options[i].qos.lifespan                  | int    | Yes      | -1            | Maximum time (milliseconds) between message publication and reception<br/>-1 means use system default                                                                                            |
| pub_topics_options[i].qos.liveliness                | string | Yes      | "default"     | QOS liveliness policy<br/>automatic: Auto-detect publisher status<br/>manual_by_topic: Require periodic declaration<br/>default: System default                                                  |
| pub_topics_options[i].qos.liveliness_lease_duration | int    | Yes      | -1            | Liveliness lease duration (milliseconds)<br/>-1 means use system default                                                                                                                        |
| sub_topics_options                                  | array  | Yes      | []            | Rules for subscribing Topics                                                                                                                                                                     |
| sub_topics_options[i].topic_name                    | string | Required | ""            | Topic name supporting regular expressions                                                                                                                                                        |

| sub_topics_options[i].qos                           | map    | Optional | -             | QoS Configuration                                                                                                                                                                                |
|------------------------------------------------------|--------|----------|---------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| sub_topics_options[i].qos.history                   | string | Optional | "keep_last"   | QoS history options<br/>keep_last: Keep recent records (cache up to N records, configurable via queue depth)<br/>keep_all: Keep all records (subject to middleware limits)<br/>default: System default |
| sub_topics_options[i].qos.depth                     | int    | Optional | 1             | QoS queue depth (only works with Keep_last)                                                                                                                                                      |
| sub_topics_options[i].qos.reliability               | string | Optional | "best_effort" | QoS reliability options<br/>reliable: Guaranteed delivery (resend lost messages)<br/>best_effort: Best effort delivery (no resending)<br/>default: System default                                |
| sub_topics_options[i].qos.durability                | string | Optional | "volatile"    | QoS durability options<br/>transient_local: Retain data for late-joining subscribers<br/>volatile: No data retention<br/>default: System default                                                 |
| sub_topics_options[i].qos.deadline                  | int    | Optional | -1            | Maximum expected time between message publications (milliseconds)<br/>-1 means system default                                                                                                   |
| sub_topics_options[i].qos.lifespan                  | int    | Optional | -1            | Maximum time between message publication and reception (milliseconds)<br/>Expired messages are silently dropped<br/>-1 means system default                                                      |
| sub_topics_options[i].qos.liveliness                | string | Optional | "default"     | Publisher liveliness detection<br/>automatic: Automatic detection<br/>manual_by_topic: Requires periodic declaration<br/>default: System default                                               |
| sub_topics_options[i].qos.liveliness_lease_duration | int    | Optional | -1            | Liveliness lease duration (milliseconds)<br/>Publisher considered inactive if no declaration within this period<br/>-1 means system default                                                      |

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

In these examples:
- The publisher starts a ROS2 node named `example_ros2_pub_node`
- The subscriber starts a ROS2 node named `example_ros2_sub_node`
- Messages are published via ROS2 backend and processed by the subscriber

When using native ROS2 protocol for message pub/sub:
- Full reuse of ROS2 native protocol
- Native ROS2 nodes can seamlessly communicate with AimRT nodes through this protocol

When the message publishing/subscribing does not use the ROS2 protocol at the protocol layer, communication will be wrapped based on the {{ '[RosMsgWrapper.msg]({}/src/protocols/plugins/ros2_plugin_proto/msg/RosMsgWrapper.msg)'.format(code_site_root_path_url) }} ROS2 protocol. The protocol content is as follows:

```
string  serialization_type
string[]  context
byte[]  data
```

When native ROS2 nodes need to interface with AimRT nodes under this scenario, developers of native ROS2 nodes must serialize/deserialize actual messages using the data field of this protocol.

Additionally, the `Topic` name for AimRT-ROS2 interoperability follows this generation rule: `${aimrt_topic}/${ros2_encode_aimrt_msg_type}`. Where:
- `${aimrt_topic}` is the AimRT Topic name
- `${ros2_encode_aimrt_msg_type}` is generated from the AimRT Msg name using URL-like encoding rules:
  - All non-alphanumeric characters and '/' are converted to HEX ASCII codes prefixed with '_'

For example:
- If AimRT Topic is `test_topic` and AimRT Msg is `pb:aaa.bbb.ccc`
- The final ROS2 Topic becomes `test_topic/pb_3Aaaa_2Ebbb_2Eccc`
- Specific values will also be printed when ros2_plugin starts

Based on this feature, the `ros2`-type Channel backend can be used to establish Channel links with native ROS2 nodes, achieving ROS2 compatibility for AimRT.

Developers can also refer to {{ '[ros2_plugin]({}/src/examples/plugins/ros2_plugin)'.format(code_site_root_path_url) }} examples to communicate with native ros2 humble nodes.