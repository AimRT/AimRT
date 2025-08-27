# ROS2 插件

## 相关链接

参考示例：
- {{ '[ros2_plugin]({}/src/examples/plugins/ros2_plugin)'.format(code_site_root_path_url) }}


## 插件概述


**ros2_plugin**是一个基于 [ROS2 Humble](https://docs.ros.org/en/humble/index.html) 实现的通信传输插件，此插件提供了以下组件：
- `ros2`类型 RPC 后端
- `ros2`类型 Channel 后端

插件的配置项如下：

| 节点                    | 类型   | 是否可选 | 默认值          | 作用                                                                               |
| ----------------------- | ------ | -------- | --------------- | ---------------------------------------------------------------------------------- |
| node_name               | string | 必选     | ""              | ROS2 节点名称                                                                      |
| executor_type           | string | 可选     | "MultiThreaded" | ROS2 执行器类型，可选值："SingleThreaded"、"StaticSingleThreaded"、"MultiThreaded" |
| executor_thread_num     | int    | 可选     | 2               | 当 executor_type == "MultiThreaded" 时，表示 ROS2 执行器的线程数                   |
| auto_initialize_logging | bool   | 可选     | true            | 是否初始化 ROS2 默认的 SPLOG 日志系统                                              |

关于**ros2_plugin**的配置，使用注意点如下：
- `node_name`表示 ROS2 节点名称，在外界看来，加载了 ROS2 插件的 AimRT 节点就是一个 ROS2 节点，它的 node 名称就是根据此项来配置。
- `executor_type`表示 ROS2 节点执行器的类型，当前有三种选择：`SingleThreaded`、`StaticSingleThreaded`、`MultiThreaded`，具体的含义请参考 ROS2 Humble 的文档。
- `executor_thread_num`仅在`executor_type`值为`MultiThreaded`时生效，表示 ROS2 的线程数。
- `auto_initialize_logging`表示是否初始化 ROS2 默认的 SPLOG 日志系统，如果设置为`true`，则会使用 ROS2 默认的 SPLOG 日志系统， 相关日志会存放在环境变量 ROS_LOG_DIR 所决定的目录下。


此外，在使用**ros2_plugin**时，Channel 订阅回调、RPC Server 处理、RPC Client 返回时，使用的都是**ros2_plugin**提供的执行器，当使用者在回调中阻塞了线程时，有可能导致**ros2_plugin**线程池耗尽，从而无法继续接收/发送消息。正如 Module 接口文档中所述，一般来说，如果回调中的任务非常轻量，那就可以直接在回调里处理；但如果回调中的任务比较重，那最好调度到其他专门执行任务的执行器里处理。


以下是一个简单的示例：
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


## ros2 类型 RPC 后端


`ros2`类型的 RPC 后端是**ros2_plugin**中提供的一种 RPC 后端，用于通过 ROS2 RPC 的方式来调用和处理 AimRT RPC 请求。其所有的配置项如下：

| 节点                                             | 类型   | 是否可选 | 默认值    | 作用                                                                                                                                                                                              |
| ------------------------------------------------ | ------ | -------- | --------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timeout_executor                                 | string | 可选     | ""        | Client 端 RPC 超时情况下的执行器                                                                                                                                                                  |
| clients_options                                  | array  | 可选     | []        | 客户端发起 RPC 请求时的规则                                                                                                                                                                       |
| clients_options[i].func_name                     | string | 必选     | ""        | RPC Func 名称，支持正则表达式                                                                                                                                                                     |
| clients_options[i].qos                           | map    | 可选     | -         | QOS 配置                                                                                                                                                                                          |
| clients_options[i].qos.history                   | string | 可选     | "default" | QOS 的历史记录选项<br/>keep_last:保留最近的记录(缓存最多 N 条记录，可通过队列长度选项来配置)<br/>keep_all:保留所有记录(缓存所有记录，但受限于底层中间件可配置的最大资源)<br/>default:使用系统默认 |
| clients_options[i].qos.depth                     | int    | 可选     | 10        | QOS 的队列深度选项(只能与 Keep_last 配合使用)                                                                                                                                                     |
| clients_options[i].qos.reliability               | string | 可选     | "default" | QOS 的可靠性选项<br/>reliable:可靠的(消息丢失时，会重新发送,反复重传以保证数据传输成功)<br/>best_effort:尽力而为的(尝试传输数据但不保证成功传输,当网络不稳定时可能丢失数据)<br/>default:系统默认  |
| clients_options[i].qos.durability                | string | 可选     | "default" | QOS 的持续性选项<br/>transient_local:局部瞬态(发布器为晚连接(late-joining)的订阅器保留数据)<br/>volatile:易变态(不保留任何数据)<br/>default:系统默认                                              |
| clients_options[i].qos.deadline                  | int    | 可选     | -1        | QOS 的后续消息发布到主题之间的预期最大时间量选项<br/>需填毫秒级时间间隔，填 -1 为不设置，按照系统默认                                                                                             |
| clients_options[i].qos.lifespan                  | int    | 可选     | -1        | QOS 的消息发布和接收之间的最大时间量(单位毫秒)选项<br/>而不将消息视为陈旧或过期（过期的消息被静默地丢弃，并且实际上从未被接收<br/>填-1保持系统默认 不设置                                         |
| clients_options[i].qos.liveliness                | string | 可选     | "default" | QOS 的如何确定发布者是否活跃选项<br/>automatic:自动(ROS2 会根据消息发布和接收的时间间隔来判断)<br/>manual_by_topic:需要发布者定期声明<br/>default:保持系统默认                                    |
| clients_options[i].qos.liveliness_lease_duration | int    | 可选     | -1        | QOS 的活跃性租期的时长(单位毫秒)选项，如果超过这个时间发布者没有声明活跃，则被认为是不活跃的<br/>填-1保持系统默认 不设置                                                                          |
| clients_options[i].remapping_rule                | string | 可选     | ""        | 用于将 clients_options[i].func_name 所正则匹配到的 func_name 按照新则规则进行重映射， 以生成新的 ros2_func_name <br/>在书写的时候支持替换规则: {j} 表示第 j 个被正则匹配的捕获组                  |
| servers_options                                  | array  | 可选     | []        | 服务端接收处理 RPC 请求时的规则                                                                                                                                                                   |
| servers_options[i].func_name                     | string | 必选     | ""        | RPC Func 名称，支持正则表达式                                                                                                                                                                     |
| servers_options[i].qos                           | map    | 可选     | -         | QOS 配置                                                                                                                                                                                          |
| servers_options[i].qos.history                   | string | 可选     | "default" | QOS 的历史记录选项<br/>keep_last:保留最近的记录(缓存最多 N 条记录，可通过队列长度选项来配置)<br/>keep_all:保留所有记录(缓存所有记录，但受限于底层中间件可配置的最大资源)<br/>default:使用系统默认 |
| servers_options[i].qos.depth                     | int    | 可选     | 10        | QOS 的队列深度选项(只能与 Keep_last 配合使用)                                                                                                                                                     |
| servers_options[i].qos.reliability               | string | 可选     | "default" | QOS 的可靠性选项<br/>reliable:可靠的(消息丢失时，会重新发送,反复重传以保证数据传输成功)<br/>best_effort:尽力而为的(尝试传输数据但不保证成功传输,当网络不稳定时可能丢失数据)<br/>default:系统默认  |
| servers_options[i].qos.durability                | string | 可选     | "default" | QOS 的持续性选项<br/>transient_local:局部瞬态(发布器为晚连接(late-joining)的订阅器保留数据)<br/>volatile:易变态(不保留任何数据)<br/>default:系统默认                                              |
| servers_options[i].qos.deadline                  | int    | 可选     | -1        | QOS 的后续消息发布到主题之间的预期最大时间量选项<br/>需填毫秒级时间间隔，填 -1 为不设置，按照系统默认                                                                                             |
| servers_options[i].qos.lifespan                  | int    | 可选     | -1        | QOS 的消息发布和接收之间的最大时间量(单位毫秒)选项<br/>而不将消息视为陈旧或过期（过期的消息被静默地丢弃，并且实际上从未被接收<br/>填-1保持系统默认 不设置                                         |
| servers_options[i].qos.liveliness                | string | 可选     | "default" | QOS 的如何确定发布者是否活跃选项<br/>automatic:自动(ROS2 会根据消息发布和接收的时间间隔来判断)<br/>manual_by_topic:需要发布者定期声明<br/>default:保持系统默认                                    |
| servers_options[i].qos.liveliness_lease_duration | int    | 可选     | -1        | QOS 的活跃性租期的时长(单位毫秒)选项，如果超过这个时间发布者没有声明活跃，则被认为是不活跃的<br/>填 -1 保持系统默认 不设置                                                                        |
| servers_options[i].remapping_rule                | string | 可选     | ""        | 用于将 servers_options[i].func_name 所正则匹配到的 func_name 按照新则规则进行重映射， 以生成新的 ros2_func_name <br/>在书写的时候支持替换规则: {j} 表示第 j 个被正则匹配的捕获组                  |


下面是一个 remap 用法的简单示例：名为 `pb:/aimrt_server/GetFooData` 的 AimRT func_name ，若不需要 remap，则 最终生成的 ros func_name 就是 `/aimrt_5Fserver/GetFooData` （可以看到把 ":" 及之前的 <msg_type> 去掉并且将非数字、字母和'/'的符号的 ascii 码以 HEX 编码，加上 '_' 作为前缀）。 若需要 remap，则可以配置如下：

```yaml
rpc:
  backends:
    - type: ros2
      options:
        servers_options:
          - func_name: "(.*)/(.*)/(.*)" #这里是填写匹配 AimRT func_name 的正则表达式
            remapping_rule: "{1}/{2}" # 这里填写重映射规则，用于生成新的ros2_func_name。 这里也可以简化写成 /{2}

```
经过该配置，第一个`(.*)`捕获到了 `pb:`， 第二个`(.*)`捕获到了 `aimrt_server`， 第三个`(.*)`捕获到了 `GetFooData`， 最后生成的 ros func_name 就是 `/GetFooData` , 为了简化书写，在选项中的`remapping_rule`可以不填写`{1}`所代表的消息类型，系统会自动生成以适配AimRT func_name 和 ros2 func_name 的转换关系。以下是一些快速的例子，用于演示更丰富的用法，假设 AimRT func_name 为 `pb:/aaa/bbb/ccc` ：

| func_name               | remapping_rule  | ros2_func_name |
| ----------------------- | --------------- | -------------- |
| (.\*)/(.\*)/(.\*)/(.\*) |                 | /aaa/bbb/ccc   |
| (.\*)/(.\*)/(.\*)/(.\*) | {1}/{2}/ddd/{4} | /aaa/ddd/ccc   |
| (.\*)/(.\*)/(.\*)/(.\*) | /{2}/ddd/{4}    | /aaa/ddd/ccc   |
| (.\*)/(.\*)/(bbb)/(.\*) | {1}/{3}_{4}     | /bbb_5Fccc     |
| (.\*)/(.\*)/(bbb)/(.\*) | /{3}/eee        | /bbb/eee       |


以下是一个简单的客户端的示例：
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

以下则是一个简单的服务端的示例：
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


以上示例中，Server 端启动了一个 ROS2 节点`example_ros2_server_node`，Client 端则启动了一个 ROS2 节点`example_ros2_client_node`，Client 端通过 ROS2 的后端发起 RPC 调用，Server 端通过 ROS2 后端接收到 RPC 请求并进行处理。

Client 端向 Server 端发起调用时，如果协议层是原生 ROS2 协议，那么通信时将完全复用 ROS2 的原生协议，原生 ROS2 节点可以基于该协议无缝与 AimRT 节点对接。

如果 Client 端向 Server 端发起调用时，协议层没有使用 ROS2 协议，那么通信时将基于{{ '[RosRpcWrapper.srv]({}/src/protocols/plugins/ros2_plugin_proto/srv/RosRpcWrapper.srv)'.format(code_site_root_path_url) }}这个 ROS2 协议进行包装，该协议内容如下：
```
string  serialization_type
string[]  context
byte[]  data
---
int64   code
string  serialization_type
byte[]  data
```

如果此时原生的 ROS2 节点需要和 AimRT 的节点对接，原生 ROS2 节点的开发者需要搭配此协议 Req/Rsp 的 data 字段来序列化/反序列化真正的请求包/回包。

此外，由于 ROS2 对`service_name`有一些特殊要求，AimRT 与ROS2 互通时的`service_name`由 AimRT Func 根据一个类似于 URL 编码的规则来生成：
- 将所有除数字、字母和'/'的符号的 ascii 码以 HEX 编码，加上 '_' 作为前缀。

例如，如果 AimRT Func 名称为`/aaa.bbb.ccc/ddd`，则编码后的 ROS2 service name 就是`/aaa_2Ebbb_2Eccc/ddd`。具体值也会在 ros2_plugin 启动时打印出来。

基于以上特性，`ros2`类型的 RPC 后端可以用于打通与原生 ROS2 节点的 RPC 链路，从而实现 AimRT 对 ROS2 的兼容。


## ros2 类型 Channel 后端

`ros2`类型的 Channel 后端是**ros2_plugin**中提供的一种 Channel 后端，用于通过 ROS2 Topic 的方式来发布和订阅 AimRT Channel 消息。其所有的配置如下:

| 节点                                                | 类型   | 是否可选 | 默认值    | 作用                                                                                                                                                                                              |
| --------------------------------------------------- | ------ | -------- | --------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| pub_topics_options                                  | array  | 可选     | []        | 发布 Topic 时的规则                                                                                                                                                                               |
| pub_topics_options[i].topic_name                    | string | 必选     | ""        | Topic 名称，支持正则表达式                                                                                                                                                                        |
| pub_topics_options[i].qos                           | map    | 可选     | -         | QOS 配置                                                                                                                                                                                          |
| pub_topics_options[i].qos.history                   | string | 可选     | "default" | QOS 的历史记录选项<br/>keep_last:保留最近的记录(缓存最多 N 条记录，可通过队列长度选项来配置)<br/>keep_all:保留所有记录(缓存所有记录，但受限于底层中间件可配置的最大资源)<br/>default:使用系统默认 |
| pub_topics_options[i].qos.depth                     | int    | 可选     | 10        | QOS 的队列深度选项(只能与Keep_last配合使用)                                                                                                                                                       |
| pub_topics_options[i].qos.reliability               | string | 可选     | "default" | QOS 的可靠性选项<br/>reliable:可靠的(消息丢失时，会重新发送,反复重传以保证数据传输成功)<br/>best_effort:尽力而为的(尝试传输数据但不保证成功传输,当网络不稳定时可能丢失数据)<br/>default:系统默认  |
| pub_topics_options[i].qos.durability                | string | 可选     | "default" | QOS 的持续性选项<br/>transient_local:局部瞬态(发布器为晚连接(late-joining)的订阅器保留数据)<br/>volatile:易变态(不保留任何数据)<br/>default:系统默认                                              |
| pub_topics_options[i].qos.deadline                  | int    | 可选     | -1        | QOS 的后续消息发布到主题之间的预期最大时间量选项<br/>需填毫秒级时间间隔，填 -1 为不设置，按照系统默认                                                                                             |
| pub_topics_options[i].qos.lifespan                  | int    | 可选     | -1        | QOS 的消息发布和接收之间的最大时间量(单位毫秒)选项<br/>而不将消息视为陈旧或过期（过期的消息被静默地丢弃，并且实际上从未被接收<br/>填-1保持系统默认 不设置                                         |
| pub_topics_options[i].qos.liveliness                | string | 可选     | "default" | QOS 的如何确定发布者是否活跃选项<br/>automatic:自动(ROS2 会根据消息发布和接收的时间间隔来判断)<br/>manual_by_topic:需要发布者定期声明<br/>default:保持系统默认                                    |
| pub_topics_options[i].qos.liveliness_lease_duration | int    | 可选     | -1        | QOS 的活跃性租期的时长(单位毫秒)选项，如果超过这个时间发布者没有声明活跃，则被认为是不活跃的<br/>填-1保持系统默认 不设置                                                                          |
| sub_topics_options                                  | array  | 可选     | []        | 订阅 Topic 时的规则                                                                                                                                                                               |
| sub_topics_options[i].topic_name                    | string | 必选     | ""        | Topic 名称，支持正则表达式                                                                                                                                                                        |
| sub_topics_options[i].use_serialized                | bool   | 可选     | "false"   | 控制订阅回调拿到的数据形态，只针对ros2msg有效                                                                                                                                               |
| sub_topics_options[i].qos                           | map    | 可选     | -         | QOS 配置                                                                                                                                                                                          |
| sub_topics_options[i].qos.history                   | string | 可选     | "default" | QOS 的历史记录选项<br/>keep_last:保留最近的记录(缓存最多 N 条记录，可通过队列长度选项来配置)<br/>keep_all:保留所有记录(缓存所有记录，但受限于底层中间件可配置的最大资源)<br/>default:使用系统默认 |
| sub_topics_options[i].qos.depth                     | int    | 可选     | 10        | QOS 的队列深度选项(只能与Keep_last配合使用)                                                                                                                                                       |
| sub_topics_options[i].qos.reliability               | string | 可选     | "default" | QOS 的可靠性选项<br/>reliable:可靠的(消息丢失时，会重新发送,反复重传以保证数据传输成功)<br/>best_effort:尽力而为的(尝试传输数据但不保证成功传输,当网络不稳定时可能丢失数据)<br/>default:系统默认  |
| sub_topics_options[i].qos.durability                | string | 可选     | "default" | QOS 的持续性选项<br/>transient_local:局部瞬态(发布器为晚连接(late-joining)的订阅器保留数据)<br/>volatile:易变态(不保留任何数据)<br/>default:系统默认                                              |
| sub_topics_options[i].qos.deadline                  | int    | 可选     | -1        | QOS 的后续消息发布到主题之间的预期最大时间量选项<br/>需填毫秒级时间间隔，填 -1 为不设置，按照系统默认                                                                                             |
| sub_topics_options[i].qos.lifespan                  | int    | 可选     | -1        | QOS 的消息发布和接收之间的最大时间量(单位毫秒)选项<br/>而不将消息视为陈旧或过期（过期的消息被静默地丢弃，并且实际上从未被接收<br/>填-1保持系统默认 不设置                                         |
| sub_topics_options[i].qos.liveliness                | string | 可选     | "default" | QOS 的如何确定发布者是否活跃选项<br/>automatic:自动(ROS2 会根据消息发布和接收的时间间隔来判断)<br/>manual_by_topic:需要发布者定期声明<br/>default:保持系统默认                                    |
| sub_topics_options[i].qos.liveliness_lease_duration | int    | 可选     | -1        | QOS 的活跃性租期的时长(单位毫秒)选项，如果超过这个时间发布者没有声明活跃，则被认为是不活跃的<br/>填 -1 保持系统默认 不设置                                                                        |


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
                durability: volatile
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

如果此时原生的 ROS2 节点需要和 AimRT 的节点对接，原生 ROS2 节点的开发者需要搭配此协议的 data 字段来序列化/反序列化真正的消息。

此外，AimRT 与 ROS2 互通时的`Topic`名称由以下规则生成：`${aimrt_topic}/${ros2_encode_aimrt_msg_type}`。其中`${aimrt_topic}`是 AimRT Topic 名称，`${ros2_encode_aimrt_msg_type}`根据 AimRT Msg 名称由一个类似于 URL 编码的规则来生成：
- 将所有除数字、字母和'/'的符号的 ascii 码以 HEX 编码，加上 '_' 作为前缀。

例如，如果 AimRT Topic 名称是`test_topic`，AimRT Msg 名称为`pb:aaa.bbb.ccc`，则最终 ROS2 Topic 值就是`test_topic/pb_3Aaaa_2Ebbb_2Eccc`。具体值也会在 ros2_plugin 启动时打印出来。

基于这个特性，`ros2`类型的 Channel 后端可以用于打通与原生 ROS2 节点的 Channel 链路，从而实现 AimRT 对 ROS2 的兼容。


开发者还可以参考{{ '[ros2_plugin]({}/src/examples/plugins/ros2_plugin)'.format(code_site_root_path_url) }}中的示例，与原生 ros2 humble 节点进行通信。
