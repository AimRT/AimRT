# proxy插件

## 相关链接

参考示例：
- {{ '[proxy_plugin]({}/src/examples/plugins/proxy_plugin)'.format(code_site_root_path_url) }}

## 插件概述

**proxy_plugin**用于对 Channel 中的消息进行代理转发，插件支持独立的 type_support_pkg，并支持指定执行器, 其中执行器需要线程安全，在使用时，插件会根据配置注册一个或多个 Channel Subscriber 或 Publisher。

插件的配置项如下：

| 节点                              | 类型          | 是否可选| 默认值  | 作用 |
| ----                              | ----          | ----  | ----      | ---- |
| type_support_pkgs                 | array         | 必选  | []        | type support 包配置 |
| type_support_pkgs[i].path         | string        | 必选  | ""        | type support 包的路径 |
| proxy_actions                     | array         | 必选  | []        | 代理转发配置 |
| proxy_actions[i].name            | string        | 必选  | ""        | 代理转发名称 |
| proxy_actions[i].options         | object        | 必选  | {}        | 代理转发配置 |
| proxy_actions[i].options.executor| string        | 必选  | ""        | 代理转发执行器 |
| proxy_actions[i].options.topic_meta_list                   | array         | 必选  | []        | 要代理转发的 topic 和类型 |
| proxy_actions[i].options.topic_meta_list[j].topic_name     | string        | 必选  | ""        | 要代理转发的 topic |
| proxy_actions[i].options.topic_meta_list[j].msg_type       | string        | 必选  | ""        | 要代理转发的消息类型 |
| proxy_actions[i].options.topic_meta_list[j].pub_topic_name | array         | 必选  | []        | 代理转发后的 topic |


请注意，**proxy_plugin**中是以`action`为单元管理代理转发动作的，每个代理转发`action`可以有自己的执行器、topic 等参数，使用时可以根据数据实际大小和频率，为每个 action 分配合理的资源。


### 代理转发的简单示例配置

以下是将一个以 http 为后端的 topic 消息代理转发到两个以 zenoh 和 ros2 为后端的 topic 的简单示例配置，对于 proxy_plugin 需要为每个 action 指定执行器，并且在 channel 处需要为每个订阅的 topic 和转发的 topic 指定后端，其他相关插件的配置请参考[net_plugin](./net_plugin.md), [zenoh_plugin](./zenoh_plugin.md) 和 [ros2_plugin](./ros2_plugin.md);

```yaml
aimrt:
  plugin:
    plugins:
      - name: proxy_plugin
        path: ./libaimrt_proxy_plugin.so
        options:
          type_support_pkgs:
            - path: ./libexample_pb_ts.so
          proxy_actions:
            - name: my_proxy
              options:
                executor: proxy_plugin_executor
                topic_meta_list:
                  - sub_topic_name: test_topic_http
                    pub_topic_name: [test_topic_zenoh, test_topic_ros2]
                    msg_type: pb:aimrt.protocols.example.ExampleEventMsg
      - name: zenoh_plugin
        path: ./libaimrt_zenoh_plugin.so
      - name: ros2_plugin
        path: ./libaimrt_ros2_plugin.so
        options:
          node_name: example_ros2_pb_chn_publisher_node
          executor_type: MultiThreaded # SingleThreaded/StaticSingleThreaded/MultiThreaded/EventsExecutor
          executor_thread_num: 2
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
      - type: zenoh
      - type: ros2
    sub_topics_options:
      - topic_name: test_topic_http
        enable_backends: [http]
    pub_topics_options:
      - topic_name: test_topic_zenoh
        enable_backends: [zenoh]
      - topic_name: test_topic_ros2
        enable_backends: [ros2]
    # ...
```
