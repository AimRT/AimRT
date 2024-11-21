
# echo插件

## 相关链接

参考示例：
- {{ '[proxy_plugin]({}/src/examples/plugins/proxy_plugin)'.format(code_site_root_path_url) }}

## 插件概述

**proxy_plugin**用于对 Channel 中的消息进行代理转发，插件支持独立的 type_support_pkg，并支持指定执行器, 其中执行器需要线程安全。

插件的配置项如下：

| 节点                              | 类型          | 是否可选| 默认值  | 作用 |
| ----                              | ----          | ----  | ----      | ---- |
| type_support_pkgs                 | array         | 必选  | []        | type support 包配置 |
| type_support_pkgs[i].path         | string        | 必选  | ""        | type support 包的路径 |
| proxy_actions                     | array         | 必选  | []        | 代理转发配置 |
| proxy_actions[j].name            | string        | 必选  | ""        | 代理转发名称 |
| proxy_actions[j].options         | object        | 必选  | {}        | 代理转发配置 |
| proxy_actions[j].options.executor| string        | 必选  | ""        | 代理转发执行器 |
| proxy_actions[j].options.topic_meta_list                   | array         | 必选  | []        | 要代理转发的 topic 和类型 |
| proxy_actions[j].options.topic_meta_list[k].topic_name     | string        | 必选  | ""        | 要代理转发的 topic |
| proxy_actions[j].options.topic_meta_list[k].msg_type       | string        | 必选  | ""        | 要代理转发的消息类型 |
| proxy_actions[j].options.topic_meta_list[k].pub_topic_name | array         | 必选  | []        | 代理转发后的 topic |



### 代理转发的简单示例配置

以下是将一个 topic 消息代理转发到两个 topic 的简单示例配置：
```yaml
aimrt:
  plugin:
      plugins:
        - name: proxy_plugin
          path: ./libaimrt_proxy_plugin.so
          options:
            type_support_pkgs:
              - path: ./libexample_event_ts_pkg.so
            proxy_actions:
              - name: my_proxy
                options:
                  executor: proxy_plugin_executor
                  topic_meta_list:
                    - sub_topic_name: test_topic
                      pub_topic_name: [test_topic_zenoh, test_topic_ros2]
                      msg_type: pb:aimrt.protocols.example.ExampleEventMsg
        - name: zenoh_plugin
          path: ./libaimrt_zenoh_plugin.so
        - name: ros2_plugin
          path: ./libaimrt_ros2_plugin.so
          options:
            node_name: example_ros2_pb_chn_publisher_node
            executor_type: MultiThreaded # SingleThreaded/StaticSingleThreaded/MultiThreaded
            executor_thread_num: 2
  log:
    core_lvl: Info # Trace/Debug/Info
    backends:
      - type: console
  channel:
    # ...
```



