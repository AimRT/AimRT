# echo插件

## 相关链接

参考示例：
- {{ '[echo_plugin]({}/src/examples/plugins/echo_plugin)'.format(code_site_root_path_url) }}

## 插件概述

**echo_plugin**用于对 Channel 中的消息进行回显，插件支持独立的 type_support_pkg，并支持指定执行器, 必须设定 log_lvl 为 Trace，Debug，Info 之一才能正常工作。

插件的配置项如下：

| 节点                              | 类型          | 是否可选| 默认值  | 作用 |
| ----                              | ----          | ----  | ----      | ---- |
| type_support_pkgs                 | array         | 必选  | []        | type support 包配置 |
| type_support_pkgs[i].path         | string        | 必选  | ""        | type support 包的路径 |
| topic_meta_list                   | array         | 必选  | []        | 要回显的 topic 和类型 |
| topic_meta_list[j].topic_name     | string        | 必选  | ""        | 要回显的 topic |
| topic_meta_list[j].msg_type       | string        | 必选  | ""        | 要回显的消息类型 |
| topic_meta_list[j].echo_type      | string        | 可选  | "json"    | 回显消息的格式，ros2 支持 "json", "yaml" ， pb 只支持 "json" |



### 回显消息的简单示例配置

对于回显消息的格式，ros2 消息类型 支持 "json", "yaml" ， pb只支持 "json"

以下是一个 pb 消息类型回显消息格式为 json 的简单示例配置：
```yaml
aimrt:
  plugin:
    plugins:
      - name: echo_plugin
        path: ./libaimrt_echo_plugin.so
        options:
          type_support_pkgs:
            - path: ./libexample_event_ts_pkg.so
          topic_meta_list:
            - topic_name: test_topic
              msg_type: pb:aimrt.protocols.example.ExampleEventMsg
              echo_type: json
  log:
    core_lvl: Info # Trace/Debug/Info
    backends:
      - type: console
  channel:
    # ...
```


以下是一个 ros2 消息类型回显消息格式为 yaml 的简单示例配置：
```yaml
aimrt:
  plugin:
    plugins:
      - name: echo_plugin
        path: ./libaimrt_echo_plugin.so
        options:
          type_support_pkgs:
            - path: ./libexample_event_ts_pkg.so
          topic_meta_list:
            - topic_name: test_topic
              msg_type: ros2:example_ros2/msg/RosTestMsg
              echo_type: yaml
  log:
    core_lvl: Info # Trace/Debug/Info
    backends:
      - type: console
  channel:
    # ...
```

