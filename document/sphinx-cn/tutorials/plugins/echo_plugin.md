
# echo插件

## 相关链接

参考示例：
- {{ '[echo_plugin]({}/src/examples/plugins/echo_plugin)'.format(code_site_root_path_url) }}

## 插件概述

**echo_plugin**用于对 Channel 中的消息进行回显，插件支持独立的 type_support_pkg，并支持指定执行器。

插件的配置项如下：

| 节点                              | 类型          | 是否可选| 默认值  | 作用 |
| ----                              | ----          | ----  | ----      | ---- |
| type_support_pkgs                 | array         | 必选  | []        | type support 包配置 |
| type_support_pkgs[i].path         | string        | 必选  | ""        | type support 包的路径 |
| executor                          | string        | 可选  | ""        | 回显使用的执行器，要求必须是线程安全 |
| topic_meta_list                   | array         | 必选  | []        | 要回显的 topic 和类型 |
| topic_meta_list[j].topic_name     | string        | 必选  | ""        | 要回显的 topic |
| topic_meta_list[j].msg_type       | string        | 必选  | ""        | 要回显的消息类型 |
| topic_meta_list[j].echo_type      | string        | 可选  | "json"    | 回显消息的格式，支持 "json", "yaml", "ros2" |



### 回显消息的简单示例配置
回显消息的存在两种配置，分别是 是否指定执行器 和 回显消息的格式：
- 是否指定执行器： 插件会使用指定的执行器来处理回显消息，如果未指定执行器，则使用默认的执行器；
- 回显消息的格式： 支持 "json", "yaml"，默认是 "json"

以下是一个带执行器的回显消息格式为 yaml 的简单示例配置：
```yaml
aimrt:
  plugin:
    plugins:
      - name: echo_plugin
        path: ./libaimrt_echo_plugin.so
        options:
          type_support_pkgs:
            - path: ./libexample_event_ts_pkg.so       
          executor: echo_executor
          topic_meta_list:
            - topic_name: test_topic
              msg_type: pb:aimrt.protocols.example.ExampleEventMsg    
              echo_type: yaml
  executor:
    executors:
      - name: echo_executor
        type: simple_thread
  channel:
    # ...
```


以下是一个不带执行器的回显消息格式为 json 的简单示例配置：
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
  executor:
    executors:
      - name: echo_executor
        type: simple_thread
  channel:
    # ...
```

