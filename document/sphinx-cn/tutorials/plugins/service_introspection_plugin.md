# service introspection 插件

## 相关链接

协议字段：

- {{ '[protocol: service_introspection_plugin]({}/src/protocols/plugins/service_introspection_plugin)'.format(code_site_root_path_url) }}

参考示例：

- {{ '[example: service_introspection_plugin]({}/src/examples/plugins/service_introspection_plugin)'.format(code_site_root_path_url) }}

## 插件概述

**service_introspection_plugin** 插件用于将 RPC 通信链路的请求、响应和元信息（如序列号、时间戳、服务名等）以 `Topic` 形式发布到指定的话题上，方便进行服务跟踪和问题诊断。

插件的配置项如下：

| 节点                   | 类型   | 是否可选 | 默认值 | 作用                 |
| ---------------------- | ------ | -------- | ------ | -------------------- |
| mode                   | string | 可选     | full   | 插件运行模式         |
| rpc_serialization_type | string | 可选     | json   | rpc 数据序列化类型   |
| client_info_topic_name | string | 必选     | ""     | 客户端信息发布话题名 |
| server_info_topic_name | string | 必选     | ""     | 服务端信息发布话题名 |

使用注意点如下：

- `mode` 表示插件运行模式，可选值为 `meta` 、 `hybrid` 和 `full`，默认为 `full`。这些模式分别表示：

  - `meta` 模式：只记录客户端和服务端的 id、时间戳等元信息。
  - `hybrid` 模式：记录客户端的元信息、请求和响应数据，服务端的元信息。
  - `full` 模式：记录客户端的元信息、请求和响应数据，服务端的元信息、请求和响应数据。

- `rpc_serialization_type` 表示 RPC 数据序列化类型，可选值为 `json` 和 `auto`，默认值为 `json`，如果选择 `auto` 则会根据请求数据自动选择序列化类型（protobuf 或者 ros2）。
- `client_info_topic_name`和`server_info_topic_name` 表分别表示客户端信息发布话题名和服务端信息发布话题名， 如果二者设置的话题名一致，则只会通过一个发布者发布信息；如果二者设置的话题名不一致，则会通过两个发布者分别发布信息。

### 使用示例配置

以下是一个简单示例配置：

```yaml
aimrt:
  plugin:
    plugins:
      - name: service_introspection_plugin
        path: ./libaimrt_service_introspection_plugin.so
        options:
          client_info_topic_name: /service_introspection
          server_info_topic_name: /service_introspection
          mode: full
          rpc_serialization_type: json
  rpc:
    backends:
      - type: local
    clients_options:
      - func_name: "(.*)"
        enable_backends: [local]
        enable_filters: [service_introspection]
    servers_options:
      - func_name: "(.*)"
        enable_backends: [local]
        enable_filters: [service_introspection]
    # ...
```
