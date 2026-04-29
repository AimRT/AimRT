# agi_header 插件

## 相关链接

协议字段：

- {{ '[protocol: agi_header]({}/src/protocols/pb/common/agi_header.proto)'.format(code_site_root_path_url) }}
- {{ '[protocol: ros2 AgiHeader]({}/src/protocols/ros2/aimrt_msgs/msg/AgiHeader.msg)'.format(code_site_root_path_url) }}
- {{ '[protocol: ros2 AgiRequestHeader]({}/src/protocols/ros2/aimrt_msgs/msg/AgiRequestHeader.msg)'.format(code_site_root_path_url) }}

## 插件概述

`agi_header_plugin` 用于在运行时自动检测并填充业务消息中的：

- `AgiHeader`（Channel 发布消息）
- `AgiRequestHeader`（RPC Client 请求消息）

## 自动填充字段

- Channel 发布：
  - `seq_num`
  - `publisher_name`
  - `publish_time`
- RPC Client 请求：
  - `request_id`
  - `client_name`
  - `request_time`

## 过滤器

插件会注册以下过滤器：

- Channel Publish Filter：`agi_header_fill`
- RPC Client Filter：`agi_request_header_fill`

需要在 `channel.pub_topics_options` 或 `rpc.clients_options` 中通过 `enable_filters` 显式启用。

## 配置示例

```yaml
aimrt:
  plugin:
    plugins:
      - name: agi_header_plugin
        path: ./libaimrt_agi_header_plugin.so
  module:
    # ...
  channel:
    pub_topics_options:
      - topic_name: "(.*)"
        enable_filters: [agi_header_fill]
  rpc:
    clients_options:
      - func_name: "(.*)"
        enable_filters: [agi_request_header_fill]
```

## 行为说明

- 插件会在 `kPreStart` 阶段扫描已注册的 Publish Type / RPC Client Func，并统一执行检测。
- 检测结果保存在插件内部只读表中，运行期过滤器仅查表填充，不再执行反射检测。
- 当消息/请求不包含目标 header 时，也会在扫描阶段记录“无 header”结果。
- 当前仅处理：
  - Channel 发布路径
  - RPC Client 请求路径
