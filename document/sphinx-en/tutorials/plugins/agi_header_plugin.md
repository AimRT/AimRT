# agi_header plugin

## Related Links

Protocol fields:

- {{ '[protocol: agi_header]({}/src/protocols/pb/common/agi_header.proto)'.format(code_site_root_path_url) }}
- {{ '[protocol: ros2 AgiHeader]({}/src/protocols/ros2/aimrt_msgs/msg/AgiHeader.msg)'.format(code_site_root_path_url) }}
- {{ '[protocol: ros2 AgiRequestHeader]({}/src/protocols/ros2/aimrt_msgs/msg/AgiRequestHeader.msg)'.format(code_site_root_path_url) }}

## Plugin Overview

`agi_header_plugin` automatically detects and fills headers in runtime messages:

- `AgiHeader` (Channel publish messages)
- `AgiRequestHeader` (RPC client requests)

## Auto-filled Fields

- Channel publish:
  - `seq_num`
  - `publisher_name`
  - `publish_time`
- RPC client request:
  - `request_id`
  - `client_name`
  - `request_time`

## Filters

The plugin registers these filters:

- Channel publish filter: `agi_header_fill`
- RPC client filter: `agi_request_header_fill`

You must explicitly enable them with `enable_filters` under `channel.pub_topics_options` or `rpc.clients_options`.

## Example Configuration

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

## Behavior Notes

- The plugin scans all registered Publish Types / RPC Client Funcs at `kPreStart` and performs detection once.
- Detection results are stored in plugin-internal readonly tables; runtime filters only do table lookup and fill.
- Types without target headers are also recorded as negative results during the prestart scan.
- Current scope:
  - Channel publish path
  - RPC client request path
