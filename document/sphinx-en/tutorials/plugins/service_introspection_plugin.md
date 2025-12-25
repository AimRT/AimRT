# service introspection plugin

## Related Links

Protocol fields:

- {{ '[protocol: service_introspection_plugin]({}/src/protocols/plugins/service_introspection_plugin)'.format(code_site_root_path_url) }}

Reference examples:

- {{ '[example: service_introspection_plugin]({}/src/examples/plugins/service_introspection_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

The **service_introspection_plugin** is used to publish RPC communication link requests, responses, and metadata (such as sequence numbers, timestamps, service names, etc.) as `Topic` messages to specified topics, facilitating service tracking and issue diagnosis.

The plugin configuration options are as follows:

| Node                   | Type   | Optional | Default | Description                   |
| ---------------------- | ------ | -------- | ------- | ----------------------------- |
| mode                   | string | Optional | full    | Plugin operation mode         |
| rpc_serialization_type | string | Optional | json    | RPC data serialization type   |
| client_info_topic_name | string | Required | ""      | Client info publication topic |
| server_info_topic_name | string | Required | ""      | Server info publication topic |

Usage notes:

- `mode` specifies the plugin operation mode. Valid values are `meta`, `hybrid`, and `full`, with `full` as the default. These modes represent:

  - `meta` mode: Records only metadata such as client and server IDs, timestamps, etc.
  - `hybrid` mode: Records client metadata, request and response data, and server metadata.
  - `full` mode: Records client metadata, request and response data, server metadata, request and response data.

- `rpc_serialization_type` specifies the RPC data serialization type. Valid values are `json` and `auto`, with `json` as the default. If `auto` is selected, the serialization type (protobuf or ros2) will be automatically determined based on the request data.

- `client_info_topic_name` and `server_info_topic_name` specify the client info publication topic name and server info publication topic name, respectively. If both are set to the same topic name, information will be published through a single publisher. If they are set to different topic names, information will be published through two separate publishers.

### Example Configuration

The following is a simple example configuration:

````yaml
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
````
