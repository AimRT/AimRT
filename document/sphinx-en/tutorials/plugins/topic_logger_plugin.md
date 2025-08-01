# topic logger plugin

## Related Links

Reference example:

- {{ '[topic_logger_plugin]({}/src/examples/plugins/topic_logger_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**topic_logger_plugin** provides a log backend called `topic_logger` for AimRT's logging system. It publishes log data as topics. Once this plugin is enabled in the configuration file, the backend becomes available.

## topic_logger Topic Log Backend

`topic_logger` emits logs as topics. All configuration items are listed below:

| Node                | Type   | Optional | Default  | Purpose                                      |
| ------------------- | ------ | -------- | -------- | -------------------------------------------- |
| topic_name          | string | Required | ""       | Name of the topic to which logs are sent     |
| timer_executor_name | string | Required | ""       | Timer executor name                          |
| interval_ms         | string | Optional | 500      | Topic publish interval                       |
| module_filter       | string | Optional | (.\*)    | Module filter                                |
| max_msg_size        | size_t | Optional | SIZE_MAX | Maximum log message length in bytes          |
| max_msg_count       | size_t | Optional | SIZE_MAX | Maximum number of messages to publish        |

Usage notes:

- This backend publishes topics in `protobuf` format; see {{ '[topic_logger.proto]({}/src/protocols/plugins/topic_logger_plugin/topic_logger.proto)'.format(code_site_root_path_url) }} for details.
- The plugin publishes log data as topics, so you must configure the corresponding topic publisher backend in the configuration file.
- The `topic_logger` backend supports multiple registrations. You can leverage this feature to publish logs from different modules to different topics.
- `timer_executor_name` specifies the executor responsible for periodically publishing the topic. It must support timer scheduling.
- `interval_ms` sets the periodic flush interval in milliseconds; the default is 500 ms. Choose an appropriate value to balance data completeness and performance.
- `module_filter` allows filtering which modules' logs are handled by this backend using regular expressions. This differs from module log levels: log levels are global, precede any backend, and affect all backends, whereas this setting only affects the current backend.
- `max_msg_size` limits the maximum log message length in bytes; the default is SIZE_MAX. Note the unit is bytes, not characters. For UTF-8 encoded text, the actual length is adjusted downward to a safe value. Be aware this option may impact log completeness.
- `max_msg_count` limits the maximum number of messages published; the default is SIZE_MAX. It is mainly used to restrict high-frequency log printing. Be aware this option may impact log completeness. You can check the `dropped_count` field in the received data to determine if any data was lost.

Here is a simple configuration example:
```yaml
aimrt:
  plugin:
    plugins:
      - name: topic_logger_plugin
        path: ./libaimrt_topic_logger_plugin.so
  log:
    core_lvl: INFO # Trace/Debug/Info/Warn/Error/Fatal/Off
    backends:
      - type: topic_logger
        options:
          timer_executor_name: timer_executor
          topic_name: test_topic
  executor:
    executors:
      - name: timer_executor
        type: asio_thread
  channel:
    # ...
```
