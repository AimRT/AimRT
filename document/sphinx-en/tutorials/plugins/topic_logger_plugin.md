

# Topic Logger Plugin

## Related Links

Reference examples:
- {{ '[topic_logger_plugin]({}/src/examples/plugins/topic_logger_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**topic_logger_plugin** provides a log backend called `topic_logger` for AimRT's logging system, which publishes log data in topic format. When enabled in the configuration file, this log backend becomes available for use.

## Topic Logger Backend

`topic_logger` publishes logs as topics. Its complete configuration options are as follows:

| Node                 | Type   | Optional | Default   | Description                          |
| -------------------- | ------ | -------- | --------- | ------------------------------------- |
| topic_name           | string | Required | ""        | Topic name for log publishing         |
| timer_executor_name  | string | Required | ""        | Timer executor name                   |
| interval_ms          | string | Optional | 100       | Topic publishing interval (ms)        |
| module_filter        | string | Optional | (.*)      | Module filter regex                   |
| max_msg_size         | size_t | Optional | SIZE_MAX  | Maximum log message size (bytes)      |

Usage notes:
1. The backend publishes logs in protobuf format. For details, see {{ '[topic_logger.proto]({}/src/protocols/plugins/topic_logger_plugin/topic_logger.proto)'.format(code_site_root_path_url) }}
2. The plugin supports repeated registration, enabling different modules to publish logs through different topics
3. `timer_executor_name` must specify an executor that supports timer scheduling
4. `interval_ms` controls publishing frequency (default: 100ms). Balance between data integrity and performance
5. `module_filter` uses regular expressions to determine which modules' logs are processed by this backend. This works independently from global log level settings
6. `max_msg_size` specifies maximum log message length in bytes (default: SIZE_MAX). Note: The unit is bytes, not characters. For UTF-8 encoding, actual character count will be adjusted downward safely

Example configuration:
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