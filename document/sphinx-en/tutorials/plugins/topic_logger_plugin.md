# Topic Logger Plugin

## Related Links

Reference example:
- {{ '[topic_logger_plugin]({}/src/examples/plugins/topic_logger_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**topic_logger_plugin** provides a log backend `topic_logger` for AimRT, which is used to publish data in the form of topics. When this plugin is enabled in the configuration file, this log backend can be used.

## Topic Logger Backend

`topic_logger` is used to send logs in the form of topics. All its configuration items are as follows:

| Node                | Type   | Optional | Default   | Description                     |
| ------------------- | ------ | -------- | -------- | ------------------------------- |
| topic_name          | string | Required | ""       | Topic name for log publishing   |
| timer_executor_name | string | Required | ""       | Timer name                      |
| interval_ms         | string | Optional | 100      | Topic publishing interval       |
| module_filter       | string | Optional | (.*)     | Module filter                   |
| max_msg_size        | size_t | Optional | SIZE_MAX | Maximum length of log messages, unit: byte |

Usage notes:
- This backend will publish topics in `protobuf` format. For specific content, see {{ '[topic_logger.proto]({}/src/protocols/plugins/topic_logger_plugin/topic_logger.proto)'.format(code_site_root_path_url) }}.
- This plugin will publish in topic form
- The `topic_logger` backend allows repeated registration. Based on this feature, businesses can publish logs from different modules through different topics.
- `timer_executor_name` configures the executor for periodic topic publishing. It must support timer scheduling.
- `interval_ms` configures the interval for periodic disk writing, unit: ms, default 100 ms. Please set an appropriate value considering the balance between data integrity and performance.
- `module_filter` supports configuring which modules' logs can be processed by this backend in the form of regular expressions. This is different from the module log level, which is global, prerequisite, and affects all log backends, while this configuration only affects this backend.
- `max_msg_size` configures the maximum length of log messages, unit: byte, default SIZE_MAX. Note that the unit is byte, not character count. For example, for UTF-8 encoded characters, the actual length will be adjusted downward to a safe length based on the actual situation.

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