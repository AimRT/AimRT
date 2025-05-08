# topic logger 插件

## 相关链接

参考示例：
- {{ '[topic_logger_plugin]({}/src/examples/plugins/topic_logger_plugin)'.format(code_site_root_path_url) }}

## 插件概述

**topic_logger_plugin**为 AimRT 的日志提供了一种日志后端 `topic_logger`， 其用于以话题的形式将数据发布出去， 当在配置文件中开启该插件后，即可使用该日志后端。

## topic_logger 话题日志后端

`topic_logger`，用于将日志以话题形式发出。其所有的配置项如下：

| 节点                | 类型   | 是否可选 | 默认值   | 作用                            |
| ------------------- | ------ | -------- | -------- | ------------------------------- |
| topic_name          | string | 必选     | ""       | 日志发送的 topic 名称           |
| timer_executor_name | string | 必选     | ""       | 定时器名称                      |
| interval_ms         | string | 可选     | 100      | topic 发布间隔时间              |
| module_filter       | string | 可选     | (.*)     | 模块过滤器                      |
| max_msg_size        | size_t | 可选     | SIZE_MAX | 日志消息的最大长度， 单位：byte |

使用注意点如下：
- 该后端将以`protobuf`格式将话题发布， 具体内容详见{{ '[topic_logger.proto]({}/src/protocols/plugins/topic_logger_plugin/topic_logger.proto)'.format(code_site_root_path_url) }}。
- 该插件会以话题形式
- `topic_logger`日志后端允许重复注册，业务可以基于这个特点，将不同模块的日志以不同话题发布出去。
- `timer_executor_name`配置定期发布话题的执行器。其必须支持 timer scheduling。
- `interval_ms`配置定期落盘的时间间隔，单位：ms， 默认100 ms。 请在数据完整性和性能之间设置合适大小。
- `module_filter`支持以正则表达式的形式，来配置哪些模块的日志可以通过本后端处理。这与模块日志等级不同，模块日志等级是全局的、先决的、影响所有日志后端的，而这里的配置只影响本后端。
- `max_msg_size`配置日志消息的最大长度，单位：byte， 默认 SIZE_MAX 。 需要注意的是单位是 byte，而不是字符数，例如对于 UTF-8 编码的字符会根据实际情况向下调整为安全长度。

以下是一个简单示例配置：
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



