# aimrt.channel


## 配置项概述

`aimrt.channel`配置项用于配置 Channel 功能。其中的细节配置项说明如下：

| 节点                                  | 类型        | 是否可选| 默认值 | 作用 |
| ----                                  | ----        | ----  | ----  | ---- |
| backends                              | array       | 可选  | []    | Channel 后端列表 |
| backends[i].type                      | string      | 必选  | ""    | Channel 后端类型 |
| backends[i].options                   | map         | 可选  | -     | 具体 Channel 后端的配置 |
| pub_topics_options                    | array       | 可选  | ""    | Channel Pub Topic 配置 |
| pub_topics_options[i].topic_name      | string      | 必选  | ""    | Channel Pub Topic 名称，支持正则表达式 |
| pub_topics_options[i].enable_backends | string array | 必选  | [] | Channel Pub Topic 允许使用的 Channel 后端列表 |
| pub_topics_options[i].enable_filters  | string array | 可选  | [] | Channel Pub Topic 需要加载的框架侧过滤器列表 |
| sub_topics_options                    | array       | 可选  | ""    | Channel Sub Topic 配置 |
| sub_topics_options[i].topic_name      | string      | 必选  | ""    | Channel Sub Topic 名称，支持正则表达式 |
| sub_topics_options[i].enable_backends | string array | 必选  | [] | Channel Sub Topic 允许使用的 Channel 后端列表 |
| sub_topics_options[i].enable_filters  | string array | 可选  | [] | Channel Sub Topic 需要加载的框架侧过滤器列表 |


`aimrt.channel`的详细配置说明如下：
- `backends`是一个数组，用于配置各个 Channel 后端。
  - `backends[i].type`是 Channel 后端的类型。AimRT 官方提供了`local`后端，部分插件也提供了一些 Channel 后端类型。
  - `backends[i].options`是 AimRT 传递给各个 Channel 后端的初始化参数，这部分配置格式由各个 Channel 后端类型定义，请参考对应 Channel 后端类型的文档。
- `pub_topics_options`和`sub_topics_options`是一个规则列表，用于控制各个`Topic`在发布或订阅消息时使用的 Channel 后端规则，其中：
  - `topic_name`表示本条规则的`Topic`名称，以正则表达式形式配置，如果`Topic`名称命中了该正则表达式，则会应用该条规则。
  - `enable_backends`是一个字符串数组，表示如果`Topic`名称命中了本条规则，则将该`Topic`下发布的所有消息投递到这个列表中的所有 Channel 后端进行处理。注意：
    - 该数组中出现的名称都必须要在`backends`中配置过。
    - 该数组配置的 Channel 后端顺序决定了消息投递到各个 Channel 后端进行处理的顺序。
  - `enable_filters`是一个字符串数组，表示需要注册的 pub/sub 端框架侧过滤器列表，数组中的顺序为过滤器注册的顺序。一些插件会提供一些框架侧过滤器，用于在 channel 调用时做一些前置/后置操作。
  - 采用由上往下的顺序检查命中的规则，当某个`Topic`命中某条规则后，则不会针对此`Topic`再检查后面的规则。


在 AimRT 中，Channel 的前端接口和后端实现是解耦的，在接口中`Publish`一个消息后最终是要 Channel 后端来进行正真的发布动作，消息通常会在调用`Publish`之后，在当前线程里顺序的投递到各个 Channel 后端中进行处理。大部分 Channel 后端是异步处理消息的，但有些特殊的后端-例如`local`后端，可以配置成阻塞的调用订阅端回调。因此，`Publish`方法到底会阻塞多久是未定义的，与具体的后端配置相关。


以下是一个简单的示例：
```yaml
aimrt:
  channel:
    backends:
      - type: local
      - type: mqtt
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [local]
        enable_filters: []
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [local]
        enable_filters: []
```

## local 类型 Channel 后端


`local`类型的 Channel 后端是 AimRT 官方提供的一种 Channel 后端，用于将消息发布到同进程中的其他模块，它会自动判断发布端和订阅端是否在同一个`Pkg`内，从而采用各种方式进行性能的优化。其所有的配置项如下：


| 节点                            | 类型    | 是否可选| 默认值 | 作用 |
| ----                            | ----    | ----  | ----  | ---- |
| subscriber_use_inline_executor  | bool    | 可选  | true  | 订阅端回调是否使用inline执行器 |
| subscriber_executor             | string  | subscriber_use_inline_executor为false时必选  | "" | 订阅端回调使用的执行器名称 |


使用注意点如下：
- `subscriber_use_inline_executor`如果配置为`true`，则直接使用发布端的执行器来执行订阅端的回调，会阻塞发布端的 Publish 方法直到所有的订阅端回调都执行完成。
- `subscriber_executor`仅在`subscriber_use_inline_executor`为`false`时生效，后端会将订阅端的回调都投递进此执行器中异步执行，发布端的 Publish 方法会立即返回。



以下是一个简单的示例：
```yaml
aimrt:
  executor:
    executors:
      - name: work_thread_pool
        type: asio_thread
        options:
          thread_num: 4
  channel:
    backends:
      - type: local
        options:
          subscriber_use_inline_executor: false
          subscriber_executor: work_thread_pool
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [local]
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [local]
```
