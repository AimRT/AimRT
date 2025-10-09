# aimrt.channel


## Configuration Overview

The `aimrt.channel` configuration item is used to configure the Channel feature. Detailed configuration items are described below:

| Node                                  | Type        | Optional | Default | Purpose |
| ----                                  | ----        | ----     | ----    | ----    |
| backends                              | array       | Optional | []      | List of Channel backends |
| backends[i].type                      | string      | Required | ""      | Type of the Channel backend |
| backends[i].options                   | map         | Optional | -       | Configuration for the specific Channel backend |
| pub_topics_options                    | array       | Optional | ""      | Configuration for Channel Pub Topics |
| pub_topics_options[i].topic_name      | string      | Required | ""      | Name of the Channel Pub Topic, supports regular expressions |
| pub_topics_options[i].enable_backends | string array | Required | []      | List of allowed Channel backends for the Channel Pub Topic |
| pub_topics_options[i].enable_filters  | string array | Optional | []      | List of framework-side filters to be loaded for the Channel Pub Topic |
| sub_topics_options                    | array       | Optional | ""      | Configuration for Channel Sub Topics |
| sub_topics_options[i].topic_name      | string      | Required | ""      | Name of the Channel Sub Topic, supports regular expressions |
| sub_topics_options[i].enable_backends | string array | Required | []      | List of allowed Channel backends for the Channel Sub Topic |
| sub_topics_options[i].enable_filters  | string array | Optional | []      | List of framework-side filters to be loaded for the Channel Sub Topic |


Detailed configuration instructions for `aimrt.channel`:
- `backends` is an array used to configure various Channel backends.
  - `backends[i].type` is the type of the Channel backend. AimRT officially provides the `local` backend, and some plugins also provide certain Channel backend types.
  - `backends[i].options` are the initialization parameters passed by AimRT to each Channel backend. The format of this configuration is defined by each Channel backend type; please refer to the documentation of the corresponding Channel backend type.
- `pub_topics_options` and `sub_topics_options` are rule lists used to control the Channel backend rules used by each `Topic` when publishing or subscribing to messages, where:
  - `topic_name` indicates the `Topic` name for this rule, configured as a regular expression. If the `Topic` name matches this regular expression, this rule will be applied.
  - `enable_backends` is a string array indicating that if the `Topic` name matches this rule, all messages published under this `Topic` will be delivered to all Channel backends in this list for processing. Note:
    - All names appearing in this array must have been configured in `backends`.
    - The order of Channel backends configured in this array determines the order in which messages are delivered to each Channel backend for processing.
  - `enable_filters` is a string array indicating the list of framework-side filters to be registered on the pub/sub side, with the order in the array being the order of filter registration. Some plugins provide framework-side filters for performing pre/post operations during channel invocation.
  - Rules are checked from top to bottom; once a `Topic` matches a rule, no further rules will be checked for that `Topic`.


In AimRT, the front-end interface and back-end implementation of the Channel are decoupled. After a message is `Publish`ed through the interface, the actual publishing action is carried out by the Channel backend. Typically, after the `Publish` call, the message is sequentially delivered to various Channel backends for processing within the current thread. Most Channel backends handle messages asynchronously, but some special backends—such as the `local` backend—can be configured to invoke subscriber callbacks in a blocking manner. Therefore, how long the `Publish` method will block is undefined and depends on the specific backend configuration.


Here is a simple example:

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


## local Type Channel Backend


The `local` type Channel backend is an officially provided Channel backend by AimRT, used to publish messages to other modules within the same process. It automatically determines whether the publisher and subscriber are within the same `Pkg` and optimizes performance accordingly. All its configuration items are as follows:


| Node                            | Type    | Optional | Default | Purpose |
| ----                            | ----    | ----     | ----    | ----    |
| subscriber_use_inline_executor  | bool    | Optional | true    | Whether the subscriber callback uses the inline executor |
| subscriber_executor             | string  | Required when subscriber_use_inline_executor is false | "" | Name of the executor used for subscriber callbacks |


Usage notes:
- If `subscriber_use_inline_executor` is set to `true`, the subscriber callback will be executed directly using the publisher's executor, blocking the publisher's Publish method until all subscriber callbacks have completed execution.
- `subscriber_executor` only takes effect when `subscriber_use_inline_executor` is `false`. The backend will deliver all subscriber callbacks to this executor for asynchronous execution, and the publisher's Publish method will return immediately.



Here is a simple example:

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
