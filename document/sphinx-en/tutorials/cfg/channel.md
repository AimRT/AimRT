# aimrt.channel

## Configuration Overview

The `aimrt.channel` configuration item is used to configure the Channel functionality. The detailed configuration items are described below:

| Node                                  | Type        | Optional | Default Value | Purpose |
| ----                                  | ----        | ----     | ----          | ----    |
| backends                              | array       | Optional | []            | List of Channel backends |
| backends[i].type                      | string      | Required | ""            | Channel backend type |
| backends[i].options                   | map         | Optional | -             | Configuration for specific Channel backend |
| pub_topics_options                    | array       | Optional | ""            | Channel Pub Topic configuration |
| pub_topics_options[i].topic_name      | string      | Required | ""            | Channel Pub Topic name, supports regular expressions |
| pub_topics_options[i].enable_backends | string array | Required | []            | List of allowed Channel backends for Channel Pub Topic |
| pub_topics_options[i].enable_filters  | string array | Optional | []            | List of framework-side filters to be loaded for Channel Pub Topic |
| sub_topics_options                    | array       | Optional | ""            | Channel Sub Topic configuration |
| sub_topics_options[i].topic_name      | string      | Required | ""            | Channel Sub Topic name, supports regular expressions |
| sub_topics_options[i].enable_backends | string array | Required | []            | List of allowed Channel backends for Channel Sub Topic |
| sub_topics_options[i].enable_filters  | string array | Optional | []            | List of framework-side filters to be loaded for Channel Sub Topic |

Detailed configuration description for `aimrt.channel`:
- `backends` is an array used to configure various Channel backends.
  - `backends[i].type` specifies the type of Channel backend. AimRT officially provides the `local` backend, while some plugins may offer additional Channel backend types.
  - `backends[i].options` contains initialization parameters passed by AimRT to each Channel backend. The configuration format is defined by each Channel backend type. Please refer to the corresponding Channel backend type documentation.
- `pub_topics_options` and `sub_topics_options` are rule lists that control the Channel backend rules used when publishing or subscribing to messages for each `Topic`, where:
  - `topic_name` represents the `Topic` name for this rule, configured as a regular expression. If a `Topic` name matches this regular expression, the rule will be applied.
  - `enable_backends` is a string array indicating that if a `Topic` name matches this rule, all messages published under this `Topic` will be delivered to all Channel backends in this list for processing. Note:
    - All names appearing in this array must have been configured in `backends`.
    - The order of Channel backends in this array determines the sequence in which messages are delivered to each Channel backend for processing.
  - `enable_filters` is a string array representing the list of framework-side pub/sub filters to be registered. The array order determines the registration sequence of filters. Some plugins provide framework-side filters for performing pre/post operations during channel calls.
  - Rules are checked from top to bottom. Once a `Topic` matches a rule, subsequent rules will not be checked for this `Topic`.

In AimRT, the Channel frontend interface and backend implementation are decoupled. After calling `Publish` in the interface, the actual publishing action is ultimately performed by the Channel backend. Messages are typically delivered sequentially to each Channel backend for processing in the current thread after calling `Publish`. Most Channel backends process messages asynchronously, but some special backends—such as the `local` backend—can be configured to block and call subscriber callbacks synchronously. Therefore, how long the `Publish` method blocks is undefined and depends on the specific backend configuration.

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

## Local Type Channel Backend

The `local` type Channel backend is an official Channel backend provided by AimRT, used to publish messages to other modules within the same process. It automatically determines whether the publisher and subscriber are in the same `Pkg` and optimizes performance accordingly. All its configuration items are as follows:

| Node                            | Type    | Optional | Default Value | Purpose |
| ----                            | ----    | ----     | ----          | ----    |
| subscriber_use_inline_executor  | bool    | Optional | true          | Whether to use inline executor for subscriber callbacks |
| subscriber_executor             | string  | Required when subscriber_use_inline_executor is false | "" | Name of the executor used for subscriber callbacks |

Usage notes:
- If `subscriber_use_inline_executor` is set to `true`, the publisher's executor is used directly to execute subscriber callbacks, which will block the `Publish` method until all subscriber callbacks are completed.
- `subscriber_executor` takes effect only when `subscriber_use_inline_executor` is `false`. The backend will asynchronously deliver subscriber callbacks to this executor, and the `Publish` method will return immediately.

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