# aimrt.channel

## Configuration Items Overview

The `aimrt.channel` configuration is used to set up Channel functionality. Detailed configuration item specifications are as follows:

| Node                                  | Type        | Optional | Default | Purpose |
| ----                                  | ----        | ----  | ----  | ---- |
| backends                              | array       | Yes   | []    | List of Channel backends |
| backends[i].type                      | string      | Required  | ""    | Channel backend type |
| backends[i].options                   | map         | Yes  | -     | Configuration for specific Channel backend |
| pub_topics_options                    | array       | Yes  | ""    | Channel Pub Topic configuration |
| pub_topics_options[i].topic_name      | string      | Required  | ""    | Channel Pub Topic name (supports regular expressions) |
| pub_topics_options[i].enable_backends | string array | Required  | [] | List of allowed Channel backends for Pub Topic |
| pub_topics_options[i].enable_filters  | string array | Yes  | [] | List of framework-side filters to load for Pub Topic |
| sub_topics_options                    | array       | Yes  | ""    | Channel Sub Topic configuration |
| sub_topics_options[i].topic_name      | string      | Required  | ""    | Channel Sub Topic name (supports regular expressions) |
| sub_topics_options[i].enable_backends | string array | Required  | [] | List of allowed Channel backends for Sub Topic |
| sub_topics_options[i].enable_filters  | string array | Yes  | [] | List of framework-side filters to load for Sub Topic |

## Detailed Configuration Explanation

- The `backends` array configures various Channel backends:
  - `backends[i].type`: Specifies Channel backend type. AimRT officially provides the `local` backend, while some plugins may offer additional types.
  - `backends[i].options`: Initialization parameters passed to each Channel backend. Configuration format depends on specific backend type. Refer to corresponding documentation.
- `pub_topics_options` and `sub_topics_options` control backend rules for message publishing/subscribing:
  - `topic_name`: Regular expression pattern for matching Topic names
  - `enable_backends`: List of Channel backends for message processing (order determines processing sequence)
    - All listed names must be configured in `backends`
  - `enable_filters`: Ordered list of framework-side filters to register (execution order matches list order)
  - Rules are checked top-down. First matching rule applies exclusively.

In AimRT, Channel frontend interfaces and backend implementations are decoupled. The `Publish` method delegates actual publishing to configured backends. Messages are typically processed sequentially in the calling thread. Most backends handle messages asynchronously, though some (e.g., `local`) may support blocking subscriber callbacks. Therefore, `Publish` blocking duration depends on specific backend configuration.

Example configuration:
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

The `local` Channel backend (officially provided by AimRT) enables intra-process message publishing. It automatically optimizes performance based on whether publishers and subscribers reside in the same `Pkg`. Configuration items:

| Node                            | Type    | Optional | Default | Purpose |
| ----                            | ----    | ----  | ----  | ---- |
| subscriber_use_inline_executor  | bool    | Yes  | true  | Whether to use inline executor for subscriber callbacks |
| subscriber_executor             | string  | Required when `subscriber_use_inline_executor`=false | "" | Executor name for subscriber callbacks |

Usage Notes:
- When `subscriber_use_inline_executor`=true, uses publisher's executor synchronously, blocking `Publish` until all callbacks complete
- `subscriber_executor` takes effect only when `subscriber_use_inline_executor`=false, enabling asynchronous callback execution

Example configuration:
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