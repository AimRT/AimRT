# aimrt.guard_thread

## Configuration Overview

The guard thread is an additional thread that AimRT starts at startup. For details, refer to [AimRT Core Design Philosophy](../concepts/core_design.md).

The `aimrt.guard_thread` configuration item is used to configure the guard thread. The detailed configuration items are described below:

| Node                        | Type               | Optional | Default Value | Description                                |
| --------------------------- | ------------------ | -------- | ------------- | ------------------------------------------ |
| name                        | string             | Optional | "aimrt_guard" | Guard thread name                          |
| thread_sched_policy         | string             | Optional | ""            | Thread scheduling policy                   |
| thread_bind_cpu             | unsigned int array | Optional | []            | CPU binding configuration                  |
| queue_threshold             | unsigned int       | Optional | 10000         | Queue task upper limit                     |
| threshold_alarm_interval_ms | int                | Optional | 1000          | Thread usage threshold alarm interval, unit: milliseconds |

Notes for using `aimrt.guard_thread`:

- `name` configures the guard thread name, which calls some operating system APIs during implementation. If the operating system does not support it, this configuration will be invalid.
- `thread_sched_policy` and `thread_bind_cpu` refer to the thread CPU binding configuration instructions in [Common Information](./common.md).
- `queue_threshold` configures the upper limit of queue tasks. When there are more than this threshold of tasks already in the queue, new tasks will fail to be submitted.
- `threshold_alarm_interval_ms` configures the thread usage threshold alarm interval. When the thread usage exceeds the threshold, an alarm log will be printed at regular intervals. If set to a negative number, this log printing will be disabled.

## Usage Example

Here is a simple example:


```yaml
aimrt:
  guard_thread:
    name: guard_thread
    thread_sched_policy: SCHED_FIFO:80
    thread_bind_cpu: [0, 1]
    queue_threshold: 10000
```
