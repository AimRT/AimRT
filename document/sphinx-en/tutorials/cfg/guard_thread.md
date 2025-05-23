# aimrt.guard_thread

## Configuration Overview

The guard thread is an additional thread that AimRT starts during initialization. For details, please refer to [AimRT Core Design Concepts](../concepts/core_design.md).

The `aimrt.guard_thread` configuration item is used to configure the guard thread. The detailed configuration items are described below:

| Node                | Type          | Optional | Default Value | Description |
| ----                | ----          | ----     | ----          | ----        |
| name                | string        | Yes      | "aimrt_guard" | Guard thread name |
| thread_sched_policy | string        | Yes      | ""            | Thread scheduling policy |
| thread_bind_cpu     | unsigned int array | Yes | [] | CPU binding configuration |
| queue_threshold     | unsigned int  | Yes      | 10000         | Queue task threshold |

Notes for using `aimrt.guard_thread`:
- The `name` configures the guard thread name, which calls some operating system APIs during implementation. If the operating system doesn't support this, the configuration will be invalid.
- For `thread_sched_policy` and `thread_bind_cpu`, please refer to the thread binding configuration instructions in [Common Information](./common.md).
- The `queue_threshold` configures the maximum number of tasks in the queue. When the number of tasks in the queue exceeds this threshold, new task submissions will fail.

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