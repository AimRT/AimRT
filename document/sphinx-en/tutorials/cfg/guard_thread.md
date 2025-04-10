

# aimrt.guard_thread

## Configuration Overview

The guard thread is an additional thread launched during AimRT startup. For details, refer to [AimRT Core Design Concepts](../concepts/core_design.md).

The `aimrt.guard_thread` configuration item is used to configure the guard thread. Detailed configuration specifications are as follows:

| Node                | Type          | Optional | Default Value | Description |
| ----                | ----          | ----     | ----          | ----        |
| name                | string        | Yes      | "aimrt_guard" | Guard thread name |
| thread_sched_policy | string        | Yes      | ""            | Thread scheduling policy |
| thread_bind_cpu     | unsigned int array | Yes | []       | Core binding configuration |
| queue_threshold     | unsigned int  | Yes      | 10000         | Task queue threshold |

Usage notes for `aimrt.guard_thread`:
- The `name` configuration sets the guard thread name, which invokes OS-level APIs during implementation. This configuration becomes invalid if the OS lacks support.
- For `thread_sched_policy` and `thread_bind_cpu`, refer to thread binding configuration descriptions in [Common Information](./common.md).
- The `queue_threshold` specifies the maximum queue capacity. New task submissions will fail when existing queued tasks exceed this threshold.

## Usage Example

Here is a basic example:
```yaml
aimrt:
  guard_thread:
    name: guard_thread
    thread_sched_policy: SCHED_FIFO:80
    thread_bind_cpu: [0, 1]
    queue_threshold: 10000
```