# aimrt.main_thread

## Configuration Overview

The main thread is the startup thread of the AimRT instance, typically also the process startup thread. For details, refer to [AimRT Core Design Concepts](../concepts/core_design.md).

The `aimrt.main_thread` configuration item is used to configure the main thread. The detailed configuration items are described below:

| Node                | Type          | Optional | Default Value | Purpose |
| ----                | ----          | ----     | ----          | ----    |
| name                | string        | Optional | "aimrt_main"  | Main thread name |
| thread_sched_policy | string        | Optional | ""            | Thread scheduling policy |
| thread_bind_cpu     | unsigned int array | Optional | [] | CPU binding configuration |

Notes on using `aimrt.main_thread`:
- The `name` configures the main thread name and calls some operating system APIs during implementation. If the operating system does not support this feature, this configuration will be ineffective.
- For `thread_sched_policy` and `thread_bind_cpu`, refer to the thread binding configuration instructions in [Common Information](./common.md).

## Usage Example

Here is a simple example:
```yaml
aimrt:
  main_thread:
    name: main_thread
    thread_sched_policy: SCHED_FIFO:80
    thread_bind_cpu: [0, 1]
```