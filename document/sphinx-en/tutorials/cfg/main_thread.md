# aimrt.main_thread

## Configuration Overview

The main thread is the startup thread of the AimRT instance, typically also the process startup thread. For details, refer to [AimRT Core Design Concepts](../concepts/core_design.md).

The `aimrt.main_thread` configuration item is used to configure the main thread, with detailed configuration parameters as follows:

| Node                | Type          | Optional | Default | Description |
| ----                | ----          | ----     | ----    | ----        |
| name                | string        | Optional | "aimrt_main" | Main thread name |
| thread_sched_policy | string        | Optional | ""      | Thread scheduling policy |
| thread_bind_cpu     | unsigned int array | Optional | []    | CPU binding configuration |

Important notes for `aimrt.main_thread` configuration:
- The `name` configuration sets the main thread name through operating system APIs. This configuration will be ineffective if the OS doesn't support this feature.
- For `thread_sched_policy` and `thread_bind_cpu`, refer to the thread binding configuration instructions in [Common Information](./common.md).

## Usage Example

Below is a simple configuration example:
```yaml
aimrt:
  main_thread:
    name: main_thread
    thread_sched_policy: SCHED_FIFO:80
    thread_bind_cpu: [0, 1]
```