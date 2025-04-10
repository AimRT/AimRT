# aimrt.guard_thread

## 配置项概述

守护线程是 AimRT 在启动时会额外启动的一个线程，详情可参考[AimRT 核心设计理念](../concepts/core_design.md)。


`aimrt.guard_thread`配置项用于配置守护线程。其中的细节配置项说明如下：

| 节点                | 类型          | 是否可选| 默认值 | 作用 |
| ----                | ----          | ----  | ----  | ---- |
| name                | string        | 可选  | "aimrt_guard"    | 守护线程名称 |
| thread_sched_policy | string        | 可选  | ""    | 线程调度策略 |
| thread_bind_cpu     | unsigned int array | 可选 | [] | 绑核配置 |
| queue_threshold     | unsigned int       | 可选  | 10000 | 队列任务上限 |

`aimrt.guard_thread`使用注意点如下：
- `name`配置了守护线程名称，在实现时调用了操作系统的一些 API。如果操作系统不支持，则此项配置无效。
- `thread_sched_policy`和`thread_bind_cpu`参考[Common Information](./common.md)中线程绑核配置的说明。
- `queue_threshold`配置了队列任务上限，当已经有超过此阈值的任务在队列中时，新任务将投递失败。


## 使用示例

以下是一个简单的示例：
```yaml
aimrt:
  guard_thread:
    name: guard_thread
    thread_sched_policy: SCHED_FIFO:80
    thread_bind_cpu: [0, 1]
    queue_threshold: 10000
```
