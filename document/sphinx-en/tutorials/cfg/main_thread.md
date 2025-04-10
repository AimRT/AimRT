# aimrt.main_thread

## 配置项概述

主线程即 AimRT 实例启动线程，通常也是进程启动线程，详情可参考[AimRT 核心设计理念](../concepts/core_design.md)。

`aimrt.main_thread`配置项用于配置主线程，其中的细节配置项说明如下：

| 节点                | 类型          | 是否可选| 默认值 | 作用 |
| ----                | ----          | ----  | ----  | ---- |
| name                | string        | 可选  | "aimrt_main"    | 主线程名称 |
| thread_sched_policy | string        | 可选  | ""    | 线程调度策略 |
| thread_bind_cpu     | unsigned int array | 可选 | [] | 绑核配置 |


`aimrt.main_thread`使用注意点如下：
- `name`配置了主线程名称，在实现时调用了操作系统的一些 API。如果操作系统不支持，则此项配置无效。
- `thread_sched_policy`和`thread_bind_cpu`参考[Common Information](./common.md)中线程绑核配置的说明。


## 使用示例

以下是一个简单的示例：
```yaml
aimrt:
  main_thread:
    name: main_thread
    thread_sched_policy: SCHED_FIFO:80
    thread_bind_cpu: [0, 1]
```
