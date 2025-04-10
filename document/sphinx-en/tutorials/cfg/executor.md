
# aimrt.executor

## 配置项概述

`aimrt.executor`配置项用于配置执行器。其中的细节配置项说明如下：

| 节点                    | 类型      | 是否可选| 默认值 | 作用 |
| ----                    | ----      | ----  | ----  | ---- |
| executors               | array     | 可选  | []    | 执行器列表 |
| executors[i].name       | string    | 必选  | ""    | 执行器名称 |
| executors[i].type       | string    | 必选  | ""    | 执行器类型 |
| executors[i].options    | map       | 可选  | -     | 具体执行器的配置 |

`aimrt.executor`的配置说明如下：
- `executors`是一个数组，用于配置各个执行器。
  - `executors[i].name`表示执行器名称。不允许出现重复的执行器名称。
  - `executors[i].type`表示执行器类型。AimRT 官方提供了几种执行器类型，部分插件也提供了一些执行器类型。
  - `executors[i].options`是AimRT传递给各个执行器的初始化参数，这部分配置格式由各个执行器类型定义，请参考对应执行器类型的文档章节。


以下是一个简单的示例：
```yaml
aimrt:
  executor:
    executors:
      - name: xxx_executor
        type: asio_thread
        options:
          thread_num: 2
      - name: yyy_executor
        type: tbb_thread
        options:
          thread_num: 1
```


## simple_thread 执行器

`simple_thread`执行器是一种简单的单线程执行器，不支持定时调度。其所有的配置项如下：

| 节点                          | 类型                  | 是否可选| 默认值 | 作用 |
| ----                          | ----                  | ----  | ----  | ---- |
| thread_sched_policy           | string                | 可选  | ""    | 线程调度策略 |
| thread_bind_cpu               | unsigned int array    | 可选  | []    | 绑核配置 |
| queue_threshold               | unsigned int          | 可选  | 10000 | 队列任务上限 |

使用注意点如下：
- `thread_sched_policy`和`thread_bind_cpu`参考[Common Information](./common.md)中线程绑核配置的说明。
- `queue_threshold`配置了队列任务上限，当已经有超过此阈值的任务在队列中时，新任务将投递失败。


以下是一个简单的示例：
```yaml
aimrt:
  executor:
    executors:
      - name: test_simple_thread_executor
        type: simple_thread
        options:
          thread_sched_policy: SCHED_FIFO:80
          thread_bind_cpu: [0, 1]
          queue_threshold: 10000
```

## asio_thread 执行器

`asio_thread`执行器是一种基于[Asio库](https://github.com/chriskohlhoff/asio)实现的执行器，是一种线程池，可以手动设置线程数，此外它还支持定时调度。其所有的配置项如下：


| 节点                          | 类型                  | 是否可选| 默认值 | 作用 |
| ----                          | ----                  | ----  | ----  | ---- |
| thread_num                    | unsigned int          | 可选  | 1     | 线程数 |
| thread_sched_policy           | string                | 可选  | ""    | 线程调度策略 |
| thread_bind_cpu               | unsigned int array    | 可选  | []    | 绑核配置 |
| timeout_alarm_threshold_us    | unsigned int          | 可选  | 1000000 | 调度超时告警阈值，单位：微秒 |
| use_system_clock              | bool                  | 可选  | false | 是否使用 std::system_clock，默认使用 std::steady_clock |


使用注意点如下：
- `thread_num`配置了线程数，默认为 1。当线程数配置为 1 时为线程安全执行器，否则是线程不安全的。
- `thread_sched_policy`和`thread_bind_cpu`参考[Common Information](./common.md)中线程绑核配置的说明。
- `timeout_alarm_threshold_us`配置了一个调度超时告警的阈值。当进行定时调度时，如果 CPU 负载太重、或队列中任务太多，导致超过设定的时间才调度到，则会打印一个告警日志。
- `use_system_clock`配置是否使用 std::system_clock 作为时间系统，默认为 false，使用 std::steady_clock。注意使用 std::system_clock 时，执行器的时间将与系统同步，可能会受到外部调节。

以下是一个简单的示例：
```yaml
aimrt:
  executor:
    executors:
      - name: test_asio_thread_executor
        type: asio_thread
        options:
          thread_num: 2
          thread_sched_policy: SCHED_FIFO:80
          thread_bind_cpu: [0, 1]
          timeout_alarm_threshold_us: 1000
          use_system_clock: false
```

## asio_strand 执行器

`asio_strand`执行器是一种依附于`asio_thread`执行器的伪执行器，基于 Asio 库的 strand 实现。它不能独立存在，并不拥有实际的线程，它在运行过程中会将任务交给绑定的`asio_thread`执行器来实际执行。但是它保证线程安全，也支持定时调度。其所有的配置项如下：

| 节点                              | 类型          | 是否可选| 默认值 | 作用 |
| ----                              | ----          | ----  | ----  | ---- |
| bind_asio_thread_executor_name    | string        | 必选  | ""     | 绑定的asio_thread执行器名称 |
| timeout_alarm_threshold_us        | unsigned int  | 可选  | 1000000 | 调度超时告警阈值，单位：微秒 |
| use_system_clock              | bool                  | 可选  | false | 是否使用 std::system_clock，默认使用 std::steady_clock |

使用注意点如下：
- 通过`bind_asio_thread_executor_name`配置项来绑定`asio_thread`类型的执行器。如果指定名称的执行器不存在、或不是`asio_thread`类型，则会在初始化时抛出异常。
- `timeout_alarm_threshold_us`配置了一个调度超时告警的阈值。当进行定时调度时，如果 CPU 负载太重、或队列中任务太多，导致超过设定的时间才调度到，则会打印一个告警日志。
- `use_system_clock`配置是否使用 std::system_clock 作为时间系统，默认为 false，使用 std::steady_clock。注意使用 std::system_clock 时，执行器的时间将与系统同步，可能会受到外部调节。


以下是一个简单的示例：
```yaml
aimrt:
  executor:
    executors:
      - name: test_asio_thread_executor
        type: asio_thread
        options:
          thread_num: 2
      - name: test_asio_strand_executor
        type: asio_strand
        options:
          bind_asio_thread_executor_name: test_asio_thread_executor
          timeout_alarm_threshold_us: 1000
          use_system_clock: false
```



## tbb_thread 执行器

`tbb_thread`是一种基于[oneTBB 库](https://github.com/oneapi-src/oneTBB)的无锁并发队列实现的高性能无锁线程池，可以手动设置线程数，但它不支持定时调度。其所有的配置项如下：

| 节点                          | 类型                  | 是否可选| 默认值 | 作用 |
| ----                          | ----                  | ----  | ----  | ---- |
| thread_num                    | unsigned int          | 可选  | 1     | 线程数 |
| thread_sched_policy           | string                | 可选  | ""    | 线程调度策略 |
| thread_bind_cpu               | unsigned int array    | 可选  | []    | 绑核配置 |
| timeout_alarm_threshold_us    | unsigned int          | 可选  | 1000000 | 调度超时告警阈值，单位：微秒 |
| queue_threshold               | unsigned int          | 可选  | 10000 | 队列任务上限 |


使用注意点如下：
- `thread_num`配置了线程数，默认为 1。当线程数配置为 1 时为线程安全执行器，否则是线程不安全的。
- `thread_sched_policy`和`thread_bind_cpu`参考[Common Information](./common.md)中线程绑核配置的说明。
- `timeout_alarm_threshold_us`配置了一个调度超时告警的阈值。当进行定时调度时，如果 CPU 负载太重、或队列中任务太多，导致超过设定的时间才调度到，则会打印一个告警日志。
- `queue_threshold`配置了队列任务上限，当已经有超过此阈值的任务在队列中时，新任务将投递失败。


以下是一个简单的示例：
```yaml
aimrt:
  executor:
    executors:
      - name: test_tbb_thread_executor
        type: tbb_thread
        options:
          thread_num: 2
          thread_sched_policy: SCHED_FIFO:80
          thread_bind_cpu: [0, 1]
          timeout_alarm_threshold_us: 1000
          queue_threshold: 10000
```


## time_wheel 执行器

`time_wheel`执行器是一种基于时间轮实现的执行器，一般用于有大量定时任务、且对定时精度要求不高的场景，例如 RPC 超时处理。它会启动一个单独的线程跑时间轮，同时也支持将具体的任务投递到其他执行器中执行。其所有的配置项如下：


| 节点                  | 类型                  | 是否可选| 默认值 | 作用 |
| ----                  | ----                  | ----  | ----  | ---- |
| bind_executor         | string                | 可选  | ""    | 绑定的执行器 |
| dt_us                 | unsigned int          | 可选  | 100000  | 时间轮 tick 的间隔，单位：微秒 |
| wheel_size            | unsigned int array    | 可选  | [100, 360]    | 各个时间轮的大小 |
| thread_sched_policy   | string                | 可选  | ""    | 时间轮线程的调度策略 |
| thread_bind_cpu       | unsigned int array    | 可选  | []    | 时间轮线程的绑核配置 |
| use_system_clock      | bool                  | 可选  | false | 是否使用 std::system_clock，默认使用 std::steady_clock |

使用注意点如下：
- `bind_executor`用于配置绑定的执行器，从而在时间达到后将任务投递到绑定的执行器里具体执行。
  - 如果不绑定其他执行器，则所有的任务都会在时间轮线程里执行，有可能阻塞时间轮的 Tick。
  - 如果不绑定其他执行器，则本执行器是线程安全的。如果绑定其他执行器，则线程安全性与绑定的执行器一致。
  - 如果绑定的执行器不存在，则会在初始化时抛出一个异常。
- `dt_us`是时间轮算法的一个参数，表示 Tick 的间隔。间隔越大，定时调度的精度越低，但越节省 CPU 资源。
- `wheel_size`是时间轮算法的另一个参数，表示各个时间轮的大小。比如默认的参数`[1000, 600]`表示有两个时间轮，第一个轮的刻度是 1000，第二个轮的刻度是 600。如果 Tick 时间是 1ms，则第一个轮的完整时间是 1s，第二个轮的完整时间是 10min。一般来说，要让可能的定时时间都落在轮内最好。
- `thread_sched_policy`和`thread_bind_cpu`参考[Common Information](./common.md)中线程绑核配置的说明。
- `use_system_clock`配置是否使用 std::system_clock 作为时间系统，默认为 false，使用 std::steady_clock。注意使用 std::system_clock 时，执行器的时间将与系统同步，可能会受到外部调节。

以下是一个简单的示例：
```yaml
aimrt:
  executor:
    executors:
      - name: test_tbb_thread_executor
        type: tbb_thread
        options:
          thread_num: 2
      - name: test_time_wheel_executor
        type: time_wheel
        options:
          bind_executor: test_tbb_thread_executor
          dt_us: 1000
          wheel_size: [1000, 600]
          thread_sched_policy: SCHED_FIFO:80
          thread_bind_cpu: [0]
```

