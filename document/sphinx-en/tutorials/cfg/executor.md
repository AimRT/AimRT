# aimrt.executor

## Configuration Overview

The `aimrt.executor` configuration item is used to configure executors. The detailed configuration items are described below:

| Node                 | Type   | Optional | Default | Description             |
| -------------------- | ------ | -------- | ------- | ----------------------- |
| executors            | array  | Optional | []      | List of executors       |
| executors[i].name    | string | Required | ""      | Executor name           |
| executors[i].type    | string | Required | ""      | Executor type           |
| executors[i].options | map    | Optional | -       | Specific executor config |

The configuration description for `aimrt.executor` is as follows:

- `executors` is an array used to configure individual executors.
  - `executors[i].name` indicates the executor name. Duplicate executor names are not allowed.
  - `executors[i].type` indicates the executor type. AimRT officially provides several executor types, and some plugins also provide additional executor types.
  - `executors[i].options` are initialization parameters passed by AimRT to each executor. The format of this configuration is defined by each executor type; please refer to the corresponding executor type documentation section.

Here is a simple example:


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


## simple_thread Executor

The `simple_thread` executor is a simple single-thread executor that does not support timed scheduling. All its configuration items are as follows:

| Node                        | Type               | Optional | Default | Description                                                  |
| --------------------------- | ------------------ | -------- | ------- | ------------------------------------------------------------ |
| thread_sched_policy         | string             | Optional | ""      | Thread scheduling policy                                     |
| thread_bind_cpu             | unsigned int array | Optional | []      | CPU binding configuration                                    |
| queue_threshold             | unsigned int       | Optional | 10000   | Queue task upper limit                                       |
| threshold_alarm_interval_ms | int                | Optional | 1000    | Thread usage threshold alarm interval, unit: milliseconds    |

Usage notes:

- Refer to the [Common Information](./common.md) section for `thread_sched_policy` and `thread_bind_cpu` regarding CPU binding configuration.
- `queue_threshold` sets the upper limit of tasks in the queue. When the number of tasks in the queue exceeds this threshold, new tasks will fail to be submitted.
- `threshold_alarm_interval_ms` sets the thread usage threshold alarm interval. When thread usage exceeds the threshold, a warning log will be printed at regular intervals. If set to a negative value, this log printing is disabled.

Here is a simple example:


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
## asio_thread Executor

The `asio_thread` executor is an executor implemented based on the [Asio library](https://github.com/chriskohlhoff/asio). It is a thread pool that allows manual configuration of the number of threads and also supports timed scheduling. All of its configuration items are as follows:

| Node                       | Type               | Optional | Default | Description                                                   |
| -------------------------- | ------------------ | -------- | ------- | ------------------------------------------------------------- |
| thread_num                 | unsigned int       | Optional | 1       | Number of threads                                             |
| thread_sched_policy        | string             | Optional | ""      | Thread scheduling policy                                      |
| thread_bind_cpu            | unsigned int array | Optional | []      | CPU binding configuration                                     |
| timeout_alarm_threshold_us | unsigned int       | Optional | 1000000 | Scheduling timeout alarm threshold, in microseconds           |
| use_system_clock           | bool               | Optional | false   | Whether to use std::system_clock, defaults to std::steady_clock |
| timeout_alarm_interval_ms  | int                | Optional | 1000    | Thread timeout alarm interval, in milliseconds                |

Usage notes:

- `thread_num` configures the number of threads, defaulting to 1. When the thread count is set to 1, it is a thread-safe executor; otherwise, it is thread-unsafe.
- `thread_sched_policy` and `thread_bind_cpu` refer to the thread binding configuration instructions in [Common Information](./common.md).
- `timeout_alarm_threshold_us` sets a scheduling timeout alarm threshold. When performing timed scheduling, if CPU load is too high or there are too many tasks in the queue, causing scheduling to exceed the set time, an alarm log will be printed.
- `use_system_clock` configures whether to use std::system_clock as the time system, defaulting to false (using std::steady_clock). Note that when using std::system_clock, the executor's time will synchronize with the system and may be affected by external adjustments.
- `timeout_alarm_interval_ms` configures the thread timeout alarm interval. When a thread timeout exceeds the threshold, an alarm log will be printed at regular intervals. If set to a negative value, this log printing will be disabled.

Here is a simple example:


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


## asio_strand Executor

The `asio_strand` executor is a pseudo-executor attached to the `asio_thread` executor, implemented based on Asio's strand. It cannot exist independently and does not own actual threads; during operation, it delegates tasks to the bound `asio_thread` executor for actual execution. However, it guarantees thread safety and also supports timed scheduling. All of its configuration items are as follows:

| Node                           | Type         | Optional | Default | Description                                                   |
| ------------------------------ | ------------ | -------- | ------- | ------------------------------------------------------------- |
| bind_asio_thread_executor_name | string       | Required | ""      | Name of the bound asio_thread executor                        |
| timeout_alarm_threshold_us     | unsigned int | Optional | 1000000 | Scheduling timeout alarm threshold, in microseconds           |
| use_system_clock               | bool         | Optional | false   | Whether to use std::system_clock, defaults to std::steady_clock |
| timeout_alarm_interval_ms      | int          | Optional | 1000    | Thread timeout alarm interval, in milliseconds                |

Usage notes:

- Use the `bind_asio_thread_executor_name` configuration item to bind an executor of type `asio_thread`. If the specified executor does not exist or is not of type `asio_thread`, an exception will be thrown during initialization.
- `timeout_alarm_threshold_us` sets a scheduling timeout alarm threshold. When performing timed scheduling, if CPU load is too high or there are too many tasks in the queue, causing scheduling to exceed the set time, an alarm log will be printed.
- `use_system_clock` configures whether to use std::system_clock as the time system, defaulting to false (using std::steady_clock). Note that when using std::system_clock, the executor's time will synchronize with the system and may be affected by external adjustments.
- `timeout_alarm_interval_ms` configures the thread timeout alarm interval. When a thread timeout exceeds the threshold, an alarm log will be printed at regular intervals. If set to a negative value, this log printing will be disabled.

Here is a simple example:


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
## tbb_thread Executor

`tbb_thread` is a high-performance lock-free thread pool implemented with a lock-free concurrent queue based on the [oneTBB library](https://github.com/oneapi-src/oneTBB). You can manually set the number of threads, but it does not support timed scheduling. All its configuration items are as follows:

| Node                        | Type               | Optional | Default | Description                                                                 |
| --------------------------- | ------------------ | -------- | ------- | --------------------------------------------------------------------------- |
| thread_num                  | unsigned int       | Optional | 1       | Number of threads                                                           |
| thread_sched_policy         | string             | Optional | ""      | Thread scheduling policy                                                    |
| thread_bind_cpu             | unsigned int array | Optional | []      | CPU binding configuration                                                   |
| timeout_alarm_threshold_us  | unsigned int       | Optional | 1000000 | Scheduling timeout alarm threshold, unit: microseconds                      |
| queue_threshold             | unsigned int       | Optional | 10000   | Upper limit of queued tasks                                                 |
| threshold_alarm_interval_ms | int                | Optional | 1000    | Thread utilization threshold alarm interval, unit: milliseconds             |

Usage notes:

- `thread_num` configures the number of threads, defaulting to 1. When set to 1, it is a thread-safe executor; otherwise, it is not thread-safe.
- `thread_sched_policy` and `thread_bind_cpu` refer to the thread binding configuration description in [Common Information](./common.md).
- `timeout_alarm_threshold_us` sets a scheduling timeout alarm threshold. When performing timed scheduling, if CPU load is too heavy or there are too many tasks in the queue, causing scheduling to exceed the set time, an alarm log will be printed.
- `queue_threshold` sets the upper limit of queued tasks. When the number of tasks in the queue exceeds this threshold, new tasks will fail to be submitted.
- `threshold_alarm_interval_ms` sets the interval for thread utilization threshold alarms. When thread utilization exceeds the threshold, an alarm log will be printed at regular intervals. If set to a negative value, this log printing is disabled.

Below is a simple example:


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


## time_wheel Executor

The `time_wheel` executor is based on a time-wheel implementation, generally used for scenarios with a large number of timed tasks and low precision requirements, such as RPC timeout handling. It starts a dedicated thread to run the time wheel and also supports dispatching specific tasks to other executors for execution. All its configuration items are as follows:

| Node                | Type               | Optional | Default    | Description                                                                 |
| ------------------- | ------------------ | -------- | ---------- | --------------------------------------------------------------------------- |
| bind_executor       | string             | Optional | ""         | Bound executor                                                              |
| dt_us               | unsigned int       | Optional | 100000     | Time-wheel tick interval, unit: microseconds                                |
| wheel_size          | unsigned int array | Optional | [100, 360] | Sizes of each time wheel                                                    |
| thread_sched_policy | string             | Optional | ""         | Scheduling policy of the time-wheel thread                                  |
| thread_bind_cpu     | unsigned int array | Optional | []         | CPU binding configuration of the time-wheel thread                          |
| use_system_clock    | bool               | Optional | false      | Whether to use std::system_clock; defaults to std::steady_clock             |

Usage notes:

- `bind_executor` is used to configure the bound executor, so that when the time arrives, tasks are dispatched to the bound executor for actual execution.
  - If no other executor is bound, all tasks will execute within the time-wheel thread, potentially blocking the time-wheel tick.
  - If no other executor is bound, this executor is thread-safe. If another executor is bound, thread safety is consistent with the bound executor.
  - If the bound executor does not exist, an exception will be thrown during initialization.
- `dt_us` is a parameter of the time-wheel algorithm, representing the tick interval. The larger the interval, the lower the timing precision, but the more CPU resources are saved.
- `wheel_size` is another parameter of the time-wheel algorithm, representing the sizes of each time wheel. For example, the default parameter `[1000, 600]` indicates two time wheels: the first wheel has 1000 ticks, and the second has 600 ticks. If the tick interval is 1 ms, the first wheel covers 1 s, and the second wheel covers 10 min. Generally, it is best to ensure that all possible timeout durations fall within the wheels.
- `thread_sched_policy` and `thread_bind_cpu` refer to the thread binding configuration description in [Common Information](./common.md).
- `use_system_clock` configures whether to use std::system_clock as the time source, defaulting to false (using std::steady_clock). Note that when using std::system_clock, the executor's time will synchronize with the system and may be affected by external adjustments.

Below is a simple example:


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
