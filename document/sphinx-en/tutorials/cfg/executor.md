# aimrt.executor

## Configuration Overview

The `aimrt.executor` configuration item is used to configure executors. The detailed configuration items are described below:

| Node                    | Type      | Optional | Default Value | Purpose |
| ----                    | ----      | ----     | ----          | ----    |
| executors               | array     | Optional | []            | List of executors |
| executors[i].name       | string    | Required | ""            | Executor name |
| executors[i].type       | string    | Required | ""            | Executor type |
| executors[i].options    | map       | Optional | -             | Specific executor configuration |

Configuration notes for `aimrt.executor`:
- `executors` is an array used to configure various executors.
  - `executors[i].name` represents the executor name. Duplicate executor names are not allowed.
  - `executors[i].type` represents the executor type. AimRT officially provides several executor types, and some plugins also offer additional executor types.
  - `executors[i].options` contains initialization parameters passed by AimRT to each executor. The configuration format is defined by each executor type. Please refer to the corresponding executor type's documentation section.

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

The `simple_thread` executor is a simple single-threaded executor that does not support scheduled tasks. All its configuration items are as follows:

| Node                          | Type                  | Optional | Default Value | Purpose |
| ----                          | ----                  | ----     | ----          | ----    |
| thread_sched_policy           | string                | Optional | ""            | Thread scheduling policy |
| thread_bind_cpu               | unsigned int array    | Optional | []            | CPU binding configuration |
| queue_threshold               | unsigned int          | Optional | 10000         | Task queue threshold |

Usage notes:
- For `thread_sched_policy` and `thread_bind_cpu`, refer to the thread binding configuration in [Common Information](./common.md).
- `queue_threshold` configures the maximum number of tasks in the queue. When the number of queued tasks exceeds this threshold, new task submissions will fail.

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

The `asio_thread` executor is implemented based on the [Asio library](https://github.com/chriskohlhoff/asio). It is a thread pool that allows manual thread count configuration and supports scheduled tasks. All its configuration items are as follows:

| Node                          | Type                  | Optional | Default Value | Purpose |
| ----                          | ----                  | ----     | ----          | ----    |
| thread_num                    | unsigned int          | Optional | 1             | Number of threads |
| thread_sched_policy           | string                | Optional | ""            | Thread scheduling policy |
| thread_bind_cpu               | unsigned int array    | Optional | []            | CPU binding configuration |
| timeout_alarm_threshold_us    | unsigned int          | Optional | 1000000       | Scheduling timeout alarm threshold (microseconds) |
| use_system_clock              | bool                  | Optional | false         | Whether to use std::system_clock (default uses std::steady_clock) |

Usage notes:
- `thread_num` configures the number of threads, defaulting to 1. When set to 1, it is a thread-safe executor; otherwise, it is thread-unsafe.
- For `thread_sched_policy` and `thread_bind_cpu`, refer to the thread binding configuration in [Common Information](./common.md).
- `timeout_alarm_threshold_us` sets a threshold for scheduling timeout alarms. If CPU load is too high or the task queue is too full, causing scheduled tasks to be delayed beyond this threshold, a warning log will be printed.
- `use_system_clock` determines whether to use std::system_clock as the time system (default is false, using std::steady_clock). Note that when using std::system_clock, the executor's time will synchronize with the system and may be affected by external adjustments.

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

The `asio_strand` executor is a pseudo-executor that depends on the `asio_thread` executor, implemented using Asio's strand. It cannot exist independently and does not own actual threads. During operation, it delegates tasks to the bound `asio_thread` executor for execution. However, it guarantees thread safety and supports scheduled tasks. All its configuration items are as follows:

| Node                              | Type          | Optional | Default Value | Purpose |
| ----                              | ----          | ----     | ----          | ----    |
| bind_asio_thread_executor_name    | string        | Required | ""            | Name of the bound asio_thread executor |
| timeout_alarm_threshold_us        | unsigned int  | Optional | 1000000       | Scheduling timeout alarm threshold (microseconds) |
| use_system_clock                  | bool          | Optional | false         | Whether to use std::system_clock (default uses std::steady_clock) |

Usage notes:
- The `bind_asio_thread_executor_name` configuration item binds to an `asio_thread` type executor. If the specified executor does not exist or is not of type `asio_thread`, an exception will be thrown during initialization.
- `timeout_alarm_threshold_us` sets a threshold for scheduling timeout alarms. If CPU load is too high or the task queue is too full, causing scheduled tasks to be delayed beyond this threshold, a warning log will be printed.
- `use_system_clock` determines whether to use std::system_clock as the time system (default is false, using std::steady_clock). Note that when using std::system_clock, the executor's time will synchronize with the system and may be affected by external adjustments.

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

The `tbb_thread` executor is a high-performance lock-free thread pool implemented using the [oneTBB library](https://github.com/oneapi-src/oneTBB). It allows manual thread count configuration but does not support scheduled tasks. All its configuration items are as follows:

| Node                          | Type                  | Optional | Default Value | Purpose |
| ----                          | ----                  | ----     | ----          | ----    |
| thread_num                    | unsigned int          | Optional | 1             | Number of threads |
| thread_sched_policy           | string                | Optional | ""            | Thread scheduling policy |
| thread_bind_cpu               | unsigned int array    | Optional | []            | CPU binding configuration |
| timeout_alarm_threshold_us    | unsigned int          | Optional | 1000000       | Scheduling timeout alarm threshold (microseconds) |
| queue_threshold               | unsigned int          | Optional | 10000         | Task queue threshold |

Usage notes:
- `thread_num` configures the number of threads, defaulting to 1. When set to 1, it is a thread-safe executor; otherwise, it is thread-unsafe.
- For `thread_sched_policy` and `thread_bind_cpu`, refer to the thread binding configuration in [Common Information](./common.md).
- `timeout_alarm_threshold_us` sets a threshold for scheduling timeout alarms. If CPU load is too high or the task queue is too full, causing scheduled tasks to be delayed beyond this threshold, a warning log will be printed.
- `queue_threshold` configures the maximum number of tasks in the queue. When the number of queued tasks exceeds this threshold, new task submissions will fail.

Here is a simple example:
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

The `time_wheel` executor is implemented based on a timing wheel, typically used in scenarios with numerous scheduled tasks that do not require high timing precision, such as RPC timeout handling. It starts a separate thread to run the timing wheel and can delegate tasks to other executors for execution. All its configuration items are as follows:

| Node                  | Type                  | Optional | Default Value | Purpose |
| ----                  | ----                  | ----     | ----          | ----    |
| bind_executor         | string                | Optional | ""            | Bound executor |
| dt_us                 | unsigned int          | Optional | 100000        | Timing wheel tick interval (microseconds) |
| wheel_size            | unsigned int array    | Optional | [100, 360]    | Size of each timing wheel |
| thread_sched_policy   | string                | Optional | ""            | Timing wheel thread scheduling policy |
| thread_bind_cpu       | unsigned int array    | Optional | []            | Timing wheel thread CPU binding |
| use_system_clock      | bool                  | Optional | false         | Whether to use std::system_clock (default uses std::steady_clock) |

Usage notes:
- `bind_executor` configures the bound executor to delegate tasks for execution when their time arrives.
  - If no executor is bound, all tasks will execute in the timing wheel thread, potentially blocking the wheel's ticks.
  - Without a bound executor, this executor is thread-safe. With a bound executor, thread safety depends on the bound executor.
  - If the bound executor does not exist, an exception will be thrown during initialization.
- `dt_us` is a timing wheel algorithm parameter representing the tick interval. Larger intervals reduce scheduling precision but save CPU resources.
- `wheel_size` is another timing wheel algorithm parameter representing the size of each wheel. For example, the default `[1000, 600]` means two wheels with 1000 and 600 ticks respectively. With a 1ms tick, the first wheel covers 1s, and the second covers 10 minutes. Generally, ensure all possible timing falls within the wheels.
- For `thread_sched_policy` and `thread_bind_cpu`, refer to the thread binding configuration in [Common Information](./common.md).
- `use_system_clock` determines whether to use std::system_clock as the time system (default is false, using std::steady_clock). Note that when using std::system_clock, the executor's time will synchronize with the system and may be affected by external adjustments.

Here is a simple example:
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