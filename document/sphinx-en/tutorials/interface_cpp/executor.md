# Executor

## Related Links

### Basic Executor Interface

Code Files:
- {{ '[aimrt_module_cpp_interface/executor/executor_manager.h]({}/src/interface/aimrt_module_cpp_interface/executor/executor_manager.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/executor/executor.h]({}/src/interface/aimrt_module_cpp_interface/executor/executor.h)'.format(code_site_root_path_url) }}

Reference Examples:
- {{ '[executor_module.cc]({}/src/examples/cpp/executor/module/executor_module/executor_module.cc)'.format(code_site_root_path_url) }}

### Coroutine Interface

Code Files:
- {{ '[aimrt_module_cpp_interface/co/aimrt_context.h]({}/src/interface/aimrt_module_cpp_interface/co/aimrt_context.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/async_scope.h]({}/src/interface/aimrt_module_cpp_interface/co/async_scope.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/inline_scheduler.h]({}/src/interface/aimrt_module_cpp_interface/co/inline_scheduler.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/on.h]({}/src/interface/aimrt_module_cpp_interface/co/on.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/schedule.h]({}/src/interface/aimrt_module_cpp_interface/co/schedule.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/sync_wait.h]({}/src/interface/aimrt_module_cpp_interface/co/sync_wait.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/task.h]({}/src/interface/aimrt_module_cpp_interface/co/task.h)'.format(code_site_root_path_url) }}

Reference Examples:
- {{ '[executor_co_module.cc]({}/src/examples/cpp/executor/module/executor_co_module/executor_co_module.cc)'.format(code_site_root_path_url) }}
- {{ '[executor_co_loop_module.cc]({}/src/examples/cpp/executor/module/executor_co_loop_module/executor_co_loop_module.cc)'.format(code_site_root_path_url) }}

## The Concept of Executor

An executor is a long-standing concept that represents an abstraction capable of executing logical code. An executor can be a thread pool, a coroutine/fiber, a CPU, GPU, or even a remote server. The simplest code we write daily also has a default executor: the main thread.

Generally, executors will have an interface similar to this:
```cpp
void Execute(std::function<void()>);
```

This interface indicates that a task closure similar to `std::function<void()>` can be submitted to a specified executor for execution. When and where this task executes depends on the specific implementation of the executor. The `std::thread` in the C++ standard library is a typical executor - its constructor accepts a `std::function<void()>` task closure and executes that task in a new thread.## Overview of Basic Executor Interface

In AimRT, modules can obtain the `aimrt::configurator::ExecutorManagerRef` handle by calling the `GetExecutorManager()` interface of the `CoreRef` handle, which provides a simple interface for acquiring Executors:

```cpp
namespace aimrt::executor {

class ExecutorManagerRef {
 public:
  ExecutorRef GetExecutor(std::string_view executor_name) const;
};

}  // namespace aimrt::executor
```

Users can call the `GetExecutor` method of the `ExecutorManagerRef` type to obtain an `aimrt::configurator::ExecutorRef` handle with a specified name, enabling access to executor-related functionalities. The core interfaces of `ExecutorRef` are as follows:

```cpp
namespace aimrt::executor {

class ExecutorRef {
 public:
  std::string_view Type() const;

  std::string_view Name() const;

  bool ThreadSafe() const;

  bool IsInCurrentExecutor() const;

  bool SupportTimerSchedule() const;

  void Execute(Task&& task) const;

  std::chrono::system_clock::time_point Now() const;

  void ExecuteAt(std::chrono::system_clock::time_point tp, Task&& task) const;

  void ExecuteAfter(std::chrono::nanoseconds dt, Task&& task) const;
};

}  // namespace aimrt::executor
```

Executors in AimRT have some inherent attributes, most of which are related to the **executor type** and remain unchanged during runtime. These inherent attributes include:
- **Executor Type**: A string field identifying the executor's type during runtime.
  - An AimRT instance may contain multiple types of executors. AimRT officially provides several executors, and plugins can also introduce new types.
  - For specific executor types and their characteristics, refer to the `executor` configuration section in the deployment documentation.
  - During logical development, focus should not be overly placed on the actual runtime executor type; instead, business logic should be implemented based on the abstract executor interface.
- **Executor Name**: A string field identifying the executor's name during runtime.
  - Within an AimRT process, the name uniquely identifies an executor.
  - All executor instance names are determined at runtime through configuration. For details, refer to the `executor` configuration section in the deployment documentation.
  - The `GetExecutor` method of `ExecutorManagerRef` can be used to retrieve an executor with a specified name.
- **Thread Safety**: A boolean value indicating whether the executor is thread-safe.
  - Typically related to the executor type.
  - Thread-safe executors ensure that tasks submitted to them will not run concurrently; otherwise, no such guarantee is provided.
- **Supports Timed Scheduling**: A boolean value indicating whether the executor supports timed scheduling interfaces, namely `ExecuteAt` and `ExecuteAfter`.
  - If the executor does not support timed scheduling, calling `ExecuteAt` or `ExecuteAfter` will throw an exception.

Detailed usage instructions for the `ExecutorRef` interface are as follows:
- `std::string_view Type()`: Retrieves the executor's type.
- `std::string_view Name()`: Retrieves the executor's name.
- `bool ThreadSafe()`: Returns whether the executor is thread-safe.
- `bool IsInCurrentExecutor()`: Determines whether the current call is being made within this executor.
  - Note: If true, the current environment is definitely within this executor; if false, the current environment may or may not be within this executor.
- `bool SupportTimerSchedule()`: Returns whether the executor supports timed scheduling interfaces, i.e., `ExecuteAt` and `ExecuteAfter`.
- `void Execute(Task&& task)`: Submits a task to this executor for immediate execution upon scheduling.
  - The `Task` parameter can be simply viewed as a task closure that satisfies the `std::function<void()>` signature.
  - This interface can be called during the Initialize/Start phases, but the executor only guarantees task execution after the Start phase. Therefore, calling this interface before the Start phase may only queue the task in the executor's task queue without immediate execution, with execution commencing after the Start phase.
- `std::chrono::system_clock::time_point Now()`: Retrieves the current time within the executor's time system.
  - For most executors, this returns the result of `std::chrono::system_clock::now()`.
  - Special executors with time-adjustment capabilities may return processed time values.
- `void ExecuteAt(std::chrono::system_clock::time_point tp, Task&& task)`: Executes a task at a specified time point.
  - The first parameter, the time point, is based on the executor's time system.
  - The second parameter, `Task`, can be simply viewed as a task closure that satisfies the `std::function<void()>` signature.
  - If the executor does not support timed scheduling, calling this interface will throw an exception.
  - This interface can be called during the Initialize/Start phases, but the executor only guarantees task execution after the Start phase. Therefore, calling this interface before the Start phase may only queue the task in the executor's task queue without immediate execution, with execution commencing after the Start phase.
- `void ExecuteAfter(std::chrono::nanoseconds dt, Task&& task)`: Executes a task after a specified duration.
  - The first parameter, the duration, is based on the executor's time system.
  - The second parameter, `Task`, can be simply viewed as a task closure that satisfies the `std::function<void()>` signature.
  - If the executor does not support timed scheduling, calling this interface will throw an exception.
  - This interface can be called during the Initialize/Start phases, but the executor only guarantees task execution after the Start phase. Therefore, calling this interface before the Start phase may only queue the task in the executor's task queue without immediate execution, with execution commencing after the Start phase.## Basic Executor Interface Usage Example

Here is a simple usage example demonstrating how to obtain an executor handle and submit a simple task to the executor for execution:
```cpp
#include "aimrt_module_cpp_interface/module_base.h"

class HelloWorldModule : public aimrt::ModuleBase {
 public:
  bool Initialize(aimrt::CoreRef core) override {
    core_ = core;

    return true;
  }

  bool Start() override {
    // Get an executor handle named 'work_executor'
    auto work_executor = core_.GetExecutorManager().GetExecutor("work_executor");

    // Check
    AIMRT_CHECK_ERROR_THROW(work_executor, "Can not get work_executor");

    // Post a task to this executor
    work_executor.Execute([this]() {
      AIMRT_INFO("This is a simple task");
    });
  }

  // ...
 private:
  aimrt::CoreRef core_;
};
```

If it is a thread-safe executor, tasks submitted to it do not require locking to ensure thread safety, as shown in the following example:
```cpp
#include "aimrt_module_cpp_interface/module_base.h"

class HelloWorldModule : public aimrt::ModuleBase {
 public:
  bool Initialize(aimrt::CoreRef core) override {
    core_ = core;

    return true;
  }

  bool Start() override {
    // Get an executor handle named 'thread_safe_executor'
    auto thread_safe_executor = core_.GetExecutorManager().GetExecutor("thread_safe_executor");

    // Check
    AIMRT_CHECK_ERROR_THROW(thread_safe_executor && thread_safe_executor.ThreadSafe(),
                            "Can not get thread_safe_executor");

    // Post some tasks to this executor
    uint32_t n = 0;
    for (uint32_t ii = 0; ii < 10000; ++ii) {
      thread_safe_executor_.Execute([&n]() {
        n++;
      });
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));

    AIMRT_INFO("Value of n is {}", n);
  }

  // ...
 private:
  aimrt::CoreRef core_;
};
```

The following example demonstrates how to use the Time Schedule interface to implement timed loops:
```cpp
#include "aimrt_module_cpp_interface/module_base.h"

class HelloWorldModule : public aimrt::ModuleBase {
 public:
  bool Initialize(aimrt::CoreRef core) override {
    core_ = core;

    // Get an executor handle named 'time_schedule_executor'
    auto time_schedule_executor_ = core_.GetExecutorManager().GetExecutor("time_schedule_executor");

    // Check
    AIMRT_CHECK_ERROR_THROW(time_schedule_executor_ && time_schedule_executor_.SupportTimerSchedule(),
                            "Can not get time_schedule_executor");

    return true;
  }

  // Task
  void ExecutorModule::TimeScheduleDemo() {
    // Check shutdown
    if (!run_flag_) return;

    AIMRT_INFO("Loop count : {}", loop_count_++);

    // Execute itself
    time_schedule_executor_.ExecuteAfter(
        std::chrono::seconds(1),
        std::bind(&ExecutorModule::TimeScheduleDemo, this));
  }

  bool Start() override {
    TimeScheduleDemo();
  }

  void ExecutorModule::Shutdown() {
    run_flag_ = false;

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // ...

 private:
  aimrt::CoreRef core_;

  bool run_flag_ = true;
  uint32_t loop_count_ = 0;
  aimrt::executor::ExecutorRef time_schedule_executor_;
};
```



## Overview of Executor Coroutine Interface

In AimRT, a coroutine-style interface based on C++20 coroutines and the [libunifex library](https://github.com/facebookexperimental/libunifex) is encapsulated for executors. It provides an important class: `aimrt::co::AimRTScheduler`, which can be constructed from the `aimrt::executor::ExecutorRef` handle. This class wraps the native AimRT executor handle into a coroutine form, with its core interfaces as follows:

```cpp
namespace aimrt::co {

// Corresponding to ExecutorRef
class AimRTScheduler {
 public:
  explicit AimRTScheduler(executor::ExecutorRef executor_ref = {}) noexcept;
};

// Corresponding to ExecutorManagerRef
class AimRTContext {
 public:
  explicit AimRTContext(executor::ExecutorManagerRef executor_manager_ref = {}) noexcept;

  AimRTScheduler GetScheduler(std::string_view executor_name) const;
};

}  // namespace aimrt::co
```
## Executor Coroutine Interface Usage Example

With the `AimRTScheduler` handle, you can use a series of coroutine tools under the `aimrt::co` namespace. Here is a simple usage example demonstrating how to start a coroutine and schedule tasks to a specified executor within the coroutine:
```cpp
#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/co/inline_scheduler.h"
#include "aimrt_module_cpp_interface/co/on.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_cpp_interface/module_base.h"

class HelloWorldModule : public aimrt::ModuleBase {
 public:
  bool Initialize(aimrt::CoreRef core) override {
    core_ = core;

    // Get an executor handle named 'work_executor_1' and check
    work_executor_1_ = core_.GetExecutorManager().GetExecutor("work_executor_1");
    AIMRT_CHECK_ERROR_THROW(work_executor_1_, "Can not get work_executor_1");

    // Get an executor handle named 'work_executor_2' and check
    work_executor_2_ = core_.GetExecutorManager().GetExecutor("work_executor_2");
    AIMRT_CHECK_ERROR_THROW(work_executor_2_, "Can not get work_executor_2");

    return true;
  }

  bool Start() override {
    // Start a coroutine and use the current executor (main thread) to execute the coroutine
    scope_.spawn(co::On(co::InlineScheduler(), MyTask()));

    return true;
  }

  aimrt::co::Task<void> MyTask() {
    AIMRT_INFO("Now run in init executor");

    // Encapsulate the executor handle as the scheduler handle
    auto work_executor_1_scheduler = co::AimRTScheduler(work_executor_1_);

    // Schedule to work_executor_1_
    co_await aimrt::co::Schedule(work_executor_1_scheduler);

    AIMRT_INFO("Now run in work_executor_1_");

    // Encapsulate the executor handle as the scheduler handle
    auto work_executor_2_scheduler = co::AimRTScheduler(work_executor_2_);

    // Schedule to work_executor_2_
    co_await aimrt::co::Schedule(work_executor_2_scheduler);

    AIMRT_INFO("Now run in work_executor_2_");

    co_return;
  }

  void ExecutorCoModule::Shutdown() {
    // Blocked waiting for all coroutines in the scope to complete execution
    co::SyncWait(scope_.complete());

    AIMRT_INFO("Shutdown succeeded.");
  }

 private:
  aimrt::CoreRef core_;
  aimrt::co::AsyncScope scope_;

  aimrt::executor::ExecutorRef work_executor_1_;
  aimrt::executor::ExecutorRef work_executor_2_;
};
```

The following example demonstrates how to use the Time Schedule interface to implement a timed loop based on coroutines:
```cpp
#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/co/inline_scheduler.h"
#include "aimrt_module_cpp_interface/co/on.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_cpp_interface/module_base.h"

class HelloWorldModule : public aimrt::ModuleBase {
 public:
  bool Initialize(aimrt::CoreRef core) override {
    core_ = core;

    // Get an executor handle named 'time_schedule_executor' and check
    time_schedule_executor_ = core_.GetExecutorManager().GetExecutor("time_schedule_executor");
    AIMRT_CHECK_ERROR_THROW(time_schedule_executor_ && time_schedule_executor_.SupportTimerSchedule(),
                            "Can not get time_schedule_executor");

    return true;
  }

  bool Start() override {
    // Start a coroutine and use the current executor (main thread) to execute the coroutine
    scope_.spawn(co::On(co::InlineScheduler(), MainLoop()));

    return true;
  }

  aimrt::co::Task<void> MainLoop() {
    auto time_scheduler = co::AimRTScheduler(time_schedule_executor_);

    // Schedule to time_schedule_executor
    co_await co::Schedule(time_scheduler);

    uint32_t count = 0;
    while (run_flag_) {
      count++;
      AIMRT_INFO("Loop count : {} -------------------------", count);

      // Schedule to time_schedule_executor after some time. Equivalent to non blocking sleep
      co_await co::ScheduleAfter(time_scheduler, std::chrono::seconds(1));
    }

    AIMRT_INFO("Exit loop.");

    co_return;
  }

  void ExecutorCoModule::Shutdown() {
    run_flag_ = false;```cpp
    // Blocked waiting for all coroutines in the scope to complete execution
    co::SyncWait(scope_.complete());

    AIMRT_INFO("Shutdown succeeded.");
  }

 private:
  aimrt::CoreRef core_;
  aimrt::co::AsyncScope scope_;
  std::atomic_bool run_flag_ = true;
  aimrt::executor::ExecutorRef time_schedule_executor_;
};
```
## Executor-based Timer

### Timer Interface

Code files:
- {{ '[aimrt_module_cpp_interface/executor/timer.h]({}/src/interface/aimrt_module_cpp_interface/executor/timer.h)'.format(code_site_root_path_url) }}

Reference examples:
- {{ '[timer_module.cc]({}/src/examples/cpp/executor/module/timer_module/timer_module.cc)'.format(code_site_root_path_url) }}

### Timer Concept

The timer is a tool provided by the executor for periodically executing tasks. You can create a timer based on an executor and specify its execution interval.

### Timer Interface

Use the `aimrt::executor::CreateTimer` interface to create a timer and specify its execution interval and task. The function declaration is as follows:

```cpp
namespace aimrt::executor {

template <typename TaskType>
std::shared_ptr<TimerBase> CreateTimer(ExecutorRef executor, std::chrono::nanoseconds period,
                                       TaskType&& task, bool auto_start = true);

}  // namespace aimrt::executor
```

Where:
- `ExecutorRef` is the executor handle
- `TaskType` is the task type
- `period` is the timer's execution interval
- `auto_start` determines whether to automatically start the timer (default is `true`)

The `ExecutorRef` used by the timer must support timer scheduling functionality, i.e., `SupportTimerSchedule()` returns `true`. Refer to the [Executor Configuration](../cfg/executor.md) section to check whether an executor supports timer scheduling.

`TaskType` represents the task type and accepts any callable object, such as `std::function`, `std::bind`, or lambda expressions, as long as its function signature satisfies one of the following requirements:

```cpp
void()
void(TimerBase&)
void(const TimerBase&)
```

In the function signature:
- `TimerBase&` refers to the timer object itself
- `const TimerBase&` is a constant reference to the timer object

`TimerBase` is the base class for timer objects, while `Timer` is the derived class that primarily encapsulates the execution of user-specified timer tasks. We typically use the smart pointer type of `TimerBase`: `std::shared_ptr<TimerBase>`.

The core interfaces of `TimerBase` are as follows:

```cpp
class TimerBase {
 public:
  virtual void Reset() = 0;
  virtual void Cancel() = 0;
  virtual void ExecuteTask() = 0;

  [[nodiscard]] bool IsCancelled() const;
  [[nodiscard]] std::chrono::nanoseconds Period() const;
  [[nodiscard]] std::chrono::system_clock::time_point NextCallTime() const;
  [[nodiscard]] std::chrono::nanoseconds TimeUntilNextCall() const;
  [[nodiscard]] ExecutorRef Executor() const;
};
```

Detailed usage instructions for the `TimerBase` class interfaces:
- `void Cancel()`: Cancels the timer and sets the cancel state.
- `void Reset()`: Resets the timer, clears the cancel state, and recalculates the next execution time based on the current time plus the interval.
- `void ExecuteTask()`: Executes the timer task.
- `bool IsCancelled()`: Returns whether the timer has been canceled.
- `std::chrono::nanoseconds Period()`: Returns the timer's execution interval.
- `std::chrono::system_clock::time_point NextCallTime()`: Returns the next scheduled execution time of the timer.
- `std::chrono::nanoseconds TimeUntilNextCall()`: Returns the time difference between the next execution time and the current time.
- `ExecutorRef Executor()`: Returns the executor to which the timer belongs.

### Timer Behavior Overview

The timer behaves as follows:
- After creation, the timer automatically starts by default (equivalent to calling `Reset()` once). To disable auto-start, set `auto_start` to `false`, in which case the timer will remain in the `cancel` state.
- Regardless of whether the timer is started, calling `Cancel()` will cancel the timer and set the cancel state.
- Regardless of whether the timer is started, calling `Reset()` will reset the timer, clear the cancel state, and recalculate the next execution time based on the current time plus the interval.
- The `Reset()` interface can override previous timer tasks. If `Reset()` is called consecutively, the task will be re-executed according to the new interval, and the previous task will be replaced by the new one.
- If task execution takes too long or the executor used by the timer contains blocking operations that cause missed intervals, the timer will not compensate for the missed executions. Instead, it will wait until the next scheduled execution time. For example:
  - Suppose the timer interval is 1000 ms, with expected executions at 0, 1000, 2000, 3000, 4000, ... ms.
  - If a task takes 1500 ms to execute, the task started at 0 ms will complete at 1500 ms, missing the 1000 ms execution.
  - The timer will reset the next execution time to 2000 ms and execute the task at that time, without compensating for the missed 1000 ms execution.
  - The final task execution start times will be: 0, 2000, 4000, 6000, ... ms.### Timer Usage Example

Here is a simple usage example demonstrating how to create a timer and use it to execute a task:
```cpp
bool TimerModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  timer_executor_ = core_.GetExecutorManager().GetExecutor("timer_executor");
  AIMRT_CHECK_ERROR_THROW(timer_executor_, "Can not get timer_executor");
  AIMRT_CHECK_ERROR_THROW(timer_executor_.SupportTimerSchedule(),
                          "timer_executor does not support timer schedule");

  return true;
}

bool TimerModule::Start() {
  using namespace std::chrono_literals;

  auto start_time = timer_executor_.Now();
  auto task = [logger = core_.GetLogger(), start_time](aimrt::executor::TimerBase& timer) {
    static int count = 0;

    auto now = timer.Executor().Now();
    auto timepoint = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
    AIMRT_HL_INFO(logger, "Executed {} times, execute timepoint: {} ms", ++count, timepoint);

    if (count >= 10) {
      timer.Cancel();
      AIMRT_HL_INFO(logger, "Timer cancelled at timepoint: {} ms", timepoint);
    }
  };

  timer_ = aimrt::executor::CreateTimer(timer_executor_, 100ms, std::move(task));
  AIMRT_INFO("Timer created at timepoint: 0 ms");

  timer_executor_.ExecuteAfter(350ms, [this, logger = core_.GetLogger()]() {
    timer_->Reset();
    AIMRT_HL_INFO(logger, "Timer reset at timepoint: 350 ms");
  });

  timer_executor_.ExecuteAfter(600ms, [this, logger = core_.GetLogger()]() {
    timer_->Reset();
    AIMRT_HL_INFO(logger, "Timer reset at timepoint: 600 ms");
  });

  return true;
}

void TimerModule::Shutdown() { timer_->Cancel(); }
```