# Executor


## Related Links

### Basic Executor Interface

Code files:
- {{ '[aimrt_module_cpp_interface/executor/executor_manager.h]({}/src/interface/aimrt_module_cpp_interface/executor/executor_manager.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/executor/executor.h]({}/src/interface/aimrt_module_cpp_interface/executor/executor.h)'.format(code_site_root_path_url) }}

Reference examples:
- {{ '[executor_module.cc]({}/src/examples/cpp/executor/module/executor_module/executor_module.cc)'.format(code_site_root_path_url) }}


### Coroutine Interface

Code files:
- {{ '[aimrt_module_cpp_interface/co/aimrt_context.h]({}/src/interface/aimrt_module_cpp_interface/co/aimrt_context.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/async_scope.h]({}/src/interface/aimrt_module_cpp_interface/co/async_scope.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/inline_scheduler.h]({}/src/interface/aimrt_module_cpp_interface/co/inline_scheduler.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/on.h]({}/src/interface/aimrt_module_cpp_interface/co/on.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/schedule.h]({}/src/interface/aimrt_module_cpp_interface/co/schedule.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/sync_wait.h]({}/src/interface/aimrt_module_cpp_interface/co/sync_wait.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/task.h]({}/src/interface/aimrt_module_cpp_interface/co/task.h)'.format(code_site_root_path_url) }}

Reference examples:
- {{ '[executor_co_module.cc]({}/src/examples/cpp/executor/module/executor_co_module/executor_co_module.cc)'.format(code_site_root_path_url) }}
- {{ '[executor_co_loop_module.cc]({}/src/examples/cpp/executor/module/executor_co_loop_module/executor_co_loop_module.cc)'.format(code_site_root_path_url) }}

## Concept of Executor


The executor is a long-standing concept that represents an abstract entity capable of executing logical code. An executor can be a thread pool, a coroutine/fiber, a CPU, a GPU, or even a remote server. The simplest code we usually write also has a default executor: the main thread.


Generally, executors usually have two similar interfaces:

```cpp
void Execute(std::function<void()>);
bool Execute(util::DynamicLatch&, std::function<void()>);
```


The first interface allows you to submit a task closure similar to `std::function<void()>` to the specified executor for execution. When and where this task is executed depends on the specific executor implementation. The C++ standard library’s `std::thread` is a typical example of an executor: its constructor accepts a `std::function<void()>` task closure and executes the task in a new thread.

The second interface takes a parameter of type `util::DynamicLatch&`, which is used to perform atomic counting and lifecycle management of "in-flight" tasks at the time they are posted:

- When `Execute(latch, task)` is called, the executor will first attempt to call `TryAdd()` on the latch; if it returns `false`, it indicates the latch has been closed (no new tasks are accepted), the current task will not be submitted, and the function returns `false`.
- When the task is actually completed, the executor will automatically call `latch.CountDown()` once, so there is no need for the user to manually count down within the task body.
- If you want to "stop accepting new tasks and wait for all submitted tasks to complete execution", you can sequentially call latch.Close() (to stop accepting new tasks) and latch.Wait() (to block and wait until the task count reaches zero) in the control thread, or directly use CloseAndWait() (a simplified combination of the above operations).

Note: If an exception may be thrown within your task, please catch it yourself to ensure that `CountDown()` is still called; it is also recommended that you invoke `Close() + Wait()` from the control/management thread, to avoid deadlocks caused by waiting in paths still holding the task count.

In AimRT, modules can obtain an `aimrt::configurator::ExecutorManagerRef` handle by calling the `GetExecutorManager()` interface of the `CoreRef` handle, which provides a simple interface for acquiring an Executor.

## Overview of the Basic Executor Interface

```cpp
namespace aimrt::executor {

class ExecutorManagerRef {
 public:
  ExecutorRef GetExecutor(std::string_view executor_name) const;
};

}  // namespace aimrt::executor
```


Users can call the `GetExecutor` method of the `ExecutorManagerRef` type to obtain an `aimrt::configurator::ExecutorRef` handle with a specified name, in order to invoke executor-related functionality. The core interface of `ExecutorRef` is as follows:


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

  bool Execute(util::DynamicLatch& latch_, Task&& task);

  std::chrono::system_clock::time_point Now() const;

  void ExecuteAt(std::chrono::system_clock::time_point tp, Task&& task) const;

  void ExecuteAfter(std::chrono::nanoseconds dt, Task&& task) const;
};

}  // namespace aimrt::executor
```


Executors in AimRT have some inherent attributes, most of which are related to the **executor type** and do not change during runtime. These inherent attributes include:
- **Executor Type**: A string field that identifies the type of the executor at runtime.
  - In a single AimRT instance, there can be multiple types of executors. AimRT officially provides several executors, and plugins can also provide new executor types.
  - For details on specific executor types and their characteristics, please refer to the `executor` configuration chapter in the deployment section.
  - During logical development, you should not focus too much on the actual runtime executor type; instead, implement business logic based on the abstract executor interface.
- **Executor Name**: A string field that identifies the name of the executor at runtime.
  - Within an AimRT process, the name uniquely identifies an executor.
  - All executor instance names are determined at runtime through configuration. For details, please refer to the `executor` configuration chapter in the deployment section.
  - You can obtain an executor with a specified name via the `GetExecutor` method of `ExecutorManagerRef`.
- **Thread Safety**: A boolean value indicating whether this executor is thread-safe.
  - Usually related to the executor type.
  - A thread-safe executor can guarantee that tasks posted to it will not run concurrently; otherwise, no such guarantee exists.
- **Support for Time-based Scheduling**: A boolean value indicating whether this executor supports time-based scheduling interfaces, namely the `ExecuteAt` and `ExecuteAfter` interfaces.
  - If the executor does not support time-based scheduling, calling the `ExecuteAt` or `ExecuteAfter` interfaces will throw an exception.


Detailed usage instructions for the `ExecutorRef` interface are as follows:
- `std::string_view Type()`: Gets the type of the executor.
- `std::string_view Name()`: Gets the name of the executor.
- `bool ThreadSafe()`: Returns whether this executor is thread-safe.
- `bool IsInCurrentExecutor()`: Determines whether the current environment is within this executor when this function is called.
  - Note: If it returns true, the current environment is definitely within this executor; if it returns false, the current environment may or may not be within this executor.
- `bool SupportTimerSchedule()`: Returns whether this executor supports time-based scheduling interfaces, namely the `ExecuteAt` and `ExecuteAfter` interfaces.
- `void Execute(Task&& task)`: Posts a task to this executor for immediate execution upon scheduling.
  - The parameter `Task` can simply be viewed as a task closure that satisfies the `std::function<void()>` signature.
  - This interface can be called during the Initialize/Start phase, but the executor only guarantees to start execution after the Start phase. Therefore, calling this interface before the Start phase may only enqueue the task into the executor's task queue without immediate execution, and the task will start executing only after the Start phase begins.
- `bool Execute(util::DynamicLatch& latch_, Task&& task)`: Submits a task with a dynamic latch.
  - Returns `true` if the task was accepted and dispatched; returns `false` if the latch is closed (no new tasks are accepted), and the task will not be dispatched.
  - After the task is completed, `latch_.CountDown()` will be called automatically; the caller does not need to manually call it inside the task body.
  - Can be combined with `latch_.Close(); latch_.Wait();` to smoothly shut down by stopping new task acceptance and waiting for all in-flight tasks to finish.
- `std::chrono::system_clock::time_point Now()`: Gets the time in this executor's time system.
  - For general executors, this returns the result of `std::chrono::system_clock::now()`.
  - Some special executors with time-scaling functionality may return processed time here.
- `void ExecuteAt(std::chrono::system_clock::time_point tp, Task&& task)`: Executes a task at a specific time point.
  - The first parameter—time point—is based on this executor's time system.
  - The second parameter `Task` can simply be viewed as a task closure that satisfies the `std::function<void()>` signature.
  - If this executor does not support time-based scheduling, calling this interface will throw an exception.
  - This interface can be called during the Initialize/Start phase, but the executor only guarantees to start execution after the Start phase. Therefore, calling this interface before the Start phase may only enqueue the task into the executor's task queue without immediate execution, and the task will start executing only after the Start phase begins.
- `void ExecuteAfter(std::chrono::nanoseconds dt, Task&& task)`: Executes a task after a certain duration.
  - The first parameter—duration—is based on this executor's time system.
  - The second parameter `Task` can simply be viewed as a task closure that satisfies the `std::function<void()>` signature.
  - If this executor does not support time-based scheduling, calling this interface will throw an exception.
  - This interface can be called during the Initialize/Start phase, but the executor only guarantees to start execution after the Start phase. Therefore, calling this interface before the Start phase may only enqueue the task into the executor's task queue without immediate execution, and the task will start executing only after the Start phase begins.## Basic Executor Interface Usage Example


## Basic Executor Interface Usage Example

The following is a simple usage example demonstrating how to obtain an executor handle and dispatch a simple task to that executor for execution:

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

If it is a thread-safe executor, then tasks dispatched to it do not need to be locked to ensure thread safety, as shown in the example below:

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

### Example: Submitting Tasks with a Latch

```cpp
auto exec = core_.GetExecutorManager().GetExecutor("work_executor");
aimrt::util::DynamicLatch latch;

for (int i = 0; i < 1000; ++i) {
  bool accepted = exec.Execute(latch, [i]() noexcept {
    // Task body
  });
  if (!accepted) {
    break;
  }
}


// It is recommended to directly use CloseAndWait for convenience and efficiency
latch.CloseAndWait();

// If you prefer step-by-step, you can Close first and then Wait
// latch.Close();
// latch.Wait();
```

The following example demonstrates how to use the Time Schedule interface to implement a periodic loop:
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
    // Check shutdown flag
    if (!run_flag_) return;

    AIMRT_INFO("Loop count : {}", loop_count_++);

    // Execute itself after 1 second
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




## Executor Coroutine Interface Overview

In AimRT, a coroutine-style interface is encapsulated for the executor based on C++20 coroutines and the [libunifex library](https://github.com/facebookexperimental/libunifex), providing a relatively important class: `aimrt::co::AimRTScheduler`, which can be constructed from an `aimrt::executor::ExecutorRef` handle. This class wraps the native AimRT executor handle into a coroutine form, with its core interfaces as follows:


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

With the `AimRTScheduler` handle, you can use a series of coroutine tools under the `aimrt::co` namespace. The following is a simple usage example demonstrating how to start a coroutine and schedule tasks to a specified executor within the coroutine:

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
    run_flag_ = false;    // Blocked waiting for all coroutines in the scope to complete execution
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

## Timer Based on Executor

### Timer Interface

Code files:
- {{ '[aimrt_module_cpp_interface/executor/timer.h]({}/src/interface/aimrt_module_cpp_interface/executor/timer.h)'.format(code_site_root_path_url) }}

Reference example:
- {{ '[timer_module.cc]({}/src/examples/cpp/executor/module/timer_module/timer_module.cc)'.format(code_site_root_path_url) }}

### Timer Concept

A timer is a tool provided by the executor for scheduling tasks at fixed intervals. You can create a timer based on an executor and specify the execution period.

### Timer Interface

Use the `aimrt::executor::CreateTimer` interface to create a timer and specify its execution period and task. The function declaration is as follows:


```cpp
namespace aimrt::executor {

template <typename TaskType>
std::shared_ptr<TimerBase> CreateTimer(ExecutorRef executor, std::chrono::nanoseconds period,
                                       TaskType&& task, bool auto_start = true);

}  // namespace aimrt::executor
```


Where `ExecutorRef` is the executor handle, `TaskType` is the task type, `period` is the timer execution period, and `auto_start` indicates whether to start the timer automatically, defaulting to `true`.

The `ExecutorRef` used by the timer must support timer scheduling, i.e., `SupportTimerSchedule()` returns `true`. Refer to the [Executor Configuration](../cfg/executor.md) chapter to check whether the executor supports timer scheduling.

`TaskType` is the task type, accepting a callable object. You can use `std::function`, `std::bind`, lambda expressions, etc., as long as its function signature meets one of the following requirements:


```cpp
void()
void(TimerBase&)
void(const TimerBase&)
```


In the function signature, `TimerBase&` is the timer object itself, and `const TimerBase&` is a const reference to the timer object.

`TimerBase` is the base class of the timer object, and `Timer` is a derived class of the timer object, mainly encapsulating the execution of the user-specified timer task. We generally use the smart pointer type of `TimerBase`: `std::shared_ptr<TimerBase>`.

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


Detailed usage instructions for the interfaces in the `TimerBase` class are as follows:
- `void Cancel()`: Cancels the timer and sets the cancel state.
- `void Reset()`: Resets the timer, cancels the cancel state, and resets the next execution time. The next execution time is calculated based on the current time plus the period.
- `void ExecuteTask()`: Executes the timer task.
- `bool IsCancelled()`: Returns whether the timer has been cancelled.
- `std::chrono::nanoseconds Period()`: Returns the timer execution period.
- `std::chrono::system_clock::time_point NextCallTime()`: Returns the next execution time of the timer.
- `std::chrono::nanoseconds TimeUntilNextCall()`: Returns the time difference between the next execution time and the current time.
- `ExecutorRef Executor()`: Returns the executor to which the timer belongs.

### Timer Behavior Overview

The behavior of the timer is as follows:
- After the timer is created, it is automatically started by default, equivalent to automatically calling the `Reset()` interface. If you do not want it to start automatically, you can set `auto_start` to `false`, in which case the timer will be in the `cancel` state.
- Regardless of whether the timer is started or not, calling the `Cancel()` interface will cancel the timer and set the cancel state.
- Regardless of whether the timer is started or not, calling the `Reset()` interface will reset the timer, cancel the cancel state, and reset the next execution time. The next execution time is calculated based on the current time plus the period.
- The `Reset()` interface can override the original timer task. After calling the `Reset()` interface, calling the `Reset()` interface again will reschedule the task according to the new period, and the original timer task will be overridden by the new task.
- If the task execution time is too long or there are blocking operations in the executor used by the timer, causing some timer periods to be missed, the timer will not make up for the missed executions. Instead, it will wait until the next execution time arrives to execute the task. For example:
  - Suppose the timer period is 1000 ms, and the task is expected to execute at 0, 1000, 2000, 3000, 4000, ... ms
  - Suppose the task execution time is 1500 ms, then the task started at 0 ms will complete at 1500 ms, missing the execution at 1000 ms
  - The timer will reset the next execution time to 2000 ms and execute the task at 2000 ms, without making up for the execution at 1000 ms
  - The final task execution start times will be: 0, 2000, 4000, 6000, ... ms### Timer Usage Example

Below is a simple usage example demonstrating how to create a timer and use it to execute a task:

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
