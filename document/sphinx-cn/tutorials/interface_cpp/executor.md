# Executor


## 相关链接

### 基本执行器接口

代码文件：
- {{ '[aimrt_module_cpp_interface/executor/executor_manager.h]({}/src/interface/aimrt_module_cpp_interface/executor/executor_manager.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/executor/executor.h]({}/src/interface/aimrt_module_cpp_interface/executor/executor.h)'.format(code_site_root_path_url) }}

参考示例：
- {{ '[executor_module.cc]({}/src/examples/cpp/executor/module/executor_module/executor_module.cc)'.format(code_site_root_path_url) }}


### 协程接口

代码文件：
- {{ '[aimrt_module_cpp_interface/co/aimrt_context.h]({}/src/interface/aimrt_module_cpp_interface/co/aimrt_context.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/async_scope.h]({}/src/interface/aimrt_module_cpp_interface/co/async_scope.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/inline_scheduler.h]({}/src/interface/aimrt_module_cpp_interface/co/inline_scheduler.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/on.h]({}/src/interface/aimrt_module_cpp_interface/co/on.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/schedule.h]({}/src/interface/aimrt_module_cpp_interface/co/schedule.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/sync_wait.h]({}/src/interface/aimrt_module_cpp_interface/co/sync_wait.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/co/task.h]({}/src/interface/aimrt_module_cpp_interface/co/task.h)'.format(code_site_root_path_url) }}

参考示例：
- {{ '[executor_co_module.cc]({}/src/examples/cpp/executor/module/executor_co_module/executor_co_module.cc)'.format(code_site_root_path_url) }}
- {{ '[executor_co_loop_module.cc]({}/src/examples/cpp/executor/module/executor_co_loop_module/executor_co_loop_module.cc)'.format(code_site_root_path_url) }}

## 执行器的概念


执行器是一个很早就有的概念，它表示一个可以执行逻辑代码的抽象概念，一个执行器可以是一个线程池、可以是一个协程/纤程，可以是 CPU、GPU、甚至是远端的一个服务器。我们平常写的最简单的代码也有一个默认的执行器：主线程。


一般来说，执行器都会有类似这样的两个接口：
```cpp
void Execute(std::function<void()>);
bool TryExecute(util::DynamicLatch&, std::function<void()>);

```

第一个接口表示，可以将一个类似于`std::function<void()>`的任务闭包投递到指定执行器中去执行。这个任务在何时何地执行则依赖于具体执行器的实现。C++ 标准库中的 std::thread 就是一个典型的执行器，它的构造函数接受传入一个`std::function<void()>`任务闭包，并将该任务放在一个新的线程中执行。

第二个接口携带一个`util::DynamicLatch&`参数，用于在投递任务的同时对“在途任务数”进行原子计数与生命周期管理：

- 当调用`TryExecute(latch, task)`时，执行器会先尝试对`latch`做一次`TryAdd()`；若返回`false`，表示闩锁已关闭（不再接收新任务），本次任务不会被投递，接口返回`false`。
- 当任务被实际执行完成后，执行器会自动调用`latch.CountDown()`一次，无需用户在任务体内手工计数。
- 若希望“停止接收新任务，并等待所有已投递任务全部执行完毕”，可在控制线程依次调用 `latch.Close()`（停止接收新任务）与 `latch.Wait()`（阻塞等待任务清零），或者直接使用 `CloseAndWait()`（上述操作的简化组合）。

注意：任务内若可能抛出异常，请自行捕获保证正常执行到`CountDown()`；并建议在控制/管理线程调用`Close()+Wait()`，避免在持有计数的执行路径内自锁。



## 基本执行器接口概述

在 AimRT 中，模块可以通过调用`CoreRef`句柄的`GetExecutorManager()`接口，获取`aimrt::configurator::ExecutorManagerRef`句柄，其中提供了一个简单的获取 Executor 的接口：

```cpp
namespace aimrt::executor {

class ExecutorManagerRef {
 public:
  ExecutorRef GetExecutor(std::string_view executor_name) const;
};

}  // namespace aimrt::executor
```

使用者可以调用`ExecutorManagerRef`类型的`GetExecutor`方法，获取指定名称的`aimrt::configurator::ExecutorRef`句柄，以调用执行器相关功能。`ExecutorRef`的核心接口如下：

```cpp
namespace aimrt::executor {

class ExecutorRef {
 public:
  std::string_view Type() const;

  std::string_view Name() const;

  bool ThreadSafe() const;

  bool IsInCurrentExecutor() const;

  bool SupportTimerSchedule() const;

  void Execute(Task&& task);

  bool TryExecute(util::DynamicLatch& latch_, Task&& task);

  std::chrono::system_clock::time_point Now() const;

  void ExecuteAt(std::chrono::system_clock::time_point tp, Task&& task) const;

  void ExecuteAfter(std::chrono::nanoseconds dt, Task&& task) const;
};

}  // namespace aimrt::executor
```

AimRT 中的执行器有一些固有属性，这些固有属性大部分跟**执行器类型**相关，在运行过程中不会改变。这些固有属性包括：
- **执行器类型**：一个字符串字段，标识执行器在运行时的类型。
  - 在一个 AimRT 实例中，会存在多种类型的执行器，AimRT 官方提供了几种执行器，插件也可以提供新类型的执行器。
  - 具体的执行器类型以及特性请参考部署环节的`executor`配置章节。
  - 在逻辑开发过程中，不应太关注实际运行时的执行器类型，只需根据抽象的执行器接口去实现业务逻辑。
- **执行器名称**：一个字符串字段，标识执行器在运行时的名称。
  - 在一个 AimRT 进程中，名称唯一标识了一个执行器。
  - 所有的执行器实例的名称都在运行时通过配置来决定，具体请参考部署环节的`executor`配置章节。
  - 可以通过`ExecutorManagerRef`的`GetExecutor`方法，获取指定名称的执行器。
- **线程安全性**：一个 bool 值，标识了本执行器是否是线程安全的。
  - 通常和执行器类型相关。
  - 线程安全的执行器可以保证投递到其中的任务不会同时运行；反之则不能保证。
- **是否支持按时间调度**：一个 bool 值，标识了本执行器是否支持按时间调度的接口，也就是`ExecuteAt`、`ExecuteAfter`接口。
  - 如果本执行器不支持按时间调度，则调用`ExecuteAt`、`ExecuteAfter`接口时会抛出一个异常。



关于`ExecutorRef`接口的详细使用说明如下：
- `std::string_view Type()`：获取执行器的类型。
- `std::string_view Name()`：获取执行器的名称。
- `bool ThreadSafe()`：返回本执行器是否是线程安全的。
- `bool IsInCurrentExecutor()`：判断调用此函数时是否在本执行器中。
  - 注意：如果返回 true，则当前环境一定在本执行器中；如果返回 false，则当前环境有可能不在本执行器中，也有可能在。
- `bool SupportTimerSchedule()`：返回本执行器是否支持按时间调度的接口，也就是`ExecuteAt`、`ExecuteAfter`接口。
- `void Execute(Task&& task)`：将一个任务投递到本执行器中，并在调度后立即执行。
  - 可将参数`Task`简单的视为一个满足`std::function<void()>`签名的任务闭包。
  - 此接口可以在 Initialize/Start 阶段调用，但执行器在 Start 阶段后才能保证开始执行，因此在 Start 阶段之前调用此接口，有可能只能将任务投递到执行器的任务队列中而暂时不执行，等到 Start 之后才开始执行任务。
- `bool Execute(util::DynamicLatch& latch_, Task&& task)`：带闩锁的任务投递。
  - 返回`true`表示任务被接受并投递；返回`false`表示闩锁已关闭（不再接收新任务），任务不会被投递。
  - 任务执行完成后将自动调用`latch_.CountDown()`，调用方无需在任务体内手工计数。
  - 可与`latch_.Close(); latch_.Wait();`组合实现“停止接收新任务并等待在途任务清空”的平滑关停。
- `std::chrono::system_clock::time_point Now()`：获取本执行器体系下的时间。
  - 对于一般的执行器来说，此处返回的都是`std::chrono::system_clock::now()`的结果。
  - 有一些带时间调速功能的特殊执行器，此处可能会返回经过处理的时间。
- `void ExecuteAt(std::chrono::system_clock::time_point tp, Task&& task)`：在某个时间点执行一个任务。
  - 第一个参数-时间点，以本执行器的时间体系为准。
  - 可将第二个参数`Task`简单的视为一个满足`std::function<void()>`签名的任务闭包。
  - 如果本执行器不支持按时间调度，则调用此接口时会抛出一个异常。
  - 此接口可以在 Initialize/Start 阶段调用，但执行器在 Start 阶段后才能保证开始执行，因此在 Start 阶段之前调用此接口，有可能只能将任务投递到执行器的任务队列中而暂时不执行，等到 Start 之后才开始执行任务。
- `void ExecuteAfter(std::chrono::nanoseconds dt, Task&& task)`：在某个时间后执行一个任务。
  - 第一个参数-时间段，以本执行器的时间体系为准。
  - 可将第二个参数`Task`简单的视为一个满足`std::function<void()>`签名的任务闭包。
  - 如果本执行器不支持按时间调度，则调用此接口时会抛出一个异常。
  - 此接口可以在 Initialize/Start 阶段调用，但执行器在 Start 阶段后才能保证开始执行，因此在 Start 阶段之前调用此接口，有可能只能将任务投递到执行器的任务队列中而暂时不执行，等到 Start 之后才开始执行任务


## 基本执行器接口使用示例

以下是一个简单的使用示例，演示了如何获取一个执行器句柄，并将一个简单的任务投递到该执行器中执行：
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

如果是一个线程安全的执行器，那么投递到其中的任务不需要加锁即可保证线程安全，示例如下：
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


### 带闩锁的任务投递示例

```cpp
auto exec = core_.GetExecutorManager().GetExecutor("work_executor");
aimrt::util::DynamicLatch latch;

for (int i = 0; i < 1000; ++i) {
  bool accepted = exec.Execute(latch, [i]() noexcept {
  });
  if (!accepted) {
    break;
  }
}


// 推荐直接使用 CloseAndWait，一步到位，简洁高效
latch.CloseAndWait();

// 如需分步骤，可先 Close 再 Wait
// latch.Close();
// latch.Wait();
```

以下这个示例则演示了如何使用 Time Schedule 接口，来实现定时循环：
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



## 执行器协程接口概述


AimRT 中，为执行器封装了基于 C++20 协程和 [libunifex 库](https://github.com/facebookexperimental/libunifex)的一个协程形式接口，提供了一个比较重要的类：`aimrt::co::AimRTScheduler`，可以由`aimrt::executor::ExecutorRef`句柄构造。这个类将原生的 AimRT 执行器句柄封装成协程形式，其中的核心接口如下：

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

## 执行器协程接口使用示例

有了`AimRTScheduler`句柄，就可以使用`aimrt::co`命名空间下的一系列协程工具了。以下是一个简单的使用示例，演示了如何启动一个协程，并在协程中调度到指定执行器中执行任务：
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


以下这个示例则演示了如何使用 Time Schedule 接口，基于协程来实现定时循环：
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
    run_flag_ = false;

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

## 基于执行器的定时器

### 定时器接口

代码文件：
- {{ '[aimrt_module_cpp_interface/executor/timer.h]({}/src/interface/aimrt_module_cpp_interface/executor/timer.h)'.format(code_site_root_path_url) }}

参考示例：
- {{ '[timer_module.cc]({}/src/examples/cpp/executor/module/timer_module/timer_module.cc)'.format(code_site_root_path_url) }}

### 定时器的概念

定时器是基于执行器提供的一个定时执行任务的工具，可以基于执行器创建一个定时器，并指定定时器执行的周期。

### 定时器接口

使用`aimrt::executor::CreateTimer`接口创建一个定时器，并指定定时器执行的周期和任务，其函数声明如下：

```cpp
namespace aimrt::executor {

template <typename TaskType>
std::shared_ptr<TimerBase> CreateTimer(ExecutorRef executor, std::chrono::nanoseconds period,
                                       TaskType&& task, bool auto_start = true);

}  // namespace aimrt::executor
```

其中`ExecutorRef`是执行器句柄，`TaskType`是任务类型，`period`是定时器执行的周期，`auto_start`是是否自动启动定时器，默认为`true`。

定时器所使用的 `ExecutorRef` 必须支持定时调度功能，即`SupportTimerSchedule()` 返回 `true`，可以参考[执行器配置](../cfg/executor.md)章节查询执行器是否支持定时调度功能。

`TaskType`是任务类型，接受一个可调用对象，可以使用`std::function`、`std::bind`、lambda 表达式等，只要其函数签名满足如下要求之一即可：

```cpp
void()
void(TimerBase&)
void(const TimerBase&)
```

函数签名中，`TimerBase&`是定时器对象本身，`const TimerBase&`是定时器对象的常量引用。

`TimerBase`是定时器对象的基类，`Timer`是定时器对象的派生类，主要封装了用户指定的定时器任务的执行，我们一般使用 `TimerBase` 的智能指针类型：`std::shared_ptr<TimerBase>`。

`TimerBase` 的核心接口如下：

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

关于`TimerBase`类中接口的详细使用说明如下：
- `void Cancel()`：取消定时器，设置 cancel 状态。
- `void Reset()`：重置定时器，取消 cancel 状态，并重置下次执行时间，下一次执行时间会基于当前时间加上周期计算得出。
- `void ExecuteTask()`：执行定时器任务。
- `bool IsCancelled()`：返回定时器是否被取消。
- `std::chrono::nanoseconds Period()`：返回定时器执行的周期。
- `std::chrono::system_clock::time_point NextCallTime()`：返回定时器下次执行的时间。
- `std::chrono::nanoseconds TimeUntilNextCall()`：返回定时器下次执行的时间与当前时间的时间差。
- `ExecutorRef Executor()`：返回定时器所属的执行器。

### 定时器行为概述

定时器的行为如下：
- 定时器创建后，默认是自动启动的，相当于自动调用一次`Reset()`接口，如果不想自动启动，可以设置`auto_start`为`false`，此时定时器会处于`cancel`状态。
- 定时器无论是否启动，调用`Cancel()`接口，会取消定时器，并设置 cancel 状态。
- 定时器无论是否启动，调用`Reset()`接口，会重置定时器，取消 cancel 状态，并重置下次执行时间，下一次执行时间会基于当前时间加上周期计算得出。
- `Reset()` 接口可以覆盖原先的定时器任务，即调用`Reset()`接口后，紧接着调用`Reset()`接口，会重新按照新的周期执行任务，原先的定时器任务会被新的任务覆盖。
- 如果任务执行时间太长或者定时器所使用的执行器中存在阻塞操作，导致错过部分定时周期，定时器不会将错过的次数补上，而是等到下次执行时间到达时执行任务，举例如下：
  - 假设定时器周期为 1000 ms，原本预计在 0, 1000, 2000, 3000, 4000, ... ms 各执行一次任务
  - 假设任务执行时间为 1500 ms，那么在 0 ms 时启动的任务在 1500 ms 时执行完毕，并错过了 1000 ms 时的执行
  - 定时器会将下一次执行时间重置为 2000 ms，并在 2000 ms 时执行任务，而不会补上 1000 ms 时的执行
  - 最终任务的执行起始时间点是：0, 2000, 4000, 6000, ... ms

### 定时器使用示例

以下是一个简单的使用示例，演示了如何创建一个定时器，并使用定时器执行一个任务：
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
