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


一般来说，执行器都会有类似这样的一个接口：
```cpp
void Execute(std::function<void()>);
```

这个接口表示，可以将一个类似于`std::function<void()>`的任务闭包投递到指定执行器中去执行。这个任务在何时何地执行则依赖于具体执行器的实现。C++ 标准库中的 std::thread 就是一个典型的执行器，它的构造函数接受传入一个`std::function<void()>`任务闭包，并将该任务放在一个新的线程中执行。


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

  void Execute(Task&& task) const;

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
