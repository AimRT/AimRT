// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "executor_module/executor_module.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::executor::executor_module {

bool ExecutorModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  // Get executor
  work_executor_ = core_.GetExecutorManager().GetExecutor("work_executor");
  AIMRT_CHECK_ERROR_THROW(work_executor_, "Can not get work_executor");

  // Get thread safe executor
  thread_safe_executor_ = core_.GetExecutorManager().GetExecutor("thread_safe_executor");
  AIMRT_CHECK_ERROR_THROW(thread_safe_executor_ && thread_safe_executor_.ThreadSafe(),
                          "Can not get thread_safe_executor");

  // Get time schedule executor
  time_schedule_executor_ = core_.GetExecutorManager().GetExecutor("time_schedule_executor");
  AIMRT_CHECK_ERROR_THROW(time_schedule_executor_ && time_schedule_executor_.SupportTimerSchedule(),
                          "Can not get time_schedule_executor");

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool ExecutorModule::Start() {
  // Test simple execute
  SimpleExecuteDemo();

  // Test thread safe execute
  ThreadSafeDemo();

  // Test time schedule execute
  TimeScheduleDemo();

  AIMRT_INFO("Start succeeded.");

  return true;
}

void ExecutorModule::Shutdown() {
  run_flag_ = false;

  std::this_thread::sleep_for(std::chrono::seconds(1));

  AIMRT_INFO("Shutdown succeeded.");
}

void ExecutorModule::SimpleExecuteDemo() {
  work_executor_.Execute([this]() {
    AIMRT_INFO("This is a simple task");
  });
}

void ExecutorModule::ThreadSafeDemo() {
  uint32_t n = 0;
  for (uint32_t ii = 0; ii < 10000; ++ii) {
    thread_safe_executor_.Execute([&n]() {
      n++;
    });
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));

  AIMRT_INFO("Value of n is {}", n);

  // Test thread safe execute with latch
  uint32_t m = 0;
  for (uint32_t ii = 0; ii < 10000; ++ii) {
    thread_safe_executor_.TryExecute(latch_, [&m]() {
      m++;
    });
  }
  latch_.CloseAndWait();
  AIMRT_INFO("Value of m is {}", m);
}

void ExecutorModule::TimeScheduleDemo() {
  if (!run_flag_) return;

  AIMRT_INFO("Loop count : {}", loop_count_++);

  time_schedule_executor_.ExecuteAfter(
      std::chrono::seconds(1),
      std::bind(&ExecutorModule::TimeScheduleDemo, this));
}

}  // namespace aimrt::examples::cpp::executor::executor_module
