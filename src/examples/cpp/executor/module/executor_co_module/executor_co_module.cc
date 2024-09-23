// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "executor_co_module/executor_co_module.h"
#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/inline_scheduler.h"
#include "aimrt_module_cpp_interface/co/on.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::executor::executor_co_module {

bool ExecutorCoModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  ctx_ = co::AimRTContext(core_.GetExecutorManager());

  // Check executor
  auto work_executor = core_.GetExecutorManager().GetExecutor("work_executor");
  AIMRT_CHECK_ERROR_THROW(work_executor, "Can not get work_executor");

  // Check thread safe executor
  auto thread_safe_executor = core_.GetExecutorManager().GetExecutor("thread_safe_executor");
  AIMRT_CHECK_ERROR_THROW(thread_safe_executor && thread_safe_executor.ThreadSafe(),
                          "Can not get thread_safe_executor");

  // Check time schedule executor
  auto time_schedule_executor = core_.GetExecutorManager().GetExecutor("time_schedule_executor");
  AIMRT_CHECK_ERROR_THROW(time_schedule_executor && time_schedule_executor.SupportTimerSchedule(),
                          "Can not get time_schedule_executor");

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool ExecutorCoModule::Start() {
  // Test simple execute
  scope_.spawn(co::On(co::InlineScheduler(), SimpleExecuteDemo()));

  // Test thread safe execute
  scope_.spawn(co::On(co::InlineScheduler(), ThreadSafeDemo()));

  // Test time schedule execute
  scope_.spawn(co::On(co::InlineScheduler(), TimeScheduleDemo()));

  AIMRT_INFO("Start succeeded.");

  return true;
}

void ExecutorCoModule::Shutdown() {
  run_flag_ = false;

  co::SyncWait(scope_.complete());

  AIMRT_INFO("Shutdown succeeded.");
}

co::Task<void> ExecutorCoModule::SimpleExecuteDemo() {
  // Get work_executor scheduler
  auto work_scheduler = ctx_.GetScheduler("work_executor");

  // Schedule to work_executor
  co_await co::Schedule(work_scheduler);

  // Run task in work_executor
  AIMRT_INFO("This is a simple task");

  co_return;
}

co::Task<void> ExecutorCoModule::ThreadSafeDemo() {
  // Get thread_safe_executor scheduler
  auto thread_safe_scheduler = ctx_.GetScheduler("thread_safe_executor");

  // Run task in thread_safe_executor
  co::AsyncScope scope;

  uint32_t n = 0;
  auto task = [&n]() -> co::Task<void> {
    n++;
    co_return;
  };

  for (size_t ii = 0; ii < 10000; ++ii) {
    scope.spawn(co::On(thread_safe_scheduler, task()));
  }

  // Wait for all task done
  co_await co::On(co::InlineScheduler(), scope.complete());

  AIMRT_INFO("Value of n is {}", n);

  co_return;
}

co::Task<void> ExecutorCoModule::TimeScheduleDemo() {
  AIMRT_INFO("Start loop.");

  // Get time_schedule_executor scheduler
  auto time_scheduler = ctx_.GetScheduler("time_schedule_executor");

  co_await co::Schedule(time_scheduler);

  uint32_t count = 0;
  while (run_flag_) {
    count++;
    AIMRT_INFO("Loop count : {} -------------------------", count);

    co_await co::ScheduleAfter(time_scheduler, std::chrono::seconds(1));
  }

  AIMRT_INFO("Exit loop.");

  co_return;
}

}  // namespace aimrt::examples::cpp::executor::executor_co_module
