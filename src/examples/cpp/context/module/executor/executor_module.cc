// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.
#include "executor/executor_module.h"

#include <chrono>
#include <thread>

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "util/exception.h"

namespace aimrt::examples::cpp::context::executor_module {

bool ExecutorModule::Initialize(aimrt::CoreRef core) {
  core_ = core;
  ctx_ = std::make_shared<aimrt::context::Context>(core_);

  work_executor_ = ctx_->GetExecutor("work_executor");

  thread_safe_executor_ = ctx_->GetExecutor("thread_safe_executor");

  time_schedule_executor_ = ctx_->GetExecutor("time_schedule_executor");

  AIMRT_CHECK_ERROR_THROW(work_executor_, "Can not get work_executor");

  AIMRT_CHECK_ERROR_THROW(thread_safe_executor_ && thread_safe_executor_.ThreadSafe(), "Can not get thread_safe_executor");

  AIMRT_CHECK_ERROR_THROW(time_schedule_executor_ && time_schedule_executor_.SupportTimerSchedule(),
                          "Can not get time_schedule_executor");

  AIMRT_INFO("Init succeeded.");
  return true;
}

bool ExecutorModule::Start() {
  TimeScheduleDemo();

  AIMRT_INFO("Start succeeded.");
  return true;
}

void ExecutorModule::Shutdown() {
  run_flag_.store(false, std::memory_order_relaxed);
  if (ctx_) {
    ctx_->RequireToShutdown();
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1200));
  AIMRT_INFO("Shutdown succeeded.");
}

void ExecutorModule::TimeScheduleDemo() {
  if (!run_flag_.load(std::memory_order_relaxed)) {
    return;
  }

  const auto count = loop_count_.fetch_add(1, std::memory_order_relaxed);
  AIMRT_INFO("Timer loop count : {}", count);

  time_schedule_executor_.ExecuteAfter(
      std::chrono::seconds(1),
      [this, ctx = ctx_, res = time_schedule_executor_]() {
        if (!run_flag_.load(std::memory_order_relaxed)) {
          return;
        }
        if (!ctx) {
          return;
        }

        time_schedule_executor_.Execute([this] { TimeScheduleDemo(); });
      });
}

}  // namespace aimrt::examples::cpp::context::executor_module
