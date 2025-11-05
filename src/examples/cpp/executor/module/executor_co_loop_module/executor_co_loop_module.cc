// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "executor_co_loop_module/executor_co_loop_module.h"
#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/inline_scheduler.h"
#include "aimrt_module_cpp_interface/co/on.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::executor::executor_co_loop_module {

bool ExecutorCoLoopModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  // Check time schedule executor
  time_schedule_executor_ = core_.GetExecutorManager().GetExecutor("time_schedule_executor");
  AIMRT_CHECK_ERROR_THROW(time_schedule_executor_ && time_schedule_executor_.SupportTimerSchedule(),
                          "Can not get time_schedule_executor");

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool ExecutorCoLoopModule::Start() {
  scope_.spawn(co::On(co::InlineScheduler(), MainLoop()));
  AIMRT_INFO("Start succeeded.");

  return true;
}

void ExecutorCoLoopModule::Shutdown() {
  run_flag_ = false;

  co::SyncWait(scope_.complete());

  AIMRT_INFO("Shutdown succeeded.");
}

co::Task<void> ExecutorCoLoopModule::MainLoop() {
  AIMRT_INFO("Start loop.");

  auto scheduler = aimrt::co::AimRTScheduler(time_schedule_executor_);
  co_await co::Schedule(scheduler);

  uint32_t count = 0;
  while (run_flag_) {
    count++;
    AIMRT_INFO("Loop count : {} -------------------------", count);

    co_await co::ScheduleAfter(scheduler, std::chrono::seconds(1));
  }

  AIMRT_INFO("Exit loop.");

  co_return;
}

}  // namespace aimrt::examples::cpp::executor::executor_co_loop_module
