// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.
#include "executor/executor_module.h"

#include <chrono>
#include <thread>

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "util/exception.h"

namespace aimrt::examples::cpp::context::executor_module {

bool ExecutorModule::Initialize(aimrt::CoreRef core) {
  ctx_ptr_ = aimrt::context::Context::Letme(core);
  time_schedule_executor_ = ctx_ptr_->CreateExecutor("time_schedule_executor");

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
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));
  AIMRT_INFO("Shutdown succeeded.");
}

void ExecutorModule::TimeScheduleDemo() {
  const auto count = ++loop_count_;
  AIMRT_INFO("Timer loop count : {}", count);

  time_schedule_executor_.ExecuteAfter(
      std::chrono::seconds(1),
      [this]() {
        ctx_ptr_->LetMe();
        if (!aimrt::context::Running()) {
          return;
        }
        time_schedule_executor_.Execute([this] { TimeScheduleDemo(); });
      });
}

}  // namespace aimrt::examples::cpp::context::executor_module
