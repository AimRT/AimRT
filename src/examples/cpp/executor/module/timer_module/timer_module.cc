// Copyright (c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#include "timer_module/timer_module.h"

#include <chrono>

namespace aimrt::examples::cpp::executor::timer_module {

bool TimerModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  timer_executor_ = core_.GetExecutorManager().GetExecutor("timer_executor");
  AIMRT_CHECK_ERROR_THROW(timer_executor_, "Can not get timer_executor");
  AIMRT_CHECK_ERROR_THROW(timer_executor_.SupportTimerSchedule(),
                          "timer_executor does not support timer schedule");

  return true;
}

bool TimerModule::Start() {
  auto task = [logger = core_.GetLogger()]() {
    static int count = 0;
    count += 1;
    AIMRT_HL_INFO(logger, "Executed {} times", count);
  };

  timer_ = std::make_shared<WallTimer>(timer_executor_, std::chrono::seconds(1), std::move(task));

  return true;
}

void TimerModule::Shutdown() {}

}  // namespace aimrt::examples::cpp::executor::timer_module
