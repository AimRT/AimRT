// Copyright (c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#include "timer_module/timer_module.h"

#include <chrono>
#include <memory>

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

}  // namespace aimrt::examples::cpp::executor::timer_module
