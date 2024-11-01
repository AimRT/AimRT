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

  auto task = [logger = core_.GetLogger()]() {
    auto now = std::chrono::system_clock::now();

    static int count = 0;
    static auto last_time = now;

    count += 1;
    auto interval = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
    AIMRT_HL_INFO(logger, "Executed {} times, execute interval: {} us", count, interval);

    last_time = now;
    std::this_thread::sleep_for(300ms);
  };

  timer_ = aimrt::executor::CreateTimer(timer_executor_, 1s, std::move(task));

  timer_executor_.ExecuteAfter(1500ms, [this, logger = core_.GetLogger()]() {
    timer_->Reset();
    AIMRT_HL_INFO(logger, "Timer reset");
  });

  timer_executor_.ExecuteAfter(5000ms, [this, logger = core_.GetLogger()]() {
    timer_->Cancel();
    AIMRT_HL_INFO(logger, "Timer cancelled");
  });

  return true;
}

void TimerModule::Shutdown() {
  timer_->Cancel();
}

}  // namespace aimrt::examples::cpp::executor::timer_module
