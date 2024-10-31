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
    auto now = std::chrono::system_clock::now();

    static int count = 0;
    static auto last_time = now;

    count += 1;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
    AIMRT_HL_INFO(logger, "Executed {} times, execute interval: {} ms", count, interval);

    last_time = now;
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  };

  timer_ = std::make_shared<aimrt::executor::Timer>(timer_executor_, std::chrono::seconds(1), std::move(task));

  timer_executor_.ExecuteAfter(std::chrono::seconds(4), [this, logger = core_.GetLogger()]() {
    timer_->Cancel();
    AIMRT_HL_INFO(logger, "Timer cancelled");
  });

  return true;
}

void TimerModule::Shutdown() {}

}  // namespace aimrt::examples::cpp::executor::timer_module
