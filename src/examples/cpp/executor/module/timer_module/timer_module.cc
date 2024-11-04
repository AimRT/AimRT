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
    static auto start_time = now;

    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
    AIMRT_HL_INFO(logger, "Executed {} times, execute time: {} ms", ++count, interval);
  };

  timer_ = aimrt::executor::CreateTimer(timer_executor_, 100ms, std::move(task));
  AIMRT_INFO("Timer created with auto start with 100 ms period");

  timer_executor_.ExecuteAfter(150ms, [this, logger = core_.GetLogger()]() {
    timer_->Reset();
    AIMRT_HL_INFO(logger, "Timer reset at 150 ms");
  });

  timer_executor_.ExecuteAfter(500ms, [this, logger = core_.GetLogger()]() {
    timer_->Start();
    AIMRT_HL_INFO(logger, "Timer restarted at 500 ms");
  });

  timer_executor_.ExecuteAfter(800ms, [this, logger = core_.GetLogger()]() {
    timer_->Cancel();
    AIMRT_HL_INFO(logger, "Timer cancelled at 800 ms");
  });

  return true;
}

void TimerModule::Shutdown() {
  timer_->Cancel();
  timer_->SyncWait();
}

}  // namespace aimrt::examples::cpp::executor::timer_module