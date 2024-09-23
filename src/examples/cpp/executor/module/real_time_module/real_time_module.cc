// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "real_time_module/real_time_module.h"
#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/on.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::executor::real_time_module {

bool RealTimeModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool RealTimeModule::Start() {
  StartWorkLoopByExecutor("sched_fifo_thread");

  StartWorkLoopByExecutor("sched_other_thread");

  StartWorkLoopByExecutor("sched_rr_thread");

  AIMRT_INFO("Start succeeded.");
  return true;
}

void RealTimeModule::Shutdown() {
  try {
    // Wait all coroutine complete
    run_flag_ = false;
    co::SyncWait(scope_.complete());
  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
    return;
  }

  AIMRT_INFO("Shutdown succeeded.");
}

void RealTimeModule::StartWorkLoopByExecutor(std::string_view executor_name) {
  auto executor = core_.GetExecutorManager().GetExecutor(executor_name);
  AIMRT_CHECK_ERROR_THROW(executor && executor.SupportTimerSchedule(),
                          "Get executor '{}' failed.", executor_name);
  scope_.spawn(co::On(co::AimRTScheduler(executor), WorkLoop(executor)));
}

co::Task<void> RealTimeModule::WorkLoop(aimrt::executor::ExecutorRef executor) {
  try {
    AIMRT_INFO("Start WorkLoop in {}.", executor.Name());

#ifdef __linux__
    // Get thread name
    char thread_name[16];
    pthread_getname_np(pthread_self(), thread_name, sizeof(thread_name));

    // Get thread sched param
    int policy = 0;
    struct sched_param param;
    pthread_getschedparam(pthread_self(), &policy, &param);

    AIMRT_INFO("Executor name: {}, thread_name: {}, policy: {}, priority: {}",
               executor.Name(), thread_name, policy, param.sched_priority);
#endif

    uint32_t count = 0;
    while (run_flag_) {
      count++;

      // Sleep for some time
      auto start_tp = std::chrono::steady_clock::now();
      co_await co::ScheduleAfter(
          co::AimRTScheduler(executor), std::chrono::milliseconds(1000));
      auto end_tp = std::chrono::steady_clock::now();

#ifdef __linux__
      // Get cpuset used by the current thread
      cpu_set_t cur_cpuset;
      CPU_ZERO(&cur_cpuset);
      auto pthread_getaffinity_np_ret =
          pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &cur_cpuset);
      AIMRT_CHECK_ERROR_THROW(pthread_getaffinity_np_ret == 0,
                              "Call 'pthread_getaffinity_np' get error: {}",
                              pthread_getaffinity_np_ret);

      uint32_t cpu_size = std::thread::hardware_concurrency();
      std::string cur_cpuset_str;
      for (int ii = 0; ii < cpu_size; ii++) {
        if (CPU_ISSET(ii, &cur_cpuset)) {
          cur_cpuset_str += (std::to_string(ii) + ", ");
        }
      }
      cur_cpuset_str = cur_cpuset_str.substr(0, cur_cpuset_str.size() - 2);

      // Get cpu index used by the current code
      unsigned int current_cpu = 0, current_node = 0;
      int getcpu_ret = getcpu(&current_cpu, &current_node);
      AIMRT_CHECK_ERROR_THROW(getcpu_ret == 0, "Call 'getcpu' get error: {}", getcpu_ret);

      // Log
      AIMRT_INFO(
          "Loop count: {}, executor name: {}, sleep for {}, cpu: {}, node: {}, use cpu: '{}'",
          count, executor.Name(), end_tp - start_tp, current_cpu, current_node, cur_cpuset_str);
#else
      // Log
      AIMRT_INFO(
          "Loop count: {}, executor name: {}, sleep for {}",
          count, executor.Name(), end_tp - start_tp);
#endif
    }

    AIMRT_INFO("Exit WorkLoop.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("Exit WorkLoop with exception, {}", e.what());
  }

  co_return;
}

}  // namespace aimrt::examples::cpp::executor::real_time_module
