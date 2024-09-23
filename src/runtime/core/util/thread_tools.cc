// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/util/thread_tools.h"

#include <cstdlib>
#include <thread>

#include "util/exception.h"
#include "util/format.h"

#if defined(_WIN32)
  #include <windows.h>
const DWORD MS_VC_EXCEPTION = 0x406D1388;
  #pragma pack(push, 8)
typedef struct tagTHREADNAME_INFO {
  DWORD dwType;      // Must be 0x1000.
  LPCSTR szName;     // Pointer to name (in user addr space).
  DWORD dwThreadID;  // Thread ID (-1=caller thread).
  DWORD dwFlags;     // Reserved for future use, must be zero.
} THREADNAME_INFO;
  #pragma pack(pop)
void SetThreadName(DWORD dwThreadID, const char* threadName) {
  THREADNAME_INFO info;
  info.dwType = 0x1000;
  info.szName = threadName;
  info.dwThreadID = dwThreadID;
  info.dwFlags = 0;
  #pragma warning(push)
  #pragma warning(disable : 6320 6322)
  __try {
    RaiseException(MS_VC_EXCEPTION, 0, sizeof(info) / sizeof(ULONG_PTR), (ULONG_PTR*)&info);
  } __except (EXCEPTION_EXECUTE_HANDLER) {
  }
  #pragma warning(pop)
}
#endif

namespace aimrt::runtime::core::util {

void SetNameForCurrentThread(std::string_view thread_name) {
  std::string name(thread_name);

#if defined(_WIN32)
  SetThreadName((DWORD)-1, name.c_str());
#else
  if (name.size() < 15) {
    pthread_setname_np(pthread_self(), name.data());
  } else {
    std::string real_name =
        name.substr(0, 8) + ".." + name.substr(name.size() - 5, name.size());
    pthread_setname_np(pthread_self(), real_name.c_str());
  }

#endif
}

void BindCpuForCurrentThread(const std::vector<uint32_t>& cpu_set) {
  if (cpu_set.empty()) return;

#if defined(_WIN32)
  throw aimrt::common::util::AimRTException(
      "BindCpuForCurrentThread currently not supported on the Windows platform.");
#else
  static const uint32_t kMaxCpuIdx = std::thread::hardware_concurrency();
  for (auto cpu_idx : cpu_set) {
    if (cpu_idx >= kMaxCpuIdx) {
      throw aimrt::common::util::AimRTException(::aimrt_fmt::format(
          "Invalid cpu index '{}', max cpu idx is '{}'.",
          cpu_idx, kMaxCpuIdx));
    }
  }

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  for (auto cpu_idx : cpu_set) {
    CPU_SET(cpu_idx, &cpuset);
  }

  auto ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
  if (ret) {
    throw aimrt::common::util::AimRTException(::aimrt_fmt::format(
        "Call 'pthread_setaffinity_np' get error, ret code '{}'", ret));
  }
#endif
}

void SetCpuSchedForCurrentThread(std::string_view sched) {
  if (sched.empty()) return;

#if defined(_WIN32)
  throw aimrt::common::util::AimRTException(
      "SetCpuSchedForCurrentThread currently not supported on the Windows platform.");
#else
  if (sched == "SCHED_OTHER") {
    sched_param param;
    param.sched_priority = 0;
    int ret = pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    if (ret) {
      throw aimrt::common::util::AimRTException(::aimrt_fmt::format(
          "Call 'pthread_setschedparam' get error, ret code '{}'", ret));
    }

  } else {
    auto pos = sched.find_first_of(':');

    if (pos == std::string_view::npos || pos == sched.size() - 1) {
      throw aimrt::common::util::AimRTException(
          ::aimrt_fmt::format("Invalid sched parm '{}'", sched));
    }
    auto sched_policy_str = sched.substr(0, pos);

    int policy = 0;

    if (sched_policy_str == "SCHED_FIFO") {
      policy = SCHED_FIFO;
    } else if (sched_policy_str == "SCHED_RR") {
      policy = SCHED_RR;
    } else {
      throw aimrt::common::util::AimRTException(
          ::aimrt_fmt::format("Invalid sched parm '{}'", sched));
    }

    int priority_max = sched_get_priority_max(policy);
    int priority_min = sched_get_priority_min(policy);

    struct sched_param param;
    auto sched_priority_str = sched.substr(pos + 1);
    param.sched_priority = atoi(sched_priority_str.data());
    if (param.sched_priority < priority_min ||
        param.sched_priority > priority_max) {
      throw aimrt::common::util::AimRTException(::aimrt_fmt::format(
          "Invalid sched priority '{}', required range {}~{}",
          param.sched_priority, priority_min, priority_max));
    }

    int ret = pthread_setschedparam(pthread_self(), policy, &param);
    if (ret) {
      throw aimrt::common::util::AimRTException(::aimrt_fmt::format(
          "Call 'pthread_setschedparam' get error, ret code '{}'", ret));
    }

    int check_policy = 0;
    struct sched_param check_param;
    ret = pthread_getschedparam(pthread_self(), &check_policy, &check_param);
    if (ret) {
      throw aimrt::common::util::AimRTException(::aimrt_fmt::format(
          "Call 'pthread_getschedparam' get error, ret code '{}'", ret));
    }
    if (check_policy != policy ||
        check_param.sched_priority != param.sched_priority) {
      throw aimrt::common::util::AimRTException(::aimrt_fmt::format(
          "Set sched failed, want '{}:{}', actual '{}:{}'",
          policy, param.sched_priority, check_policy, check_param.sched_priority));
    }
  }

#endif
}

}  // namespace aimrt::runtime::core::util
