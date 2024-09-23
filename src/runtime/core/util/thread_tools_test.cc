// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/util/thread_tools.h"
#include <gtest/gtest.h>
#include "util/exception.h"
#include "util/log_util.h"

namespace aimrt::runtime::core::util {

#if !defined(_WIN32)
// 非Windows系统下，获取当前线程的名字
std::string GetCurrentThreadName() {
  char name[16];
  pthread_getname_np(pthread_self(), name, 16);
  return std::string(name);
}

// 获取当前线程的CPU亲和性掩码
std::vector<uint32_t> GetCurrentThreadAffinity() {
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);

  if (pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
    std::cerr << "Failed to get thread affinity" << std::endl;
  }

  std::vector<uint32_t> cpu_indices;
  for (int i = 0; i < CPU_SETSIZE; ++i) {
    if (CPU_ISSET(i, &cpuset)) {
      cpu_indices.push_back(i);
    }
  }
  return cpu_indices;
}
#endif

TEST(ThreadToolsTest, SetNameForCurrentThread) {
  // 测试给一个短名字的线程设置名字
#if !defined(_WIN32)

  std::string short_name = "short_name";
  SetNameForCurrentThread(short_name);
  EXPECT_EQ(GetCurrentThreadName(), short_name);

  // 测试给一个长名字的线程设置名字
  std::string long_name = "long_thread_name_more_than_15_characters_long";
  SetNameForCurrentThread(long_name);
  std::string expected_name = "long_thr.._long";
  EXPECT_EQ(GetCurrentThreadName(), expected_name);
#else
  // 测试Windows平台不支持的情况
  GTEST_SKIP() << "Skipping this test on Windows as it is not supported.";
#endif
}

TEST(ThreadToolsTest, BindCpuForCurrentThread) {
#if !defined(_WIN32)
  // 测试正常情况
  std::vector<uint32_t> cpu_set = {0, 1};
  EXPECT_NO_THROW(BindCpuForCurrentThread(cpu_set));

  // 判断cpu_set和GetCurrentThreadAffinity返回值是否相等
  EXPECT_EQ(cpu_set, GetCurrentThreadAffinity());

  // 测试空cpu_set
  std::vector<uint32_t> empty_cpu_set = {};
  EXPECT_NO_THROW(BindCpuForCurrentThread(empty_cpu_set));

  // 测试无效的CPU索引
  EXPECT_ANY_THROW(BindCpuForCurrentThread({10000}));

#else
  // 测试Windows平台不支持的情况
  EXPECT_ANY_THROW(BindCpuForCurrentThread({10000}));
#endif
}

TEST(ThreadToolsTest, SetCpuSchedForCurrentThread) {
#if !defined(_WIN32)

  auto lgr = aimrt::common::util::SimpleLogger();

  // 测试传入空字符串
  EXPECT_NO_THROW(SetCpuSchedForCurrentThread(""));

  // 测试传入SCHED_OTHER
  EXPECT_NO_THROW(SetCpuSchedForCurrentThread("SCHED_OTHER"));

  // 测试传入SCHED_FIFO,先判断是否有权限访问（因为没有权限，不确定是否会正常运行）
  if (sched_setscheduler(0, SCHED_FIFO, nullptr) == 0) {
    int priority_max_fifo = sched_get_priority_max(SCHED_FIFO);
    int priority_min_fifo = sched_get_priority_min(SCHED_FIFO);
    int priority_fifo = (priority_max_fifo + priority_min_fifo) / 2;
    std::string sched_fifo = "SCHED_FIFO:" + std::to_string(priority_fifo);
    EXPECT_NO_THROW(SetCpuSchedForCurrentThread(sched_fifo));
  } else {
    AIMRT_HL_WARN(lgr, "This is a test warning: Failed to set SCHED_FIFO scheduler, permission denied !");
  }

  // 测试传入SCHED_RR,先判断是否有权限访问
  if (sched_setscheduler(0, SCHED_RR, nullptr) == 0) {
    int priority_max_rr = sched_get_priority_max(SCHED_RR);
    int priority_min_rr = sched_get_priority_min(SCHED_RR);
    int priority_rr = (priority_max_rr + priority_min_rr) / 2;
    std::string sched_rr = "SCHED_RR:" + std::to_string(priority_rr);
    EXPECT_NO_THROW(SetCpuSchedForCurrentThread(sched_rr));
  } else {
    AIMRT_HL_WARN(lgr, "This is a test warning: Failed to set SCHED_RR scheduler, permission denied !");
  }

  // 测试传入无效策略
  EXPECT_ANY_THROW(SetCpuSchedForCurrentThread("SCHED_INVALID"));

  // 测试传入无效策略优先级
  int priority_max = sched_get_priority_max(SCHED_FIFO);
  int priority_min = sched_get_priority_min(SCHED_FIFO);
  int priority = priority_max + 1;
  std::string sched = "SCHED_FIFO:" + std::to_string(priority);
  EXPECT_ANY_THROW(SetCpuSchedForCurrentThread(sched));

  // 测试传入无效格式、
  EXPECT_THROW(SetCpuSchedForCurrentThread("SCHED_FIFO"), aimrt::common::util::AimRTException);
  EXPECT_THROW(SetCpuSchedForCurrentThread("SCHED_FIFO:"), aimrt::common::util::AimRTException);

#else
  // 测试Windows平台不支持的情况
  EXPECT_THROW(SetCpuSchedForCurrentThread("SCHED_OTHER"), aimrt::common::util::AimRTException);
#endif
}

}  // namespace aimrt::runtime::core::util
