// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/executor/guard_thread_executor.h"
#include "gtest/gtest.h"
namespace aimrt::runtime::core::executor {
TEST(GuardThreadExecutorTest, Constructor) {
  GuardThreadExecutor guard_thread_executor;
  YAML::Node options_node_test = YAML::Load(R"str( 
name: guard_thread_executor 
thread_sched_policy: SCHED_OTHER 
thread_bind_cpu: [0] 
    )str");
  guard_thread_executor.Initialize(options_node_test);
  EXPECT_EQ(guard_thread_executor.Name(), "guard_thread_executor");
  EXPECT_TRUE(guard_thread_executor.ThreadSafe());
  EXPECT_FALSE(guard_thread_executor.SupportTimerSchedule());
  EXPECT_EQ(guard_thread_executor.Type(), "guard_thread");

  guard_thread_executor.Start();
  bool is_executed = false;
  aimrt::executor::Task &&task = aimrt::executor::Task(
      [&is_executed]() mutable { is_executed = true; });

  guard_thread_executor.Execute(std::ref(task));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(is_executed);

  guard_thread_executor.Shutdown();
}

TEST(GuardThreadExecutorTest, ConstructorWithEmptyCfg) {
  GuardThreadExecutor guard_thread_executor;
  YAML::Node options_node_test = YAML::Load(R"str( 
    )str");
  guard_thread_executor.Initialize(options_node_test);
  guard_thread_executor.Start();
  guard_thread_executor.Shutdown();
  SUCCEED();
}

}  // namespace aimrt::runtime::core::executor