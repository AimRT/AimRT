// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/executor/simple_thread_executor.h"
#include "gtest/gtest.h"
namespace aimrt::runtime::core::executor {
TEST(SimpleThreadExecutorTest, Constructor) {
  SimpleThreadExecutor simple_thread_executor;
  YAML::Node options_node_test = YAML::Load(R"str( 
thread_sched_policy: SCHED_OTHER 
thread_bind_cpu: [0] 
    )str");
  std::string_view name = "simple_thread_executor";
  simple_thread_executor.Initialize(name, options_node_test);
  EXPECT_EQ(simple_thread_executor.Name(), name);
  EXPECT_TRUE(simple_thread_executor.ThreadSafe());
  EXPECT_FALSE(simple_thread_executor.SupportTimerSchedule());
  EXPECT_EQ(simple_thread_executor.Type(), "simple_thread");

  simple_thread_executor.Start();
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  bool is_executed = false;
  aimrt::executor::Task &&task = aimrt::executor::Task(
      [&is_executed]() mutable { is_executed = true; });

  simple_thread_executor.Execute(std::ref(task));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(is_executed);

  simple_thread_executor.Shutdown();
}

}  // namespace aimrt::runtime::core::executor