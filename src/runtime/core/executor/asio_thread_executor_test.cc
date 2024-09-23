// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "core/executor/asio_thread_executor.h"

namespace aimrt::runtime::core::executor {

TEST(ASIO_THREAD_EXECUTOR_TEST, base) {
  YAML::Node options_node = YAML::Load(R"str(
thread_num: 1
)str");

  std::string_view name = "test_thread";

  AsioThreadExecutor executor;
  executor.Initialize(name, options_node);

  EXPECT_EQ(executor.Type(), "asio_thread");
  EXPECT_EQ(executor.Name(), name);
  EXPECT_TRUE(executor.ThreadSafe());

  executor.Shutdown();
}

TEST(ASIO_THREAD_EXECUTOR_TEST, base2) {
  YAML::Node options_node = YAML::Load(R"str(
thread_num: 2
)str");

  std::string_view name = "test_thread_2";

  AsioThreadExecutor executor;
  executor.Initialize(name, options_node);

  EXPECT_EQ(executor.Type(), "asio_thread");
  EXPECT_EQ(executor.Name(), name);
  EXPECT_FALSE(executor.ThreadSafe());

  executor.Shutdown();
}

TEST(ASIO_THREAD_EXECUTOR_TEST, execute) {
  YAML::Node options_node = YAML::Load(R"str(
    thread_num: 1
    thread_sched_policy: SCHED_OTHER
    thread_bind_cpu: [2]
    timeout_alarm_threshold_us: 1000
  )str");

  std::string_view name = "test_thread";

  AsioThreadExecutor executor;
  executor.Initialize(name, options_node);

  executor.Start();

  EXPECT_FALSE(executor.IsInCurrentExecutor());

  bool ret = false;
  executor.Execute(
      [&]() { ret = executor.IsInCurrentExecutor(); });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(ret);

  // ExecuteAt
  ret = false;
  executor.ExecuteAt(
      std::chrono::system_clock::now() + std::chrono::milliseconds(5),
      [&]() { ret = true; });
  EXPECT_FALSE(ret);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(ret);

  executor.Shutdown();
}

}  // namespace aimrt::runtime::core::executor