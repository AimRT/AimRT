// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "core/executor/time_wheel_executor.h"

namespace aimrt::runtime::core::executor {

TEST(TIME_WHEEL_EXECUTOR_TEST, base) {
  YAML::Node options_node = YAML::Load(R"str(
dt_us: 1000
wheel_size: [1000, 600]
)str");

  std::string_view name = "test_time_wheel_executor";

  TimeWheelExecutor executor;
  executor.RegisterGetExecutorFunc([](auto) {
    return aimrt::executor::ExecutorRef();
  });
  executor.Initialize(name, options_node);

  EXPECT_EQ(executor.Type(), "time_wheel");
  EXPECT_EQ(executor.Name(), name);
  EXPECT_TRUE(executor.ThreadSafe());
  EXPECT_TRUE(executor.SupportTimerSchedule());

  executor.Shutdown();
}

TEST(TIME_WHEEL_EXECUTOR_TEST, Execute) {
  YAML::Node options_node = YAML::Load(R"str(
dt_us: 1000
wheel_size: [1000, 600]
  )str");

  std::string_view name = "test_time_wheel_executor";

  TimeWheelExecutor executor;
  executor.RegisterGetExecutorFunc([](auto) {
    return aimrt::executor::ExecutorRef();
  });
  executor.Initialize(name, options_node);
  executor.Start();

  EXPECT_FALSE(executor.IsInCurrentExecutor());

  bool ret = false;
  executor.Execute(
      [&]() { ret = executor.IsInCurrentExecutor(); });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(ret);

  executor.Shutdown();
}

TEST(TIME_WHEEL_EXECUTOR_TEST, ExecuteAt) {
  YAML::Node options_node = YAML::Load(R"str(
dt_us: 1000
wheel_size: [1000, 600]
  )str");

  std::string_view name = "test_time_wheel_executor";

  TimeWheelExecutor executor;
  executor.RegisterGetExecutorFunc([](auto) {
    return aimrt::executor::ExecutorRef();
  });
  executor.Initialize(name, options_node);
  executor.Start();

  bool ret = false;
  executor.ExecuteAt(
      executor.Now() + std::chrono::milliseconds(100),
      [&]() { ret = true; });

  EXPECT_FALSE(ret);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_TRUE(ret);

  executor.Shutdown();
}
}  // namespace aimrt::runtime::core::executor
