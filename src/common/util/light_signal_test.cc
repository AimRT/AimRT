// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "util/light_signal.h"

namespace aimrt::common::util {
TEST(LightSignalTest, base) {
  LightSignal s;
  uint32_t i = 0;

  std::thread t1([&] {
    ASSERT_EQ(i, 0);
    s.Wait();
    ASSERT_EQ(i, 1);
    s.Reset();
    s.Wait();
    ASSERT_EQ(i, 2);
  });

  std::thread t2([&] {
    ASSERT_EQ(i, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i = 1;
    s.Notify();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i = 2;
    s.Notify();
  });

  t1.join();
  t2.join();
}

}  // namespace aimrt::common::util