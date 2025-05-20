// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/allocator/allocator_manager.h"
#include <gtest/gtest.h>

namespace aimrt::runtime::core::allocator {
TEST(AllocatorManagerTest, AllocatorManager) {
  AllocatorManager allocator_manager;
  YAML::Node options_node_test;
  EXPECT_EQ(allocator_manager.GetState(), AllocatorManager::State::kPreInit);
  allocator_manager.Initialize(options_node_test);
  EXPECT_EQ(allocator_manager.GetState(), AllocatorManager::State::kInit);
  const AllocatorProxy& allocator_proxy = allocator_manager.GetAllocatorProxy();
  EXPECT_EQ(allocator_proxy.NativeHandle()->get_thread_local_buf(allocator_proxy.NativeHandle()->impl, 1024 * 1024 * 16 + 1), nullptr);
  auto* buf_ptr = allocator_proxy.NativeHandle()->get_thread_local_buf(allocator_proxy.NativeHandle()->impl, 1024);
  EXPECT_NE(buf_ptr, nullptr);
  allocator_manager.Start();
  EXPECT_EQ(allocator_manager.GetState(), AllocatorManager::State::kStart);
  allocator_manager.Shutdown();
  EXPECT_EQ(allocator_manager.GetState(), AllocatorManager::State::kShutdown);
}

}  // namespace aimrt::runtime::core::allocator