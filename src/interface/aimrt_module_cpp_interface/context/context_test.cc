// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "context.h"
#include "details/thread_context.h"

TEST(ContextTest, RunningToggle) {
  using aimrt::context::Context;

  auto ctx = std::make_shared<Context>();
  ctx->LetMe();

  EXPECT_TRUE(aimrt::context::Running());

  ctx->StopRunning();
  EXPECT_FALSE(aimrt::context::Running());
}

TEST(ContextTest, SwitchThreadContext) {
  auto ctx1 = std::make_shared<aimrt::context::Context>();
  ctx1->LetMe();
  EXPECT_EQ(aimrt::context::details::GetCurrentContext().get(), ctx1.get());

  auto ctx2 = std::make_shared<aimrt::context::Context>();
  ctx2->LetMe();
  EXPECT_EQ(aimrt::context::details::GetCurrentContext().get(), ctx2.get());

  ctx1->LetMe();
  EXPECT_EQ(aimrt::context::details::GetCurrentContext().get(), ctx1.get());
}
