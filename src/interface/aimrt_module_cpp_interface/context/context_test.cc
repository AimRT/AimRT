// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "context.h"
#include "details/thread_context.h"
#include "util/log_util.h"

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

TEST(ContextTest, WithoutGetLoggerToUseLog) {
  EXPECT_NO_THROW(AIMRT_INFO("This is a test log message."););
  EXPECT_NO_THROW(AIMRT_INFO_STREAM("This is a "
                                    << "test log message."););
}

TEST(ContextStateTest, DefaultStatesAreOn) {
  auto ctx = std::make_shared<aimrt::context::Context>();
  EXPECT_EQ(ctx->GetPubState(), aimrt::context::ChannelState::kOn);
  EXPECT_EQ(ctx->GetSubState(), aimrt::context::ChannelState::kOn);
  EXPECT_EQ(ctx->GetCliState(), aimrt::context::RpcState::kOn);
  EXPECT_EQ(ctx->GetSrvState(), aimrt::context::RpcState::kOn);
}

TEST(ContextStateTest, SwitchChannelAndRpcStates) {
  auto ctx = std::make_shared<aimrt::context::Context>();

  ctx->SetPubState(aimrt::context::ChannelState::kOff);
  ctx->SetSubState(aimrt::context::ChannelState::kOff);
  ctx->SetCliState(aimrt::context::RpcState::kOff);
  ctx->SetSrvState(aimrt::context::RpcState::kOff);

  EXPECT_EQ(ctx->GetPubState(), aimrt::context::ChannelState::kOff);
  EXPECT_EQ(ctx->GetSubState(), aimrt::context::ChannelState::kOff);
  EXPECT_EQ(ctx->GetCliState(), aimrt::context::RpcState::kOff);
  EXPECT_EQ(ctx->GetSrvState(), aimrt::context::RpcState::kOff);

  ctx->SetPubState(aimrt::context::ChannelState::kOn);
  ctx->SetSubState(aimrt::context::ChannelState::kOn);
  ctx->SetCliState(aimrt::context::RpcState::kOn);
  ctx->SetSrvState(aimrt::context::RpcState::kOn);

  EXPECT_EQ(ctx->GetPubState(), aimrt::context::ChannelState::kOn);
  EXPECT_EQ(ctx->GetSubState(), aimrt::context::ChannelState::kOn);
  EXPECT_EQ(ctx->GetCliState(), aimrt::context::RpcState::kOn);
  EXPECT_EQ(ctx->GetSrvState(), aimrt::context::RpcState::kOn);
}
