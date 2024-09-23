// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/channel/channel_manager.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace aimrt::runtime::core::channel {

class ChannelManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    YAML::Node options_node_test = YAML::Load(R"str(
aimrt:
  channel: 
    backends: 
      - type: mock_backend_test 
)str");

    EXPECT_EQ(channel_manager_.GetState(), ChannelManager::State::kPreInit);
    channel_manager_.RegisterChannelBackend(std::move(channel_backend_test_ptr_));

    // 初始化ChannelManager
    channel_manager_.Initialize(options_node_test["aimrt"]["channel"]);
    EXPECT_EQ(channel_manager_.GetState(), ChannelManager::State::kInit);
  }
  // 测试用例结束后清理工作
  void TearDown() override {
    channel_manager_.Shutdown();
    EXPECT_EQ(channel_manager_.GetState(), ChannelManager::State::kShutdown);
  }

  // 模拟的通道后端类，继承自ChannelBackendBase
  class MockChannelBackend : public ChannelBackendBase {
   public:
    std::string_view Name() const noexcept override { return "mock_backend_test"; }
    MOCK_METHOD1(Initialize, void(YAML::Node options_node));
    MOCK_METHOD0(Start, void());
    MOCK_METHOD0(Shutdown, void());
    bool RegisterPublishType(
        const PublishTypeWrapper& publish_type_wrapper) noexcept override { return false; }
    bool Subscribe(const SubscribeWrapper& subscribe_wrapper) noexcept override { return false; }
    void Publish(MsgWrapper& msg_wrapper) noexcept override { return; }
  };
  std::unique_ptr<MockChannelBackend> channel_backend_test_ptr_ = std::make_unique<MockChannelBackend>();
  ChannelManager channel_manager_;
};

// 测试Start
TEST_F(ChannelManagerTest, Start) {
  channel_manager_.Start();
  EXPECT_EQ(channel_manager_.GetState(), ChannelManager::State::kStart);
}

}  // namespace aimrt::runtime::core::channel
