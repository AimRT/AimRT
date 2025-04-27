// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/channel/channel_backend_manager.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace aimrt::runtime::core::channel {

class MockChannelBackend : public ChannelBackendBase {
 public:
  std::string_view Name() const noexcept override { return "mock_backend"; }

  void Initialize(YAML::Node options_node) noexcept override {
    is_initialized_ = true;
  }
  void Start() override { is_statrted_ = true; }

  void Shutdown() override { is_shutdowned_ = false; }

  bool RegisterPublishType(const PublishTypeWrapper& publish_type_wrapper) noexcept override {
    is_registered_publish_type_ = true;
    return true;
  }

  bool Subscribe(const SubscribeWrapper& subscribe_wrapper) noexcept override {
    is_subscribed = true;
    return true;
  }

  void Publish(MsgWrapper& msg_wrapper) noexcept override {
    is_published = true;
  }

  bool is_initialized_ = false;
  bool is_statrted_ = false;
  bool is_shutdowned_ = false;

  bool is_registered_publish_type_ = false;
  bool is_subscribed = false;
  bool is_published = false;
};

class ChannelBackendManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    EXPECT_EQ(channel_backend_manager_.GetState(), ChannelBackendManager::State::kPreInit);
    channel_backend_manager_.RegisterChannelBackend(mock_backend_ptr_.get());

    channel_backend_manager_.SetPubTopicsBackendsRules({
        {"(.*)", {"mock_backend"}},
    });
    channel_backend_manager_.SetSubTopicsBackendsRules({
        {"(.*)", {"mock_backend"}},
    });

    channel_backend_manager_.SetChannelRegistry(channel_registry_test_ptr_.get());
    channel_backend_manager_.Initialize();
    EXPECT_EQ(channel_backend_manager_.GetState(), ChannelBackendManager::State::kInit);
  }

  // Test Shutdown
  void TearDown() override {
    channel_backend_manager_.Shutdown();
    EXPECT_EQ(channel_backend_manager_.GetState(), ChannelBackendManager::State::kShutdown);
  }

  std::unique_ptr<MockChannelBackend> mock_backend_ptr_ = std::make_unique<MockChannelBackend>();
  std::shared_ptr<ChannelRegistry> channel_registry_test_ptr_ = std::make_shared<ChannelRegistry>();

  ChannelBackendManager channel_backend_manager_;
};

// Test RegisterChannelBackend, Start
TEST_F(ChannelBackendManagerTest, RegisterChannelBackend_Start) {
  EXPECT_EQ(mock_backend_ptr_->is_statrted_, false);
  channel_backend_manager_.Start();
  EXPECT_EQ(mock_backend_ptr_->is_statrted_, true);
}

}  // namespace aimrt::runtime::core::channel