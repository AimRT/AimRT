// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/parameter/parameter_manager.h"
#include <gtest/gtest.h>

namespace aimrt::runtime::core::parameter {

class ParameterManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    YAML::Node options_node_test = YAML::Load(R"str()str");
    EXPECT_EQ(parametre_manager_.GetState(), ParameterManager::State::kPreInit);
    EXPECT_NO_THROW(parametre_manager_.Initialize(options_node_test));
    EXPECT_EQ(parametre_manager_.GetState(), ParameterManager::State::kInit);
  }
  void TearDown() override {
    EXPECT_NO_THROW(parametre_manager_.Shutdown());
    EXPECT_EQ(parametre_manager_.GetState(), ParameterManager::State::kShutdown);
  }

  ParameterManager parametre_manager_;
};

// 测试Start 和 GetParameterHandle
TEST_F(ParameterManagerTest, GetParameterHandle) {
  EXPECT_NO_THROW(parametre_manager_.Start());
  EXPECT_EQ(parametre_manager_.GetState(), ParameterManager::State::kStart);
  EXPECT_EQ(parametre_manager_.GetParameterHandle("test_module"), nullptr);
}

// 测试GetParameterHandleProxy
TEST_F(ParameterManagerTest, GetParameterHandleProxy) {
  parametre_manager_.GetParameterHandleProxy("test_module");
  EXPECT_NO_THROW(parametre_manager_.Start());
  ParameterHandle* parameter_handle = parametre_manager_.GetParameterHandle("test_module");
  EXPECT_NE(parameter_handle, nullptr);
}

}  // namespace aimrt::runtime::core::parameter