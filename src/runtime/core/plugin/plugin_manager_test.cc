// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/plugin/plugin_manager.h"
#include <gtest/gtest.h>
#include "core/aimrt_core.h"

namespace aimrt::runtime::core::plugin {

// Test initialization when AimRT core is not registered
TEST(PluginManagerTest, Initialize1) {
  PluginManager plugin_manager;
  YAML::Node options_node_test = YAML::Load(R"str(
)str");
  EXPECT_THROW(plugin_manager.Initialize(options_node_test), aimrt::common::util::AimRTException);
}

// Test initialization when AimRT core is registered successfully
TEST(PluginManagerTest, Initialize2) {
  // Define a MockPlugin
  class MockPlugin : public AimRTCorePluginBase {
   public:
    std::string_view Name() const noexcept override { return "test_plugin"; }
    bool Initialize(runtime::core::AimRTCore* core_ptr) noexcept override {
      is_initialized_ = true;
      return true;
    }
    void Shutdown() noexcept override { is_shutdown_ = true; }

    bool is_initialized_ = false;
    bool is_shutdown_ = false;
  };
  std::shared_ptr<MockPlugin> mock_plugin_ptr = std::make_shared<MockPlugin>();
  AimRTCore* core_ptr = new AimRTCore;

  PluginManager plugin_manager;

  YAML::Node options_node_test = YAML::Load(R"str(
plugins: 
    - name: test_plugin 
      path: ""
)str");

  plugin_manager.RegisterCorePtr(core_ptr);
  plugin_manager.RegisterPlugin(mock_plugin_ptr.get());
  EXPECT_EQ(plugin_manager.GetState(), PluginManager::State::kPreInit);
  plugin_manager.Initialize(options_node_test);
  EXPECT_EQ(plugin_manager.GetState(), PluginManager::State::kInit);
  EXPECT_TRUE(mock_plugin_ptr->is_initialized_);

  plugin_manager.Start();
  EXPECT_EQ(plugin_manager.GetState(), PluginManager::State::kStart);
}

}  // namespace aimrt::runtime::core::plugin
