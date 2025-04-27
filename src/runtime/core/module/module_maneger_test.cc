// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>
#include "aimrt_module_cpp_interface/module_base.h"
#include "core/module/module_manager.h"

#define TESTMODULE "TestModule"

namespace aimrt::runtime::core::module {

TEST(ModuleManagerTest, ModuleManagerTest) {
  // Mock a module inherits the module base
  class MockModule : public aimrt::ModuleBase {
   public:
    ModuleInfo Info() const override {
      ModuleInfo info;
      info.name = TESTMODULE;
      return info;
    }

    bool Initialize(CoreRef core) override {
      is_initialized_ = true;
      return true;
    }

    bool Start() override {
      is_start_ = true;
      return true;
    }

    void Shutdown() override {
      is_shutdown_ = true;
    }

    bool is_initialized_ = false;
    bool is_start_ = false;
    bool is_shutdown_ = false;
  };
  std::unique_ptr<MockModule> module_test_ptr = std::make_unique<MockModule>();

  ModuleManager module_manager;

  // Test Initialize
  YAML::Node options_node_test = YAML::Load(R"str(
)str");
  ModuleManager::CoreProxyConfigurator module_proxy_configurator = [](const util::ModuleDetailInfo& info, CoreProxy& proxy) {};
  module_manager.RegisterCoreProxyConfigurator(std::move(module_proxy_configurator));
  EXPECT_EQ(module_manager.GetState(), ModuleManager::State::kPreInit);
  module_manager.RegisterModule(module_test_ptr->NativeHandle());
  module_manager.Initialize(options_node_test);
  EXPECT_EQ(module_manager.GetState(), ModuleManager::State::kInit);
  EXPECT_EQ(module_test_ptr->is_initialized_, true);
  EXPECT_EQ(module_manager.GetModuleNameList().size(), 1);
  EXPECT_EQ(module_manager.GetModuleDetailInfoList()[0]->name, TESTMODULE);

  // Test Start
  EXPECT_EQ(module_test_ptr->is_start_, false);
  module_manager.Start();
  EXPECT_EQ(module_manager.GetState(), ModuleManager::State::kStart);
  EXPECT_EQ(module_test_ptr->is_start_, true);

  // Test Shutdown
  EXPECT_EQ(module_test_ptr->is_shutdown_, false);
  module_manager.Shutdown();
  EXPECT_EQ(module_manager.GetState(), ModuleManager::State::kShutdown);
  EXPECT_EQ(module_test_ptr->is_shutdown_, true);
}

}  // namespace aimrt::runtime::core::module
