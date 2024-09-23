// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>
#include <fstream>
#include <iostream>

#include "core/configurator/configurator_manager.h"
#include "core/util/module_detail_info.h"

namespace aimrt::runtime::core::configurator {

const std::filesystem::path kConfiguratorManagerTestPath = "./configurator_manager_test_cfg.yaml";

class ConfiguratorManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    configurator_manager_.Initialize(kConfiguratorManagerTestPath);

    YAML::Node configurator_options_node = configurator_manager_.GetAimRTOptionsNode("configurator");
    EXPECT_EQ(configurator_options_node.IsNull(), false);
    EXPECT_EQ(configurator_options_node.IsDefined(), true);
  }

  void TearDown() override {
    configurator_manager_.Shutdown();
  }

  static void SetUpTestCase() {
    const auto *cfg_content = R"str(
aimrt:
  configurator:
    temp_cfg_path: ./cfg/tmp 
  module: 
    modules: 
      - name: ConfiguratorManagerTest 
        log_lvl: INFO 
ConfiguratorManagerTest:
  key1: val1
  key2: val2
)str";

    std::ofstream outfile;
    outfile.open(kConfiguratorManagerTestPath, std::ios::out);
    outfile << cfg_content;

    outfile.close();
  }

  static void TearDownTestCase() {
    std::error_code error;
    auto file_status = std::filesystem::status(kConfiguratorManagerTestPath, error);

    if (std::filesystem::exists(file_status)) {
      std::filesystem::remove(kConfiguratorManagerTestPath);
    }
  }

  ConfiguratorManager configurator_manager_;
};

TEST_F(ConfiguratorManagerTest, initialize) {
  YAML::Node configurator_ori_options_node = configurator_manager_.GetOriRootOptionsNode();
  EXPECT_EQ(configurator_ori_options_node.IsNull(), false);
  EXPECT_EQ(configurator_ori_options_node.IsDefined(), true);

  YAML::Node configurator_root_options_node = configurator_manager_.GetRootOptionsNode();
  EXPECT_EQ(configurator_root_options_node.IsNull(), false);
  EXPECT_EQ(configurator_root_options_node.IsDefined(), true);
}

TEST_F(ConfiguratorManagerTest, start) {
  configurator_manager_.Start();

  configurator_manager_.Shutdown();
}

TEST_F(ConfiguratorManagerTest, get_configuratorProxy_with_legal_module_name) {
  util::ModuleDetailInfo detail_info = {
      .name = "ConfiguratorManagerTest",
      .cfg_file_path = "./cfg/tmp",
  };

  const auto *h = configurator_manager_.GetConfiguratorProxy(detail_info).NativeHandle();
  ASSERT_NE(h, nullptr);
  EXPECT_EQ(aimrt::util::ToStdStringView(h->config_file_path(h->impl)), "./cfg/tmp");
}

TEST_F(ConfiguratorManagerTest, get_configuratorProxy_with_illegal_module_name) {
  util::ModuleDetailInfo detail_info = {
      .name = "IllegalTest",
  };

  const auto *h = configurator_manager_.GetConfiguratorProxy(detail_info).NativeHandle();
  ASSERT_NE(h, nullptr);
  EXPECT_EQ(aimrt::util::ToStdStringView(h->config_file_path(h->impl)), "");
}

TEST_F(ConfiguratorManagerTest, get_configuratorProxy_with_configured_module_name) {
  util::ModuleDetailInfo detail_info = {
      .name = "ConfiguratorManagerTest",
  };

  const auto *h = configurator_manager_.GetConfiguratorProxy(detail_info).NativeHandle();
  ASSERT_NE(h, nullptr);
  EXPECT_EQ(
      std::filesystem::path(aimrt::util::ToStdStringView(h->config_file_path(h->impl))),
      std::filesystem::path("./cfg/tmp/temp_cfg_file_for_ConfiguratorManagerTest.yaml"));
}

}  // namespace aimrt::runtime::core::configurator