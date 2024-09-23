// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/util/yaml_tools.h"
#include "gtest/gtest.h"

namespace aimrt::runtime::core::util {

TEST(YamlToolsTest, CheckYamlNodes) {
  YAML::Node standard_node = YAML::Load(R"str(
aimrt:
  executor:
    executors:
      - name: 
        type: 
        options:
          thread_num: 
  module:
    modules:
      - name: 
        log_lvl: 
)str");

  YAML::Node checked_node = YAML::Load(R"str(
aimrt:
  executor:
    executors:
      - name: 
        type: 
        option:
          thread_num: 
  module:
    module:
      - name: 
        log_lvl:  
)str");
  auto result = CheckYamlNodes(standard_node["aimrt"], checked_node["aimrt"], "aimrt");

  EXPECT_NE(result.find("executor.executors[0].option"), std::string::npos);
  EXPECT_NE(result.find("module.module"), std::string::npos);
}

}  // namespace aimrt::runtime::core::util