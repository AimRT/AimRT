// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "yaml-cpp/yaml.h"

namespace aimrt::runtime::core::util {

std::string CheckYamlNodes(
    YAML::Node standard_node, YAML::Node checked_node, const std::string& path, uint32_t level = 0);

}  // namespace aimrt::runtime::core::util