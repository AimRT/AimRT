// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/util/yaml_tools.h"

namespace aimrt::runtime::core::util {

// 计算两个字符串的距离
int LevenshteinDistance(const std::string& s1, const std::string& s2) {
  int len1 = s1.size();
  int len2 = s2.size();
  std::vector<std::vector<int>> dp(len1 + 1, std::vector<int>(len2 + 1));

  for (int i = 0; i <= len1; ++i) dp[i][0] = i;
  for (int j = 0; j <= len2; ++j) dp[0][j] = j;

  for (int i = 1; i <= len1; ++i) {
    for (int j = 1; j <= len2; ++j) {
      if (s1[i - 1] == s2[j - 1]) {
        dp[i][j] = dp[i - 1][j - 1];
      } else {
        dp[i][j] = std::min({dp[i - 1][j], dp[i][j - 1], dp[i - 1][j - 1]}) + 1;
      }
    }
  }
  return dp[len1][len2];
}

// 找到最接近的匹配节点
std::string FindClosestMatch(const YAML::Node& node, const std::string& target) {
  std::string closest_match;
  int min_distance = 4;

  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
    const std::string& key = it->first.as<std::string>();
    int distance = LevenshteinDistance(key, target);
    if (distance < min_distance) {
      min_distance = distance;
      closest_match = key;
    }
  }
  return closest_match;
}

std::string CheckYamlNodes(
    YAML::Node standard_node, YAML::Node checked_node, const std::string& path, int level) {
  std::stringstream msg;

  if (!checked_node) {
    return msg.str();
  }

  if (!standard_node) {
    msg << "- Unused options: '" << path << "'" << std::endl;
    return msg.str();
  }
  // 确保YAML文件索引最多嵌套索引到第四层且节点类型为序列，因为前四层的配置条目名称是唯一确定的
  if (level >= 4 && checked_node.Type() == YAML::NodeType::Sequence) {
    return msg.str();
  }

  switch (checked_node.Type()) {
    case YAML::NodeType::Sequence:
      if (!standard_node.IsSequence()) {
        msg << "- Unused options: '" << path << "'" << std::endl;
      } else {
        for (size_t i = 0; i < checked_node.size(); ++i) {
          if (i >= standard_node.size()) {
            msg << "- Unused options: '" << path << "[" << checked_node[i] << "]'" << std::endl;
          } else {
            msg << CheckYamlNodes(standard_node[i], checked_node[i], path + "[" + std::to_string(i) + "]", level + 1);
          }
        }
      }
      break;
    case YAML::NodeType::Map:
      if (!standard_node.IsMap()) {
        msg << "ConfigurationName Warning: '" << path << "'" << std::endl;
      } else {
        for (YAML::const_iterator it = checked_node.begin(); it != checked_node.end(); ++it) {
          const std::string& key = it->first.as<std::string>();
          if (!standard_node[key]) {
            msg << "- Unused options: '" << path << "." << key << "'!";

            std::string closest_match = FindClosestMatch(standard_node, key);
            if (!closest_match.empty()) {
              msg << " Did you mean: '" << path << "." << closest_match << "'?" << std::endl;
            } else {
              msg << std::endl;
            }
          } else {
            msg << CheckYamlNodes(standard_node[key], checked_node[key], path + "." + key, level + 1);
          }
        }
      }
      break;
    default:
      break;
  }

  return msg.str();
}

}  // namespace aimrt::runtime::core::util