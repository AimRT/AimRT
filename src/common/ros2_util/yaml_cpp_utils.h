// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "yaml-cpp/yaml.h"

namespace aimrt::common::ros2_util {

template <typename T>
inline T GetYamlValue(const YAML::Node& node) {
  return T{};
}

template <>
inline int GetYamlValue<int>(const YAML::Node& node) {
  return node.as<int>();
}

template <>
inline uint64_t GetYamlValue<uint64_t>(const YAML::Node& node) {
  return node.as<uint64_t>();
}

template <>
inline uint32_t GetYamlValue<uint32_t>(const YAML::Node& node) {
  return node.as<uint32_t>();
}

template <>
inline uint16_t GetYamlValue<uint16_t>(const YAML::Node& node) {
  return node.as<uint16_t>();
}

template <>
inline uint8_t GetYamlValue<uint8_t>(const YAML::Node& node) {
  return node.as<uint8_t>();
}

template <>
inline int64_t GetYamlValue<int64_t>(const YAML::Node& node) {
  return node.as<int64_t>();
}

template <>
inline int16_t GetYamlValue<int16_t>(const YAML::Node& node) {
  return node.as<int16_t>();
}

template <>
inline int8_t GetYamlValue<int8_t>(const YAML::Node& node) {
  return node.as<int8_t>();
}

template <>
inline double GetYamlValue<double>(const YAML::Node& node) {
  return node.as<double>();
}

template <>
inline float GetYamlValue<float>(const YAML::Node& node) {
  return node.as<float>();
}

template <>
inline bool GetYamlValue<bool>(const YAML::Node& node) {
  return node.as<bool>();
}

template <>
inline std::string GetYamlValue<std::string>(const YAML::Node& node) {
  return node.as<std::string>();
}

template <>
inline char GetYamlValue<char>(const YAML::Node& node) {
  return node.as<char>();
}

template <typename T>
inline T GetYamlValueOr(const YAML::Node& node, const T& default_value) {
  return node ? GetYamlValue<T>(node) : default_value;
}

}  // namespace aimrt::common::ros2_util