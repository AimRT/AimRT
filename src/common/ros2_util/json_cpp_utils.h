// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <json/json.h>

namespace aimrt::common::ros2_util {

template <typename T>
inline T GetJsonValue(const Json::Value& value) {
  return T{};
}

template <>
inline int GetJsonValue<int>(const Json::Value& value) {
  return value.asInt();
}

template <>
inline uint64_t GetJsonValue<uint64_t>(const Json::Value& value) {
  return value.asUInt64();
}

template <>
inline uint32_t GetJsonValue<uint32_t>(const Json::Value& value) {
  return value.asUInt();
}

template <>
inline uint16_t GetJsonValue<uint16_t>(const Json::Value& value) {
  return value.asUInt();
}

template <>
inline uint8_t GetJsonValue<uint8_t>(const Json::Value& value) {
  return value.asUInt();
}

template <>
inline int64_t GetJsonValue<int64_t>(const Json::Value& value) {
  return value.asInt64();
}

template <>
inline int16_t GetJsonValue<int16_t>(const Json::Value& value) {
  return value.asInt();
}

template <>
inline int8_t GetJsonValue<int8_t>(const Json::Value& value) {
  return value.asInt();
}

template <>
inline double GetJsonValue<double>(const Json::Value& value) {
  return value.asDouble();
}

template <>
inline float GetJsonValue<float>(const Json::Value& value) {
  return value.asFloat();
}

template <>
inline bool GetJsonValue<bool>(const Json::Value& value) {
  return value.asBool();
}

template <>
inline std::string GetJsonValue<std::string>(const Json::Value& value) {
  return value.asString();
}

template <>
inline char GetJsonValue<char>(const Json::Value& value) {
  return value.asUInt();
}

}  // namespace aimrt::common::ros2_util