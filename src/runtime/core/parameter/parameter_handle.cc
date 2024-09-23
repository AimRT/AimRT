// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/parameter/parameter_handle.h"

namespace aimrt::runtime::core::parameter {

std::shared_ptr<const std::string> ParameterHandle::GetParameter(std::string_view key) {
  std::lock_guard<std::mutex> lck(parameter_map_mutex_);

  auto find_itr = parameter_map_.find(key);
  if (find_itr != parameter_map_.end()) {
    AIMRT_TRACE("Get parameter '{}'", key);

    return find_itr->second;
  }

  AIMRT_TRACE("Can not get parameter '{}'", key);
  return std::shared_ptr<std::string>();
}

void ParameterHandle::SetParameter(
    std::string_view key, const std::shared_ptr<std::string>& value_ptr) {
  std::lock_guard<std::mutex> lck(parameter_map_mutex_);

  auto find_itr = parameter_map_.find(key);
  if (find_itr != parameter_map_.end()) {
    if (!value_ptr || value_ptr->empty()) [[unlikely]] {
      AIMRT_TRACE("Erase parameter '{}'", key);
      parameter_map_.erase(find_itr);
      return;
    }

    AIMRT_TRACE("Update parameter '{}'", key);
    find_itr->second = value_ptr;

    return;
  }

  AIMRT_TRACE("Set parameter '{}'", key);
  parameter_map_.emplace(key, value_ptr);
}

std::vector<std::string> ParameterHandle::ListParameter() const {
  std::lock_guard<std::mutex> lck(parameter_map_mutex_);

  std::vector<std::string> result;
  result.reserve(parameter_map_.size());

  for (const auto& itr : parameter_map_) {
    result.emplace_back(itr.first);
  }

  return result;
}

}  // namespace aimrt::runtime::core::parameter
