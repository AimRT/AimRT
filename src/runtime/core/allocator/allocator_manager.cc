// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/allocator/allocator_manager.h"

namespace YAML {
template <>
struct convert<aimrt::runtime::core::allocator::AllocatorManager::Options> {
  using Options = aimrt::runtime::core::allocator::AllocatorManager::Options;

  static Node encode(const Options& rhs) {
    Node node;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    return true;
  }
};
}  // namespace YAML

namespace aimrt::runtime::core::allocator {

void AllocatorManager::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "AllocatorManager manager can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  options_node = options_;

  AIMRT_INFO("Allocator manager init complete");
}

void AllocatorManager::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");

  AIMRT_INFO("Allocator manager start completed.");
}

void AllocatorManager::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  AIMRT_INFO("Allocator manager shutdown.");
}

const AllocatorProxy& AllocatorManager::GetAllocatorProxy(const util::ModuleDetailInfo& module_info) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");

  return allocator_proxy_;
}

std::list<std::pair<std::string, std::string>> AllocatorManager::GenInitializationReport() const {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");

  return {};
}

}  // namespace aimrt::runtime::core::allocator
