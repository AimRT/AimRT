// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <functional>

#include "core/rpc/rpc_invoke_wrapper.h"
#include "util/exception.h"

namespace aimrt::runtime::core::rpc {

using FrameworkAsyncRpcHandle = std::function<void(const std::shared_ptr<InvokeWrapper>&)>;
using FrameworkAsyncRpcFilter = std::function<void(const std::shared_ptr<InvokeWrapper>&, FrameworkAsyncRpcHandle&&)>;

class FrameworkAsyncRpcFilterCollector {
 public:
  FrameworkAsyncRpcFilterCollector()
      : final_filter_(
            [](const std::shared_ptr<InvokeWrapper>& invoke_wrapper_ptr, FrameworkAsyncRpcHandle&& h) {
              h(invoke_wrapper_ptr);
            }) {}
  ~FrameworkAsyncRpcFilterCollector() = default;

  FrameworkAsyncRpcFilterCollector(const FrameworkAsyncRpcFilterCollector&) = delete;
  FrameworkAsyncRpcFilterCollector& operator=(const FrameworkAsyncRpcFilterCollector&) = delete;

  void RegisterFilter(const FrameworkAsyncRpcFilter& filter) {
    final_filter_ =
        [final_filter{std::move(final_filter_)}, &filter](
            const std::shared_ptr<InvokeWrapper>& invoke_wrapper_ptr, FrameworkAsyncRpcHandle&& h) {
          filter(
              invoke_wrapper_ptr,
              [&final_filter, h{std::move(h)}](const std::shared_ptr<InvokeWrapper>& invoke_wrapper_ptr) mutable {
                final_filter(invoke_wrapper_ptr, std::move(h));
              });
        };
  }

  void InvokeRpc(
      FrameworkAsyncRpcHandle&& h, const std::shared_ptr<InvokeWrapper>& invoke_wrapper_ptr) const {
    final_filter_(invoke_wrapper_ptr, std::move(h));
  }

  void Clear() {
    final_filter_ = FrameworkAsyncRpcFilter();
  }

 private:
  FrameworkAsyncRpcFilter final_filter_;
};

class FrameworkAsyncRpcFilterManager {
 public:
  FrameworkAsyncRpcFilterManager() = default;
  ~FrameworkAsyncRpcFilterManager() = default;

  FrameworkAsyncRpcFilterManager(const FrameworkAsyncRpcFilterManager&) = delete;
  FrameworkAsyncRpcFilterManager& operator=(const FrameworkAsyncRpcFilterManager&) = delete;

  void RegisterFilter(std::string_view name, FrameworkAsyncRpcFilter&& filter) {
    auto emplace_ret = filter_map_.emplace(name, std::move(filter));
    AIMRT_ASSERT(emplace_ret.second, "Register filter {} failed.", name);
  }

  std::vector<std::string> GetAllFiltersName() const {
    std::vector<std::string> result;
    result.reserve(filter_map_.size());
    for (const auto& itr : filter_map_)
      result.emplace_back(itr.first);

    return result;
  }

  void CreateFilterCollector(
      std::string_view func_name, const std::vector<std::string>& filter_name_vec) {
    if (filter_name_vec.empty()) [[unlikely]]
      return;

    auto collector_ptr = std::make_unique<FrameworkAsyncRpcFilterCollector>();

    for (const auto& name : filter_name_vec) {
      auto find_itr = filter_map_.find(name);
      AIMRT_ASSERT(find_itr != filter_map_.end(), "Can not find filter: {}", name);

      collector_ptr->RegisterFilter(find_itr->second);
    }

    auto emplace_ret = filter_collector_map_.emplace(func_name, std::move(collector_ptr));

    AIMRT_ASSERT(emplace_ret.second, "Func {} set filter collector failed.", func_name);

    filter_names_map_.emplace(func_name, filter_name_vec);
  }

  void CreateFilterCollectorIfNotExist(
      std::string_view topic_name, const std::vector<std::string>& filter_name_vec) {
    if (filter_collector_map_.find(topic_name) == filter_collector_map_.end())
      CreateFilterCollector(topic_name, filter_name_vec);
  }

  const FrameworkAsyncRpcFilterCollector& GetFilterCollector(std::string_view func_name) const {
    auto find_itr = filter_collector_map_.find(func_name);
    if (find_itr != filter_collector_map_.end()) {
      return *(find_itr->second);
    }

    return default_filter_collector_;
  }

  std::vector<std::string> GetFilterNameVec(std::string_view func_name) const {
    auto find_itr = filter_names_map_.find(func_name);
    if (find_itr != filter_names_map_.end()) {
      return find_itr->second;
    }

    return {};
  }

  void Clear() {
    filter_collector_map_.clear();
    filter_map_.clear();
  }

 private:
  // filter name - filter
  std::unordered_map<std::string, FrameworkAsyncRpcFilter> filter_map_;

  // func name - filter collector
  using FilterCollectorMap = std::unordered_map<
      std::string,
      std::unique_ptr<FrameworkAsyncRpcFilterCollector>,
      aimrt::common::util::StringHash,
      std::equal_to<>>;
  FilterCollectorMap filter_collector_map_;

  // func name - filter name vec
  using FilterNameMap = std::unordered_map<
      std::string,
      std::vector<std::string>,
      aimrt::common::util::StringHash,
      std::equal_to<>>;
  FilterNameMap filter_names_map_;

  FrameworkAsyncRpcFilterCollector default_filter_collector_;
};

}  // namespace aimrt::runtime::core::rpc