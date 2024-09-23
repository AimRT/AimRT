// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <functional>

#include "core/channel/channel_msg_wrapper.h"

namespace aimrt::runtime::core::channel {

using FrameworkAsyncChannelHandle = std::function<void(MsgWrapper&)>;
using FrameworkAsyncChannelFilter = std::function<void(MsgWrapper&, FrameworkAsyncChannelHandle&&)>;

class FrameworkAsyncChannelFilterCollector {
 public:
  FrameworkAsyncChannelFilterCollector()
      : final_filter_(
            [](MsgWrapper& msg_wrapper, FrameworkAsyncChannelHandle&& h) {
              h(msg_wrapper);
            }) {}
  ~FrameworkAsyncChannelFilterCollector() = default;

  FrameworkAsyncChannelFilterCollector(const FrameworkAsyncChannelFilterCollector&) = delete;
  FrameworkAsyncChannelFilterCollector& operator=(const FrameworkAsyncChannelFilterCollector&) = delete;

  void RegisterFilter(const FrameworkAsyncChannelFilter& filter) {
    final_filter_ =
        [final_filter{std::move(final_filter_)}, &filter](
            MsgWrapper& msg_wrapper, FrameworkAsyncChannelHandle&& h) {
          filter(
              msg_wrapper,
              [&final_filter, h{std::move(h)}](MsgWrapper& msg_wrapper) mutable {
                final_filter(msg_wrapper, std::move(h));
              });
        };
  }

  void InvokeChannel(FrameworkAsyncChannelHandle&& h, MsgWrapper& msg_wrapper) const {
    final_filter_(msg_wrapper, std::move(h));
  }

  void Clear() {
    final_filter_ = FrameworkAsyncChannelFilter();
  }

 private:
  FrameworkAsyncChannelFilter final_filter_;
};

class FrameworkAsyncChannelFilterManager {
 public:
  FrameworkAsyncChannelFilterManager() = default;
  ~FrameworkAsyncChannelFilterManager() = default;

  FrameworkAsyncChannelFilterManager(const FrameworkAsyncChannelFilterManager&) = delete;
  FrameworkAsyncChannelFilterManager& operator=(const FrameworkAsyncChannelFilterManager&) = delete;

  void RegisterFilter(std::string_view name, FrameworkAsyncChannelFilter&& filter) {
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
      std::string_view topic_name, const std::vector<std::string>& filter_name_vec) {
    if (filter_name_vec.empty()) [[unlikely]]
      return;

    auto collector_ptr = std::make_unique<FrameworkAsyncChannelFilterCollector>();

    for (const auto& name : filter_name_vec) {
      auto find_itr = filter_map_.find(name);
      AIMRT_ASSERT(find_itr != filter_map_.end(), "Can not find filter: {}", name);

      collector_ptr->RegisterFilter(find_itr->second);
    }

    auto emplace_ret = filter_collector_map_.emplace(topic_name, std::move(collector_ptr));

    AIMRT_ASSERT(emplace_ret.second, "Func {} set filter collector failed.", topic_name);

    filter_names_map_.emplace(topic_name, filter_name_vec);
  }

  void CreateFilterCollectorIfNotExist(
      std::string_view topic_name, const std::vector<std::string>& filter_name_vec) {
    if (filter_collector_map_.find(topic_name) == filter_collector_map_.end())
      CreateFilterCollector(topic_name, filter_name_vec);
  }

  const FrameworkAsyncChannelFilterCollector& GetFilterCollector(std::string_view topic_name) const {
    auto find_itr = filter_collector_map_.find(topic_name);
    if (find_itr != filter_collector_map_.end()) {
      return *(find_itr->second);
    }

    return default_filter_collector_;
  }

  std::vector<std::string> GetFilterNameVec(std::string_view topic_name) const {
    auto find_itr = filter_names_map_.find(topic_name);
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
  std::unordered_map<std::string, FrameworkAsyncChannelFilter> filter_map_;

  // topic name - filter collector
  using FilterCollectorMap = std::unordered_map<
      std::string,
      std::unique_ptr<FrameworkAsyncChannelFilterCollector>,
      aimrt::common::util::StringHash,
      std::equal_to<>>;
  FilterCollectorMap filter_collector_map_;

  // topic name - filter name vec
  using FilterNameMap = std::unordered_map<
      std::string,
      std::vector<std::string>,
      aimrt::common::util::StringHash,
      std::equal_to<>>;
  FilterNameMap filter_names_map_;

  FrameworkAsyncChannelFilterCollector default_filter_collector_;
};

}  // namespace aimrt::runtime::core::channel