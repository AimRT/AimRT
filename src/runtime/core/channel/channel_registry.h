// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

#include "core/channel/channel_msg_wrapper.h"
#include "util/log_util.h"

namespace aimrt::runtime::core::channel {

using SubscriberCallback = std::function<void(MsgWrapper&, std::function<void()>&&)>;

struct SubscribeWrapper {
  TopicInfo info;
  std::unordered_set<std::string> require_cache_serialization_types;
  SubscriberCallback callback;
};

struct PublishTypeWrapper {
  TopicInfo info;
  std::unordered_set<std::string> require_cache_serialization_types;
};

class ChannelRegistry {
 public:
  ChannelRegistry()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~ChannelRegistry() = default;

  ChannelRegistry(const ChannelRegistry&) = delete;
  ChannelRegistry& operator=(const ChannelRegistry&) = delete;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  bool Subscribe(std::unique_ptr<SubscribeWrapper>&& subscribe_wrapper_ptr);

  bool RegisterPublishType(
      std::unique_ptr<PublishTypeWrapper>&& publish_type_wrapper_ptr);

  const SubscribeWrapper* GetSubscribeWrapperPtr(
      std::string_view msg_type,
      std::string_view topic_name,
      std::string_view pkg_path,
      std::string_view module_name) const;

  using ModuleSubscribeWrapperMap = std::unordered_map<std::string_view, SubscribeWrapper*>;
  const ModuleSubscribeWrapperMap* GetModuleSubscribeWrapperMapPtr(
      std::string_view msg_type,
      std::string_view topic_name,
      std::string_view pkg_path) const;

  const PublishTypeWrapper* GetPublishTypeWrapperPtr(
      std::string_view msg_type,
      std::string_view topic_name,
      std::string_view pkg_path,
      std::string_view module_name) const;

  const auto& GetSubscribeWrapperMap() const { return subscribe_wrapper_map_; }
  const auto& GetPublishTypeWrapperMap() const { return publish_type_wrapper_map_; }

  const auto& GetPubTopicIndexMap() const { return pub_topic_index_map_; }
  const auto& GetSubTopicIndexMap() const { return sub_topic_index_map_; }

 private:
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  struct Key {
    std::string_view msg_type;
    std::string_view topic_name;
    std::string_view pkg_path;
    std::string_view module_name;

    bool operator==(const Key& rhs) const {
      return msg_type == rhs.msg_type &&
             topic_name == rhs.topic_name &&
             pkg_path == rhs.pkg_path &&
             module_name == rhs.module_name;
    }

    struct Hash {
      std::size_t operator()(const Key& k) const {
        return (std::hash<std::string_view>()(k.msg_type)) ^
               (std::hash<std::string_view>()(k.topic_name)) ^
               (std::hash<std::string_view>()(k.pkg_path)) ^
               (std::hash<std::string_view>()(k.module_name));
      }
    };
  };

  struct MTPKey {
    std::string_view msg_type;
    std::string_view topic_name;
    std::string_view pkg_path;

    bool operator==(const MTPKey& rhs) const {
      return msg_type == rhs.msg_type &&
             topic_name == rhs.topic_name &&
             pkg_path == rhs.pkg_path;
    }

    struct Hash {
      std::size_t operator()(const MTPKey& k) const {
        return (std::hash<std::string_view>()(k.msg_type)) ^
               (std::hash<std::string_view>()(k.topic_name)) ^
               (std::hash<std::string_view>()(k.pkg_path));
      }
    };
  };

  std::unordered_map<Key, std::unique_ptr<PublishTypeWrapper>, Key::Hash>
      publish_type_wrapper_map_;

  // index map, topic:wrapper
  std::unordered_map<std::string_view, std::vector<PublishTypeWrapper*>>
      pub_topic_index_map_;

  std::unordered_map<Key, std::unique_ptr<SubscribeWrapper>, Key::Hash>
      subscribe_wrapper_map_;

  // index map, topic:wrapper
  std::unordered_map<std::string_view, std::vector<SubscribeWrapper*>>
      sub_topic_index_map_;

  // index map, msg-topic-pkg:module:wrapper
  std::unordered_map<MTPKey, ModuleSubscribeWrapperMap, MTPKey::Hash>
      sub_msg_topic_pkg_index_map_;
};

}  // namespace aimrt::runtime::core::channel