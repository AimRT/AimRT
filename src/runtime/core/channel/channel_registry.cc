// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/channel/channel_registry.h"

namespace aimrt::runtime::core::channel {

bool ChannelRegistry::RegisterPublishType(
    std::unique_ptr<PublishTypeWrapper>&& publish_type_wrapper_ptr) {
  const auto& info = publish_type_wrapper_ptr->info;

  Key key{
      .msg_type = info.msg_type,
      .topic_name = info.topic_name,
      .pkg_path = info.pkg_path,
      .module_name = info.module_name};

  auto emplace_ret = publish_type_wrapper_map_.try_emplace(
      key, std::move(publish_type_wrapper_ptr));

  if (!emplace_ret.second) [[unlikely]] {
    AIMRT_WARN(
        "Publish msg type '{}' is registered repeatedly, topic '{}', module '{}', pkg path '{}'",
        key.msg_type, key.topic_name, key.module_name, key.pkg_path);
    return false;
  }

  pub_topic_index_map_[key.topic_name].emplace_back(emplace_ret.first->second.get());

  AIMRT_TRACE(
      "Publish msg type '{}' is successfully registered, topic '{}', module '{}', pkg path '{}'",
      key.msg_type, key.topic_name, key.module_name, key.pkg_path);

  return true;
}

bool ChannelRegistry::Subscribe(
    std::unique_ptr<SubscribeWrapper>&& subscribe_wrapper_ptr) {
  const auto& info = subscribe_wrapper_ptr->info;

  Key key{
      .msg_type = info.msg_type,
      .topic_name = info.topic_name,
      .pkg_path = info.pkg_path,
      .module_name = info.module_name};

  auto emplace_ret = subscribe_wrapper_map_.try_emplace(
      key, std::move(subscribe_wrapper_ptr));

  if (!emplace_ret.second) [[unlikely]] {
    AIMRT_WARN(
        "Msg type '{}' is subscribed repeatedly, topic '{}', module '{}', pkg path '{}'",
        key.msg_type, key.topic_name, key.module_name, key.pkg_path);
    return false;
  }

  sub_topic_index_map_[key.topic_name].emplace_back(emplace_ret.first->second.get());

  MTPKey m_t_p_key{
      .msg_type = key.msg_type,
      .topic_name = key.topic_name,
      .pkg_path = key.pkg_path};

  sub_msg_topic_pkg_index_map_[m_t_p_key][key.module_name] = emplace_ret.first->second.get();

  AIMRT_TRACE(
      "Msg type '{}' is successfully subscribed, topic '{}', module '{}', pkg path '{}'",
      key.msg_type, key.topic_name, key.module_name, key.pkg_path);

  return true;
}

const SubscribeWrapper* ChannelRegistry::GetSubscribeWrapperPtr(
    std::string_view msg_type,
    std::string_view topic_name,
    std::string_view pkg_path,
    std::string_view module_name) const {
  auto find_itr = subscribe_wrapper_map_.find(
      Key{.msg_type = msg_type, .topic_name = topic_name, .pkg_path = pkg_path, .module_name = module_name});

  if (find_itr != subscribe_wrapper_map_.end())
    return find_itr->second.get();

  return nullptr;
}

const ChannelRegistry::ModuleSubscribeWrapperMap* ChannelRegistry::GetModuleSubscribeWrapperMapPtr(
    std::string_view msg_type,
    std::string_view topic_name,
    std::string_view pkg_path) const {
  auto find_itr = sub_msg_topic_pkg_index_map_.find(
      MTPKey{.msg_type = msg_type, .topic_name = topic_name, .pkg_path = pkg_path});

  if (find_itr != sub_msg_topic_pkg_index_map_.end())
    return &(find_itr->second);

  return nullptr;
}

const PublishTypeWrapper* ChannelRegistry::GetPublishTypeWrapperPtr(
    std::string_view msg_type,
    std::string_view topic_name,
    std::string_view pkg_path,
    std::string_view module_name) const {
  auto find_itr = publish_type_wrapper_map_.find(
      Key{.msg_type = msg_type, .topic_name = topic_name, .pkg_path = pkg_path, .module_name = module_name});

  if (find_itr != publish_type_wrapper_map_.end())
    return find_itr->second.get();

  return nullptr;
}

}  // namespace aimrt::runtime::core::channel
