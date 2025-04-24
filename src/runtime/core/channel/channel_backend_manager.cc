// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/channel/channel_backend_manager.h"

#include <regex>
#include <vector>

#include "aimrt_module_cpp_interface/channel/channel_handle.h"
#include "core/channel/channel_backend_tools.h"

namespace aimrt::runtime::core::channel {

void ChannelBackendManager::Initialize() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Channel backend manager can only be initialized once.");
}

void ChannelBackendManager::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");

  for (auto& backend : channel_backend_index_vec_) {
    AIMRT_TRACE("Start channel backend '{}'.", backend->Name());
    backend->Start();
  }
}

void ChannelBackendManager::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  for (auto& backend : channel_backend_index_vec_) {
    AIMRT_TRACE("Shutdown channel backend '{}'.", backend->Name());
    backend->Shutdown();
  }
}

void ChannelBackendManager::SetChannelRegistry(ChannelRegistry* channel_registry_ptr) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  channel_registry_ptr_ = channel_registry_ptr;
}

void ChannelBackendManager::SetPublishFrameworkAsyncChannelFilterManager(FrameworkAsyncChannelFilterManager* ptr) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  publish_filter_manager_ptr_ = ptr;
}

void ChannelBackendManager::SetSubscribeFrameworkAsyncChannelFilterManager(FrameworkAsyncChannelFilterManager* ptr) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  subscribe_filter_manager_ptr_ = ptr;
}

void ChannelBackendManager::SetPublishFiltersRules(
    const std::vector<std::pair<std::string, std::vector<std::string>>>& rules) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  publish_filters_rules_ = rules;
}

void ChannelBackendManager::SetSubscribeFiltersRules(
    const std::vector<std::pair<std::string, std::vector<std::string>>>& rules) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  subscribe_filters_rules_ = rules;
}

void ChannelBackendManager::SetPubTopicsBackendsRules(
    const std::vector<std::pair<std::string, std::vector<std::string>>>& rules) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  pub_topics_backends_rules_ = rules;
}

void ChannelBackendManager::SetSubTopicsBackendsRules(
    const std::vector<std::pair<std::string, std::vector<std::string>>>& rules) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  sub_topics_backends_rules_ = rules;
}

void ChannelBackendManager::RegisterChannelBackend(
    ChannelBackendBase* channel_backend_ptr) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  channel_backend_index_vec_.emplace_back(channel_backend_ptr);
}

bool ChannelBackendManager::Subscribe(SubscribeProxyInfoWrapper&& wrapper) {
  if (state_.load() != State::kInit) {
    AIMRT_ERROR("Msg can only be subscribed when state is 'Init'.");
    return false;
  }

  auto topic_name = wrapper.topic_name;
  auto msg_type_support_ref = aimrt::util::TypeSupportRef(wrapper.msg_type_support);
  auto msg_type = msg_type_support_ref.TypeName();

  // create sub wrapper
  auto sub_wrapper_ptr = std::make_unique<SubscribeWrapper>();
  sub_wrapper_ptr->info = TopicInfo{
      .msg_type = std::string(msg_type),
      .topic_name = std::string(topic_name),
      .pkg_path = std::string(wrapper.pkg_path),
      .module_name = std::string(wrapper.module_name),
      .msg_type_support_ref = msg_type_support_ref};

  // create filter
  auto filter_name_vec = GetFilterRules(topic_name, subscribe_filters_rules_);
  subscribe_filter_manager_ptr_->CreateFilterCollectorIfNotExist(topic_name, filter_name_vec);

  // set callback
  const auto& filter_collector = subscribe_filter_manager_ptr_->GetFilterCollector(topic_name);

  auto sub_func_shared_ptr = std::make_shared<aimrt::channel::SubscriberCallback>(wrapper.callback);

  sub_wrapper_ptr->callback =
      [this, &filter_collector, sub_func_shared_ptr](
          MsgWrapper& msg_wrapper, std::function<void()>&& input_release_callback) {
        auto release_callback_shared_ptr = std::shared_ptr<std::function<void()>>(
            new std::function<void()>(std::move(input_release_callback)),
            [](std::function<void()>* f) {
              (*f)();
              delete f;
            });

        filter_collector.InvokeChannel(
            [this,
             sub_func_ptr = sub_func_shared_ptr.get(),
             release_callback_shared_ptr](MsgWrapper& msg_wrapper) {
              try {
                CheckMsg(msg_wrapper);

                aimrt::channel::SubscriberReleaseCallback release_callback(
                    [release_callback_shared_ptr, msg_cache_ptr{msg_wrapper.msg_cache_ptr}]() {});

                (*sub_func_ptr)(
                    msg_wrapper.ctx_ref.NativeHandle(),
                    msg_wrapper.msg_ptr,
                    release_callback.NativeHandle());

              } catch (const std::exception& e) {
                AIMRT_ERROR("{}", e.what());
              }
            },
            msg_wrapper);
      };

  // register sub wrapper
  const auto& sub_wrapper_ref = *sub_wrapper_ptr;

  if (!channel_registry_ptr_->Subscribe(std::move(sub_wrapper_ptr)))
    return false;

  auto backend_itr = sub_topics_backend_index_map_.find(topic_name);
  if (backend_itr == sub_topics_backend_index_map_.end()) {
    auto backend_ptr_vec = GetBackendsByRules(topic_name, sub_topics_backends_rules_);
    auto emplace_ret = sub_topics_backend_index_map_.emplace(topic_name, std::move(backend_ptr_vec));
    backend_itr = emplace_ret.first;
  }

  bool ret = true;
  for (auto& itr : backend_itr->second) {
    AIMRT_TRACE("Subscribe type '{}' for topic '{}' to backend '{}'.",
                msg_type, topic_name, itr->Name());
    ret &= itr->Subscribe(sub_wrapper_ref);
  }
  return ret;
}

bool ChannelBackendManager::RegisterPublishType(
    RegisterPublishTypeProxyInfoWrapper&& wrapper) {
  if (state_.load() != State::kInit) {
    AIMRT_ERROR("Publish type can only be registered when state is 'Init'.");
    return false;
  }

  auto topic_name = wrapper.topic_name;
  auto msg_type_support_ref = aimrt::util::TypeSupportRef(wrapper.msg_type_support);
  auto msg_type = msg_type_support_ref.TypeName();

  // create pub wrapper
  auto pub_type_wrapper_ptr = std::make_unique<PublishTypeWrapper>();
  pub_type_wrapper_ptr->info = TopicInfo{
      .msg_type = std::string(msg_type),
      .topic_name = std::string(topic_name),
      .pkg_path = std::string(wrapper.pkg_path),
      .module_name = std::string(wrapper.module_name),
      .msg_type_support_ref = msg_type_support_ref};

  // create filter
  auto filter_name_vec = GetFilterRules(topic_name, publish_filters_rules_);
  publish_filter_manager_ptr_->CreateFilterCollectorIfNotExist(topic_name, filter_name_vec);

  // register pub wrapper
  const auto& pub_type_wrapper_ref = *pub_type_wrapper_ptr;

  if (!channel_registry_ptr_->RegisterPublishType(std::move(pub_type_wrapper_ptr)))
    return false;

  auto backend_itr = pub_topics_backend_index_map_.find(topic_name);
  if (backend_itr == pub_topics_backend_index_map_.end()) {
    auto backend_ptr_vec = GetBackendsByRules(topic_name, pub_topics_backends_rules_);
    auto emplace_ret = pub_topics_backend_index_map_.emplace(topic_name, std::move(backend_ptr_vec));
    backend_itr = emplace_ret.first;
  }

  bool ret = true;
  for (auto& itr : backend_itr->second) {
    AIMRT_TRACE("Register publish type '{}' for topic '{}' to backend '{}'.",
                msg_type, topic_name, itr->Name());
    ret &= itr->RegisterPublishType(pub_type_wrapper_ref);
  }
  return ret;
}

void ChannelBackendManager::Publish(PublishProxyInfoWrapper&& wrapper) {
  if (state_.load() != State::kStart) [[unlikely]] {
    AIMRT_WARN("Method can only be called when state is 'Start'.");
    return;
  }

  auto msg_type = util::ToStdStringView(wrapper.msg_type);

  aimrt::channel::ContextRef ctx_ref(wrapper.ctx_ptr);

  // Find a registered publish type
  const auto* pub_type_wrapper_ptr = channel_registry_ptr_->GetPublishTypeWrapperPtr(
      msg_type, wrapper.topic_name, wrapper.pkg_path, wrapper.module_name);

  // publish type not registered
  if (pub_type_wrapper_ptr == nullptr) {
    AIMRT_WARN(
        "Publish type unregistered, msg_type: {}, topic_name: {}, pkg_path: {}, module_name: {}.",
        msg_type, wrapper.topic_name, wrapper.pkg_path, wrapper.module_name);
    return;
  }

  // Check ctx
  if (ctx_ref.GetType() != aimrt_channel_context_type_t::AIMRT_CHANNEL_PUBLISHER_CONTEXT ||
      ctx_ref.CheckUsed()) {
    AIMRT_WARN("Publish context has been used!");
    return;
  }

  ctx_ref.SetUsed();

  // Find filter
  const auto& filter_collector = publish_filter_manager_ptr_->GetFilterCollector(wrapper.topic_name);

  // Create a wrapper
  auto publish_msg_wrapper_ptr = std::make_shared<MsgWrapper>(
      MsgWrapper{
          .info = pub_type_wrapper_ptr->info,
          .msg_ptr = wrapper.msg_ptr,
          .ctx_ref = ctx_ref});

  // Start publish
  filter_collector.InvokeChannel(
      [this](MsgWrapper& msg_wrapper) {
        const auto& topic_name = msg_wrapper.info.topic_name;

        auto find_itr = pub_topics_backend_index_map_.find(topic_name);

        if (find_itr == pub_topics_backend_index_map_.end()) [[unlikely]] {
          AIMRT_WARN("Channel msg has no backend, topic: '{}'.", topic_name);
          return;
        }

        for (auto& itr : find_itr->second) {
          AIMRT_TRACE("Publish msg '{}' for topic '{}' to channel backend '{}'",
                      msg_wrapper.info.msg_type, topic_name, itr->Name());
          itr->Publish(msg_wrapper);
        }
      },
      *publish_msg_wrapper_ptr);
}

bool ChannelBackendManager::Subscribe(SubscribeWrapper&& wrapper) {
  if (state_.load() != State::kInit) {
    AIMRT_ERROR("Msg can only be subscribed when state is 'Init'.");
    return false;
  }

  auto sub_wrapper_ptr = std::make_unique<SubscribeWrapper>(std::move(wrapper));

  const auto& topic_name = sub_wrapper_ptr->info.topic_name;
  const auto& msg_type = sub_wrapper_ptr->info.msg_type;

  // create filter
  auto filter_name_vec = GetFilterRules(topic_name, subscribe_filters_rules_);
  subscribe_filter_manager_ptr_->CreateFilterCollectorIfNotExist(topic_name, filter_name_vec);

  // set callback
  const auto& filter_collector = subscribe_filter_manager_ptr_->GetFilterCollector(topic_name);

  sub_wrapper_ptr->callback =
      [this, &filter_collector, callback{std::move(sub_wrapper_ptr->callback)}](
          MsgWrapper& msg_wrapper, std::function<void()>&& input_release_callback) {
        auto release_callback_shared_ptr = std::shared_ptr<std::function<void()>>(
            new std::function<void()>(std::move(input_release_callback)),
            [](std::function<void()>* f) {
              (*f)();
              delete f;
            });

        filter_collector.InvokeChannel(
            [&callback, release_callback_shared_ptr](MsgWrapper& msg_wrapper) {
              callback(
                  msg_wrapper,
                  [release_callback_shared_ptr, msg_cache_ptr{msg_wrapper.msg_cache_ptr}]() {});
            },
            msg_wrapper);
      };

  // register sub wrapper
  const auto& sub_wrapper_ref = *sub_wrapper_ptr;

  if (!channel_registry_ptr_->Subscribe(std::move(sub_wrapper_ptr)))
    return false;

  auto backend_itr = sub_topics_backend_index_map_.find(topic_name);
  if (backend_itr == sub_topics_backend_index_map_.end()) {
    auto backend_ptr_vec = GetBackendsByRules(topic_name, sub_topics_backends_rules_);
    auto emplace_ret = sub_topics_backend_index_map_.emplace(topic_name, std::move(backend_ptr_vec));
    backend_itr = emplace_ret.first;
  }

  bool ret = true;
  for (auto& itr : backend_itr->second) {
    AIMRT_TRACE("Subscribe type '{}' for topic '{}' to backend '{}'.",
                msg_type, topic_name, itr->Name());
    ret &= itr->Subscribe(sub_wrapper_ref);
  }
  return ret;
}

bool ChannelBackendManager::RegisterPublishType(PublishTypeWrapper&& wrapper) {
  if (state_.load() != State::kInit) {
    AIMRT_ERROR("Publish type can only be registered when state is 'Init'.");
    return false;
  }

  auto pub_type_wrapper_ptr = std::make_unique<PublishTypeWrapper>(std::move(wrapper));

  const auto& topic_name = pub_type_wrapper_ptr->info.topic_name;
  const auto& msg_type = pub_type_wrapper_ptr->info.msg_type;

  // create filter
  auto filter_name_vec = GetFilterRules(topic_name, publish_filters_rules_);
  publish_filter_manager_ptr_->CreateFilterCollectorIfNotExist(topic_name, filter_name_vec);

  // register pub wrapper
  const auto& pub_type_wrapper_ref = *pub_type_wrapper_ptr;

  if (!channel_registry_ptr_->RegisterPublishType(std::move(pub_type_wrapper_ptr)))
    return false;

  auto backend_itr = pub_topics_backend_index_map_.find(topic_name);
  if (backend_itr == pub_topics_backend_index_map_.end()) {
    auto backend_ptr_vec = GetBackendsByRules(topic_name, pub_topics_backends_rules_);
    auto emplace_ret = pub_topics_backend_index_map_.emplace(topic_name, std::move(backend_ptr_vec));
    backend_itr = emplace_ret.first;
  }

  bool ret = true;
  for (auto& itr : backend_itr->second) {
    AIMRT_TRACE("Register publish type '{}' for topic '{}' to backend '{}'.",
                msg_type, topic_name, itr->Name());
    ret &= itr->RegisterPublishType(pub_type_wrapper_ref);
  }
  return ret;
}

void ChannelBackendManager::Publish(MsgWrapper&& wrapper) {
  if (state_.load() != State::kStart) [[unlikely]] {
    AIMRT_WARN("Method can only be called when state is 'Start'.");
    return;
  }

  auto publish_msg_wrapper_ptr = std::make_shared<MsgWrapper>(std::move(wrapper));

  auto ctx_ref = publish_msg_wrapper_ptr->ctx_ref;

  // Check ctx
  if (ctx_ref.GetType() != aimrt_channel_context_type_t::AIMRT_CHANNEL_PUBLISHER_CONTEXT ||
      ctx_ref.CheckUsed()) {
    AIMRT_WARN("Publish context has been used!");
    return;
  }

  ctx_ref.SetUsed();

  // Find filter
  const auto& filter_collector =
      publish_filter_manager_ptr_->GetFilterCollector(publish_msg_wrapper_ptr->info.topic_name);

  // Start publish
  filter_collector.InvokeChannel(
      [this](MsgWrapper& msg_wrapper) {
        const auto& topic_name = msg_wrapper.info.topic_name;

        auto find_itr = pub_topics_backend_index_map_.find(topic_name);

        if (find_itr == pub_topics_backend_index_map_.end()) [[unlikely]] {
          AIMRT_WARN("Channel msg has no backend, topic: '{}'.", topic_name);
          return;
        }

        for (auto& itr : find_itr->second) {
          AIMRT_TRACE("Publish msg '{}' for topic '{}' to channel backend '{}'",
                      msg_wrapper.info.msg_type, topic_name, itr->Name());
          itr->Publish(msg_wrapper);
        }
      },
      *publish_msg_wrapper_ptr);
}

ChannelBackendManager::TopicBackendInfoMap ChannelBackendManager::GetPubTopicBackendInfo() const {
  std::unordered_map<std::string_view, std::vector<std::string_view>> result;
  for (const auto& itr : pub_topics_backend_index_map_) {
    std::vector<std::string_view> backends_name;
    backends_name.reserve(itr.second.size());
    for (const auto& item : itr.second)
      backends_name.emplace_back(item->Name());

    result.emplace(itr.first, std::move(backends_name));
  }

  return result;
}

ChannelBackendManager::TopicBackendInfoMap ChannelBackendManager::GetSubTopicBackendInfo() const {
  std::unordered_map<std::string_view, std::vector<std::string_view>> result;
  for (const auto& itr : sub_topics_backend_index_map_) {
    std::vector<std::string_view> backends_name;
    backends_name.reserve(itr.second.size());
    for (const auto& item : itr.second)
      backends_name.emplace_back(item->Name());

    result.emplace(itr.first, std::move(backends_name));
  }

  return result;
}

std::vector<ChannelBackendBase*> ChannelBackendManager::GetBackendsByRules(
    std::string_view topic_name,
    const std::vector<std::pair<std::string, std::vector<std::string>>>& rules) {
  for (const auto& item : rules) {
    const auto& topic_regex = item.first;
    const auto& enable_backends = item.second;

    try {
      if (std::regex_match(topic_name.begin(), topic_name.end(), std::regex(topic_regex, std::regex::ECMAScript))) {
        std::vector<ChannelBackendBase*> backend_ptr_vec;

        for (const auto& backend_name : enable_backends) {
          auto itr = std::find_if(
              channel_backend_index_vec_.begin(), channel_backend_index_vec_.end(),
              [&backend_name](const ChannelBackendBase* backend_ptr) -> bool {
                return backend_ptr->Name() == backend_name;
              });

          if (itr == channel_backend_index_vec_.end()) [[unlikely]] {
            AIMRT_WARN("Can not find '{}' in backend list.", backend_name);
            continue;
          }

          backend_ptr_vec.emplace_back(*itr);
        }

        return backend_ptr_vec;
      }
    } catch (const std::exception& e) {
      AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                 topic_regex, topic_name, e.what());
    }
  }

  return {};
}

std::vector<std::string> ChannelBackendManager::GetFilterRules(
    std::string_view topic_name,
    const std::vector<std::pair<std::string, std::vector<std::string>>>& rules) {
  for (const auto& item : rules) {
    const auto& topic_regex = item.first;
    const auto& filters = item.second;

    try {
      if (std::regex_match(topic_name.begin(), topic_name.end(), std::regex(topic_regex, std::regex::ECMAScript))) {
        return filters;
      }
    } catch (const std::exception& e) {
      AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                 topic_regex, topic_name, e.what());
    }
  }

  return {};
}
}  // namespace aimrt::runtime::core::channel
