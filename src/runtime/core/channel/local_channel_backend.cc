// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/channel/local_channel_backend.h"
#include "core/channel/channel_backend_tools.h"

#include <memory>

namespace YAML {
template <>
struct convert<aimrt::runtime::core::channel::LocalChannelBackend::Options> {
  using Options = aimrt::runtime::core::channel::LocalChannelBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["subscriber_use_inline_executor"] = rhs.subscriber_use_inline_executor;
    if (!rhs.subscriber_use_inline_executor)
      node["subscriber_executor"] = rhs.subscriber_executor;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    rhs.subscriber_use_inline_executor = node["subscriber_use_inline_executor"].as<bool>();
    if (!rhs.subscriber_use_inline_executor)
      rhs.subscriber_executor = node["subscriber_executor"].as<std::string>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::runtime::core::channel {

void LocalChannelBackend::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Local channel backend can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  if (!options_.subscriber_use_inline_executor) {
    AIMRT_CHECK_ERROR_THROW(
        get_executor_func_,
        "Get executor function is not set before initialize.");

    subscribe_executor_ref_ = get_executor_func_(options_.subscriber_executor);

    AIMRT_CHECK_ERROR_THROW(
        subscribe_executor_ref_,
        "Invalid local subscriber executor '{}'.", options_.subscriber_executor);
  }

  options_node = options_;
}

void LocalChannelBackend::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");
}

void LocalChannelBackend::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  subscribe_index_map_.clear();

  get_executor_func_ = nullptr;
}

bool LocalChannelBackend::RegisterPublishType(
    const PublishTypeWrapper& publish_type_wrapper) noexcept {
  try {
    if (state_.load() != State::kInit) {
      AIMRT_ERROR("Publish type can only be registered when state is 'Init'.");
      return false;
    }

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

bool LocalChannelBackend::Subscribe(const SubscribeWrapper& subscribe_wrapper) noexcept {
  try {
    if (state_.load() != State::kInit) {
      AIMRT_ERROR("Msg can only be subscribed when state is 'Init'.");
      return false;
    }

    const auto& info = subscribe_wrapper.info;

    subscribe_index_map_[info.msg_type][info.topic_name][info.pkg_path].emplace(info.module_name);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

void LocalChannelBackend::Publish(MsgWrapper& msg_wrapper) noexcept {
  try {
    if (state_.load() != State::kStart) [[unlikely]] {
      AIMRT_WARN("Method can only be called when state is 'Start'.");
      return;
    }

    const auto& pub_info = msg_wrapper.info;

    // Return immediately if there are no subscribers
    auto subscribe_index_map_find_msg_itr = subscribe_index_map_.find(pub_info.msg_type);
    if (subscribe_index_map_find_msg_itr == subscribe_index_map_.end()) [[unlikely]]
      return;

    auto subscribe_index_map_find_topic_itr = subscribe_index_map_find_msg_itr->second.find(pub_info.topic_name);
    if (subscribe_index_map_find_topic_itr == subscribe_index_map_find_msg_itr->second.end()) [[unlikely]]
      return;

    // Determine the serialization type
    auto serialization_type = msg_wrapper.ctx_ref.GetSerializationType();
    if (serialization_type.empty())
      serialization_type = pub_info.msg_type_support_ref.DefaultSerializationType();

    // Iterate through each pkg
    for (const auto& subscribe_pkg_path_itr : subscribe_index_map_find_topic_itr->second) {
      std::string_view cur_sub_pkg_path = subscribe_pkg_path_itr.first;

      const auto* module_sub_wrapper_map_ptr = channel_registry_ptr_->GetModuleSubscribeWrapperMapPtr(
          pub_info.msg_type, pub_info.topic_name, cur_sub_pkg_path);

      if (module_sub_wrapper_map_ptr == nullptr ||
          module_sub_wrapper_map_ptr->empty()) [[unlikely]] {
        AIMRT_ERROR(
            "Can not find registry info, pkg_path: {}, topic_name: {}, msg_type: {}",
            cur_sub_pkg_path, pub_info.topic_name, pub_info.msg_type);
        continue;
      }

      // The creation/destruction methods of the same type should be unified for each module under the same pkg.
      // Select any module's creation/destruction method for this type as the global selection of pkg
      const auto* tpl_sub_wrapper_ptr = module_sub_wrapper_map_ptr->begin()->second;

      auto tpl_sub_info = tpl_sub_wrapper_ptr->info;

      // Test initialization fails when AimRT core is not registered
      auto ctx_ptr = std::make_shared<aimrt::channel::Context>(aimrt_channel_context_type_t::AIMRT_CHANNEL_SUBSCRIBER_CONTEXT);

      const auto& meta_keys = msg_wrapper.ctx_ref.GetMetaKeys();
      for (const auto& item : meta_keys) {
        ctx_ptr->SetMetaValue(item, msg_wrapper.ctx_ref.GetMetaValue(item));
      }

      ctx_ptr->SetMetaValue(AIMRT_CHANNEL_CONTEXT_KEY_BACKEND, Name());

      if (cur_sub_pkg_path == pub_info.pkg_path) {
        // pub and sub are in the same pkg, and copy them directly

        // Create sub msg under this pkg
        std::shared_ptr<void> msg_ptr = tpl_sub_info.msg_type_support_ref.CreateSharedPtr();

        CheckMsg(msg_wrapper);

        tpl_sub_info.msg_type_support_ref.Copy(msg_wrapper.msg_ptr, msg_ptr.get());

        // Call the registered subscribe method
        for (const auto& sub_wrapper_itr : *module_sub_wrapper_map_ptr) {
          const auto* sub_wrapper_ptr = sub_wrapper_itr.second;

          // Create MsgWrapper under this pkg-module
          MsgWrapper sub_msg_warpper{
              .info = sub_wrapper_ptr->info,
              .msg_ptr = msg_ptr.get(),
              .ctx_ref = ctx_ptr,
              .serialization_cache = msg_wrapper.serialization_cache,
              .msg_cache_ptr = msg_ptr};

          if (subscribe_executor_ref_) {
            subscribe_executor_ref_.Execute(
                [sub_wrapper_ptr, ctx_ptr, sub_msg_warpper{std::move(sub_msg_warpper)}]() mutable {
                  sub_wrapper_ptr->callback(sub_msg_warpper, [ctx_ptr]() {});
                });
          } else {
            sub_wrapper_ptr->callback(sub_msg_warpper, [ctx_ptr]() {});
          }
        }

      } else {
        // In different pkgs, pub and sub need to be serialized and deserialized
        // In different modules in the same pkg, the same type structure can be reused, regardless of which serialization method/deserialization method it is transferred from the original structure of the pub side
        // If the serialization type is specified, the specified one is used, otherwise the first of the serialization types supported by the pub side is used.

        ctx_ptr->SetSerializationType(serialization_type);

        // pub end msg serialization
        SerializeMsgWithCache(msg_wrapper, serialization_type);

        // Call the registered subscribe method
        for (const auto& sub_wrapper_itr : *module_sub_wrapper_map_ptr) {
          const auto* sub_wrapper_ptr = sub_wrapper_itr.second;

          // Create MsgWrapper under this pkg-module
          MsgWrapper sub_msg_warpper{
              .info = sub_wrapper_ptr->info,
              .msg_ptr = nullptr,
              .ctx_ref = ctx_ptr,
              .serialization_cache = msg_wrapper.serialization_cache};

          if (subscribe_executor_ref_) {
            subscribe_executor_ref_.Execute(
                [sub_wrapper_ptr, ctx_ptr, sub_msg_warpper{std::move(sub_msg_warpper)}]() mutable {
                  sub_wrapper_ptr->callback(sub_msg_warpper, [ctx_ptr]() {});
                });
          } else {
            sub_wrapper_ptr->callback(sub_msg_warpper, [ctx_ptr]() {});
          }
        }
      }
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

void LocalChannelBackend::RegisterGetExecutorFunc(
    const std::function<executor::ExecutorRef(std::string_view)>& get_executor_func) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  get_executor_func_ = get_executor_func;
}

}  // namespace aimrt::runtime::core::channel