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

  get_executor_func_ = std::function<executor::ExecutorRef(std::string_view)>();
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

    // 没有人订阅则直接返回
    auto subscribe_index_map_find_msg_itr = subscribe_index_map_.find(pub_info.msg_type);
    if (subscribe_index_map_find_msg_itr == subscribe_index_map_.end()) [[unlikely]]
      return;

    auto subscribe_index_map_find_topic_itr = subscribe_index_map_find_msg_itr->second.find(pub_info.topic_name);
    if (subscribe_index_map_find_topic_itr == subscribe_index_map_find_msg_itr->second.end()) [[unlikely]]
      return;

    // 确定序列化类型
    auto serialization_type = msg_wrapper.ctx_ref.GetSerializationType();
    if (serialization_type.empty())
      serialization_type = pub_info.msg_type_support_ref.DefaultSerializationType();

    // 遍历每个pkg
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

      // 同一个pkg下的各个模块，对同一个类型的创建/销毁方法应该是统一的
      // 随便选取一个模块的对该类型的创建/销毁方法作为pkg的全局选择
      const auto* tpl_sub_wrapper_ptr = module_sub_wrapper_map_ptr->begin()->second;

      auto tpl_sub_info = tpl_sub_wrapper_ptr->info;

      // 创建该pkg下的 context
      auto ctx_ptr = std::make_shared<aimrt::channel::Context>(aimrt_channel_context_type_t::AIMRT_CHANNEL_SUBSCRIBER_CONTEXT);

      const auto& meta_keys = msg_wrapper.ctx_ref.GetMetaKeys();
      for (const auto& item : meta_keys) {
        ctx_ptr->SetMetaValue(item, msg_wrapper.ctx_ref.GetMetaValue(item));
      }

      ctx_ptr->SetMetaValue(AIMRT_CHANNEL_CONTEXT_KEY_BACKEND, Name());

      if (cur_sub_pkg_path == pub_info.pkg_path) {
        // pub和sub在同一个pkg中，直接复制

        // 创建该pkg下的 sub msg
        std::shared_ptr<void> msg_ptr = tpl_sub_info.msg_type_support_ref.CreateSharedPtr();

        CheckMsg(msg_wrapper);

        tpl_sub_info.msg_type_support_ref.Copy(msg_wrapper.msg_ptr, msg_ptr.get());

        // 调用注册的subscribe方法
        for (const auto& sub_wrapper_itr : *module_sub_wrapper_map_ptr) {
          const auto* sub_wrapper_ptr = sub_wrapper_itr.second;

          // 创建该 pkg-module 下的 MsgWrapper
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
        // pub和sub在不同pkg中，需要进行序列化反序列化
        // 在同一个pkg中的不同模块中，对同一个类型结构可以复用，不管它是通过哪种序列化方法/反序列化方法从pub端原始结构转过来的
        // 如果指定了序列化类型，则使用指定的，否则使用pub端支持的序列化类型中的第一种

        ctx_ptr->SetSerializationType(serialization_type);

        // pub 端 msg 序列化
        SerializeMsgWithCache(msg_wrapper, serialization_type);

        // 调用注册的subscribe方法
        for (const auto& sub_wrapper_itr : *module_sub_wrapper_map_ptr) {
          const auto* sub_wrapper_ptr = sub_wrapper_itr.second;

          // 创建该 pkg-module 下的 MsgWrapper
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