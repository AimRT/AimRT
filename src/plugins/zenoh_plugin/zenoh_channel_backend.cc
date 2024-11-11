// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "zenoh_plugin/zenoh_channel_backend.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::zenoh_plugin::ZenohChannelBackend::Options> {
  using Options = aimrt::plugins::zenoh_plugin::ZenohChannelBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["pub_topics_options"] = YAML::Node();
    for (const auto& pub_topic_options : rhs.pub_topics_options) {
      Node pub_topic_options_node;
      pub_topic_options_node["topic_name"] = pub_topic_options.topic_name;
      pub_topic_options_node["shm_enabled"] = pub_topic_options.shm_enabled;
      node["pub_topics_options"].push_back(pub_topic_options_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (node["pub_topics_options"] && node["pub_topics_options"].IsSequence()) {
      for (const auto& pub_topic_options_node : node["pub_topics_options"]) {
        auto pub_topic_options = Options::PubTopicOptions{
            .topic_name = pub_topic_options_node["topic_name"].as<std::string>(),
            .shm_enabled = pub_topic_options_node["shm_enabled"].as<bool>(),
        };

        rhs.pub_topics_options.emplace_back(std::move(pub_topic_options));
      }
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::zenoh_plugin {

void ZenohChannelBackend::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Zenoh channel backend can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  options_node = options_;
}

void ZenohChannelBackend::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");
}

void ZenohChannelBackend::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;
}

bool ZenohChannelBackend::RegisterPublishType(
    const runtime::core::channel::PublishTypeWrapper& publish_type_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kInit,
                            "Method can only be called when state is 'Init'.");

    const auto& info = publish_type_wrapper.info;

    bool shm_enabled = false;

    auto find_option_itr = std::find_if(
        options_.pub_topics_options.begin(), options_.pub_topics_options.end(),
        [topic_name = info.topic_name](const Options::PubTopicOptions& pub_option) {
          try {
            return std::regex_match(topic_name.begin(), topic_name.end(), std::regex(pub_option.topic_name, std::regex::ECMAScript));
          } catch (const std::exception& e) {
            AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                       pub_option.topic_name, topic_name, e.what());
            return false;
          }
        });

    if (find_option_itr != options_.pub_topics_options.end()) {
      shm_enabled = find_option_itr->shm_enabled;
    }

    namespace util = aimrt::common::util;
    std::string pattern = std::string("channel/") +
                          util::UrlEncode(info.topic_name) + "/" +
                          util::UrlEncode(info.msg_type) +
                          limit_domain_;

    zenoh_manager_ptr_->RegisterPublisher(pattern, shm_enabled);

    AIMRT_INFO("Register publish type to zenoh channel, url: {}, shm_enabled: {}", pattern, shm_enabled);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

bool ZenohChannelBackend::Subscribe(
    const runtime::core::channel::SubscribeWrapper& subscribe_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kInit,
                            "Method can only be called when state is 'Init'.");

    const auto& info = subscribe_wrapper.info;
    namespace util = aimrt::common::util;

    std::string pattern = std::string("channel/") +
                          util::UrlEncode(info.topic_name) + "/" +
                          util::UrlEncode(info.msg_type) +
                          limit_domain_;

    auto find_itr = subscribe_wrapper_map_.find(pattern);
    if (find_itr != subscribe_wrapper_map_.end()) {
      find_itr->second->AddSubscribeWrapper(&subscribe_wrapper);
      return true;
    }

    auto sub_tool_unique_ptr = std::make_unique<aimrt::runtime::core::channel::SubscribeTool>();
    sub_tool_unique_ptr->AddSubscribeWrapper(&subscribe_wrapper);

    auto* sub_tool_ptr = sub_tool_unique_ptr.get();

    subscribe_wrapper_map_.emplace(pattern, std::move(sub_tool_unique_ptr));

    auto handle =
        [this, topic_name = info.topic_name, sub_tool_ptr](const z_loaned_sample_t* message) {
          try {
            auto ctx_ptr = std::make_shared<aimrt::channel::Context>(aimrt_channel_context_type_t::AIMRT_CHANNEL_SUBSCRIBER_CONTEXT);

            const z_loaned_bytes_t* payload = z_sample_payload(message);
            size_t serialized_size = z_bytes_len(payload);
            z_bytes_reader_t reader = z_bytes_get_reader(payload);
            std::vector<char> serialized_data(serialized_size);

            if (z_bytes_reader_read(&reader, reinterpret_cast<uint8_t*>(serialized_data.data()), serialized_size) >= 0) {
              util::ConstBufferOperator buf_oper(serialized_data.data(), serialized_size);
              std::string serialization_type(buf_oper.GetString(util::BufferLenType::kUInt8));
              ctx_ptr->SetSerializationType(serialization_type);

              size_t ctx_num = buf_oper.GetUint8();
              for (size_t ii = 0; ii < ctx_num; ++ii) {
                auto key = buf_oper.GetString(util::BufferLenType::kUInt16);
                auto val = buf_oper.GetString(util::BufferLenType::kUInt16);
                ctx_ptr->SetMetaValue(key, val);
              }

              ctx_ptr->SetMetaValue(AIMRT_CHANNEL_CONTEXT_KEY_BACKEND, Name());

              auto remaining_buf = buf_oper.GetRemainingBuffer();

              sub_tool_ptr->DoSubscribeCallback(
                  ctx_ptr, serialization_type, static_cast<const void*>(remaining_buf.data()), remaining_buf.size());

            } else {
              AIMRT_ERROR("Zenoh Plugin Read payload failed!");
            }
          } catch (const std::exception& e) {
            AIMRT_WARN("Handle Zenoh channel msg failed, exception info: {}", e.what());
          }
        };

    zenoh_manager_ptr_->RegisterSubscriber(pattern, std::move(handle));
    AIMRT_INFO("Register subscribe type to zenoh channel, url: {}", pattern);
    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

void ZenohChannelBackend::Publish(runtime::core::channel::MsgWrapper& msg_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kStart,
                            "Method can only be called when state is 'Start'.");

    namespace util = aimrt::common::util;
    const auto& info = msg_wrapper.info;

    auto publish_type_support_ref = info.msg_type_support_ref;

    auto serialization_type = msg_wrapper.ctx_ref.GetSerializationType();
    if (serialization_type.empty()) {
      serialization_type = publish_type_support_ref.DefaultSerializationType();
    }

    auto buffer_array_view_ptr = aimrt::runtime::core::channel::SerializeMsgWithCache(msg_wrapper, serialization_type);
    AIMRT_CHECK_ERROR_THROW(
        buffer_array_view_ptr,
        "Msg serialization failed, serialization_type {}, pkg_path: {}, module_name: {}, topic_name: {}, msg_type: {}",
        serialization_type, info.pkg_path, info.module_name, info.topic_name, info.msg_type);

    const auto* buffer_array_data = buffer_array_view_ptr->Data();
    const size_t buffer_array_len = buffer_array_view_ptr->Size();
    size_t msg_size = buffer_array_view_ptr->BufferSize();

    const auto& keys = msg_wrapper.ctx_ref.GetMetaKeys();
    AIMRT_CHECK_ERROR_THROW(keys.size() <= 255,
                            "Too much context meta, require less than 255, but actually {}.", keys.size());

    std::vector<std::string_view> context_meta_kv;
    size_t context_meta_kv_size = 1;
    for (const auto& key : keys) {
      context_meta_kv_size += (2 + key.size());
      context_meta_kv.emplace_back(key);

      auto val = msg_wrapper.ctx_ref.GetMetaValue(key);
      context_meta_kv_size += (2 + val.size());
      context_meta_kv.emplace_back(val);
    }

    int32_t pkg_size = 1 + serialization_type.size() + context_meta_kv_size + msg_size;

    std::vector<char> serialized_data(pkg_size);
    util::BufferOperator buf_oper(serialized_data.data(), pkg_size);
    buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);

    buf_oper.SetUint8(static_cast<uint8_t>(keys.size()));
    for (const auto& s : context_meta_kv) {
      buf_oper.SetString(s, util::BufferLenType::kUInt16);
    }

    for (size_t ii = 0; ii < buffer_array_len; ++ii) {
      buf_oper.SetBuffer(
          static_cast<const char*>(buffer_array_data[ii].data),
          buffer_array_data[ii].len);
    }

    std::string zenoh_pub_topic = std::string("channel/") +
                                  util::UrlEncode(info.topic_name) + "/" +
                                  util::UrlEncode(info.msg_type) +
                                  limit_domain_;

    AIMRT_TRACE("Zenoh publish to '{}'", zenoh_pub_topic);

    zenoh_manager_ptr_->Publish(zenoh_pub_topic, serialized_data.data(), pkg_size);

    return;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

}  // namespace aimrt::plugins::zenoh_plugin