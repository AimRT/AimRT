// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mqtt_plugin/mqtt_channel_backend.h"

#include <regex>

#include "aimrt_module_cpp_interface/util/type_support.h"
#include "mqtt_plugin/global.h"
#include "util/buffer_util.h"
#include "util/url_encode.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::mqtt_plugin::MqttChannelBackend::Options> {
  using Options = aimrt::plugins::mqtt_plugin::MqttChannelBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["pub_topics_options"] = YAML::Node();
    for (const auto& pub_topic_options : rhs.pub_topics_options) {
      Node pub_topic_options_node;
      pub_topic_options_node["topic_name"] = pub_topic_options.topic_name;
      pub_topic_options_node["qos"] = pub_topic_options.qos;
      node["pub_topics_options"].push_back(pub_topic_options_node);
    }

    node["sub_topics_options"] = YAML::Node();
    for (const auto& sub_topic_options : rhs.sub_topics_options) {
      Node sub_topic_options_node;
      sub_topic_options_node["topic_name"] = sub_topic_options.topic_name;
      sub_topic_options_node["qos"] = sub_topic_options.qos;
      node["sub_topics_options"].push_back(sub_topic_options_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (node["pub_topics_options"] && node["pub_topics_options"].IsSequence()) {
      for (const auto& pub_topic_options_node : node["pub_topics_options"]) {
        int qos = 2;

        if (pub_topic_options_node["qos"]) qos = pub_topic_options_node["qos"].as<int>();
        if (qos < 0 || qos > 2)
          throw aimrt::common::util::AimRTException("Invalid Mqtt qos: " + std::to_string(qos));

        auto pub_topic_options = Options::PubTopicOptions{
            .topic_name = pub_topic_options_node["topic_name"].as<std::string>(),
            .qos = qos};

        rhs.pub_topics_options.emplace_back(std::move(pub_topic_options));
      }
    }

    if (node["sub_topics_options"] && node["sub_topics_options"].IsSequence()) {
      for (const auto& sub_topic_options_node : node["sub_topics_options"]) {
        int qos = 2;

        if (sub_topic_options_node["qos"]) qos = sub_topic_options_node["qos"].as<int>();
        if (qos < 0 || qos > 2)
          throw aimrt::common::util::AimRTException("Invalid Mqtt qos: " + std::to_string(qos));

        auto sub_topic_options = Options::SubTopicOptions{
            .topic_name = sub_topic_options_node["topic_name"].as<std::string>(),
            .qos = qos};

        rhs.sub_topics_options.emplace_back(std::move(sub_topic_options));
      }
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::mqtt_plugin {

void MqttChannelBackend::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Mqtt channel backend can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  options_node = options_;
}

void MqttChannelBackend::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");

  // Wait a moment  for the connection to be established
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void MqttChannelBackend::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  UnSubscribeMqttTopic();

  msg_handle_registry_ptr_->Shutdown();
}

bool MqttChannelBackend::RegisterPublishType(
    const runtime::core::channel::PublishTypeWrapper& publish_type_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kInit,
                            "Method can only be called when state is 'Init'.");

    namespace util = aimrt::common::util;

    const auto& info = publish_type_wrapper.info;

    int qos = 2;

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
      qos = find_option_itr->qos;
    }

    pub_cfg_info_map_.emplace(
        info.topic_name,
        PubCfgInfo{
            .qos = qos});

    // 检查path
    std::string pattern = std::string("/channel/") +
                          util::UrlEncode(info.topic_name) + "/" +
                          util::UrlEncode(info.msg_type);

    AIMRT_CHECK_ERROR_THROW(pattern.size() <= 255, "Too long uri: {}", pattern);

    AIMRT_INFO("Register publish type to mqtt channel, url: {}", pattern);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

bool MqttChannelBackend::Subscribe(
    const runtime::core::channel::SubscribeWrapper& subscribe_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kInit,
                            "Method can only be called when state is 'Init'.");

    namespace util = aimrt::common::util;

    const auto& info = subscribe_wrapper.info;

    int qos = 2;

    auto find_option_itr = std::find_if(
        options_.sub_topics_options.begin(), options_.sub_topics_options.end(),
        [topic_name = info.topic_name](const Options::SubTopicOptions& sub_option) {
          try {
            return std::regex_match(topic_name.begin(), topic_name.end(), std::regex(sub_option.topic_name, std::regex::ECMAScript));
          } catch (const std::exception& e) {
            AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                       sub_option.topic_name, topic_name, e.what());
            return false;
          }
        });

    if (find_option_itr != options_.sub_topics_options.end()) {
      qos = find_option_itr->qos;
    }

    std::string pattern = std::string("/channel/") +
                          util::UrlEncode(info.topic_name) + "/" +
                          util::UrlEncode(info.msg_type);

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
        [this, topic_name = info.topic_name, sub_tool_ptr](MQTTAsync_message* message) {
          auto ctx_ptr = std::make_shared<aimrt::channel::Context>(aimrt_channel_context_type_t::AIMRT_CHANNEL_SUBSCRIBER_CONTEXT);

          // 解析mqtt包
          util::ConstBufferOperator buf_oper(static_cast<const char*>(message->payload), message->payloadlen);

          std::string serialization_type(buf_oper.GetString(util::BufferLenType::kUInt8));
          ctx_ptr->SetSerializationType(serialization_type);

          // 获取context
          size_t ctx_num = buf_oper.GetUint8();
          for (size_t ii = 0; ii < ctx_num; ++ii) {
            auto key = buf_oper.GetString(util::BufferLenType::kUInt16);
            auto val = buf_oper.GetString(util::BufferLenType::kUInt16);
            ctx_ptr->SetMetaValue(key, val);
          }

          ctx_ptr->SetMetaValue(AIMRT_CHANNEL_CONTEXT_KEY_BACKEND, Name());

          // 获取消息buf
          auto remaining_buf = buf_oper.GetRemainingBuffer();

          sub_tool_ptr->DoSubscribeCallback(
              ctx_ptr, serialization_type, static_cast<const void*>(remaining_buf.data()), remaining_buf.size());
        };

    msg_handle_registry_ptr_->RegisterMsgHandle(pattern, std::move(handle));

    sub_info_vec_.emplace_back(MqttSubInfo{pattern, qos});

    AIMRT_INFO("Register mqtt handle for channel, uri '{}'", pattern);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

void MqttChannelBackend::Publish(runtime::core::channel::MsgWrapper& msg_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kStart,
                            "Method can only be called when state is 'Start'.");

    namespace util = aimrt::common::util;

    const auto& info = msg_wrapper.info;

    int qos = 2;

    auto find_itr = pub_cfg_info_map_.find(info.topic_name);
    if (find_itr != pub_cfg_info_map_.end()) {
      qos = find_itr->second.qos;
    }

    // 确定数据序列化类型，先找ctx，ctx中未配置则找支持的第一种序列化类型
    auto publish_type_support_ref = info.msg_type_support_ref;

    std::string_view serialization_type = msg_wrapper.ctx_ref.GetSerializationType();
    if (serialization_type.empty()) {
      serialization_type = publish_type_support_ref.DefaultSerializationType();
    }

    // msg序列化
    auto buffer_array_view_ptr = aimrt::runtime::core::channel::SerializeMsgWithCache(msg_wrapper, serialization_type);
    AIMRT_CHECK_ERROR_THROW(
        buffer_array_view_ptr,
        "Msg serialization failed, serialization_type {}, pkg_path: {}, module_name: {}, topic_name: {}, msg_type: {}",
        serialization_type, info.pkg_path, info.module_name, info.topic_name, info.msg_type);

    // 填内容
    const auto* buffer_array_data = buffer_array_view_ptr->Data();
    const size_t buffer_array_len = buffer_array_view_ptr->Size();
    size_t msg_size = buffer_array_view_ptr->BufferSize();

    // context
    auto [meta_key_vals_array, meta_key_vals_array_len] = msg_wrapper.ctx_ref.GetMetaKeyValsArray();
    AIMRT_CHECK_ERROR_THROW(meta_key_vals_array_len / 2 <= 255,
                            "Too much context meta, require less than 255, but actually {}.", meta_key_vals_array_len / 2);

    size_t context_meta_kv_size = 1;
    for (size_t ii = 0; ii < meta_key_vals_array_len; ++ii) {
      context_meta_kv_size += (2 + meta_key_vals_array[ii].len);
    }

    size_t mqtt_pkg_size = 1 + serialization_type.size() + context_meta_kv_size + msg_size;

    AIMRT_CHECK_ERROR_THROW(mqtt_pkg_size <= max_pkg_size_,
                            "Mqtt publish failed, pkg is too large, limit {} bytes, actual {} bytes",
                            max_pkg_size_, mqtt_pkg_size);

    std::vector<char> msg_buf_vec(mqtt_pkg_size);

    util::BufferOperator buf_oper(msg_buf_vec.data(), msg_buf_vec.size());

    buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);

    buf_oper.SetUint8(static_cast<uint8_t>(meta_key_vals_array_len / 2));
    for (size_t ii = 0; ii < meta_key_vals_array_len; ++ii) {
      buf_oper.SetString(aimrt::util::ToStdStringView(meta_key_vals_array[ii]), util::BufferLenType::kUInt16);
    }

    // data
    for (size_t ii = 0; ii < buffer_array_len; ++ii) {
      buf_oper.SetBuffer(
          static_cast<const char*>(buffer_array_data[ii].data),
          buffer_array_data[ii].len);
    }

    MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
    pubmsg.payload = msg_buf_vec.data();
    pubmsg.payloadlen = msg_buf_vec.size();
    pubmsg.qos = qos;
    pubmsg.retained = 0;

    // 确定path
    std::string mqtt_pub_topic = std::string("/channel/") +
                                 util::UrlEncode(info.topic_name) + "/" +
                                 util::UrlEncode(info.msg_type);

    AIMRT_TRACE("Mqtt publish to '{}'", mqtt_pub_topic);
    int rc = MQTTAsync_sendMessage(client_, mqtt_pub_topic.data(), &pubmsg, NULL);
    AIMRT_CHECK_WARN(rc == MQTTASYNC_SUCCESS,
                     "publish mqtt msg failed, topic: {}, code: {}",
                     mqtt_pub_topic, rc);
    return;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

void MqttChannelBackend::SubscribeMqttTopic() {
  for (auto sub_info : sub_info_vec_) {
    // todo:换成MQTTClient_subscribeMany
    int rc = MQTTAsync_subscribe(client_, sub_info.topic.data(), sub_info.qos, NULL);
    if (rc != MQTTASYNC_SUCCESS) {
      AIMRT_ERROR("Failed to subscribe mqtt, topic: {} return code: {}", sub_info.topic, rc);
    }
  }
}

void MqttChannelBackend::UnSubscribeMqttTopic() {
  // todo:换成MQTTClient_unsubscribeMany
  for (auto sub_info : sub_info_vec_) {
    MQTTAsync_unsubscribe(client_, sub_info.topic.data(), NULL);
  }
}
}  // namespace aimrt::plugins::mqtt_plugin
