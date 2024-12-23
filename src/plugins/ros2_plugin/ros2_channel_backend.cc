// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "ros2_plugin/ros2_channel_backend.h"

#include <regex>

#include "aimrt_module_cpp_interface/util/string.h"
#include "aimrt_module_cpp_interface/util/type_support.h"
#include "ros2_plugin/global.h"
#include "ros2_plugin/ros2_name_encode.h"

#include "rcl/error_handling.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::ros2_plugin::Ros2ChannelBackend::Options> {
  using Options = aimrt::plugins::ros2_plugin::Ros2ChannelBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["pub_topics_options"] = YAML::Node();
    for (const auto& pub_topic_options : rhs.pub_topics_options) {
      Node pub_topic_options_node;
      pub_topic_options_node["topic_name"] = pub_topic_options.topic_name;
      pub_topic_options_node["qos"]["history"] = pub_topic_options.qos.history;
      pub_topic_options_node["qos"]["depth"] = pub_topic_options.qos.depth;
      pub_topic_options_node["qos"]["reliability"] = pub_topic_options.qos.reliability;
      pub_topic_options_node["qos"]["durability"] = pub_topic_options.qos.durability;
      pub_topic_options_node["qos"]["lifespan"] = pub_topic_options.qos.lifespan;
      pub_topic_options_node["qos"]["deadline"] = pub_topic_options.qos.deadline;
      pub_topic_options_node["qos"]["liveliness"] = pub_topic_options.qos.liveliness;
      pub_topic_options_node["qos"]["liveliness_lease_duration"] = pub_topic_options.qos.liveliness_lease_duration;
      node["pub_topics_options"].push_back(pub_topic_options_node);
    }

    node["sub_topics_options"] = YAML::Node();
    for (const auto& sub_topic_options : rhs.sub_topics_options) {
      Node sub_topic_options_node;
      sub_topic_options_node["topic_name"] = sub_topic_options.topic_name;
      sub_topic_options_node["qos"]["history"] = sub_topic_options.qos.history;
      sub_topic_options_node["qos"]["depth"] = sub_topic_options.qos.depth;
      sub_topic_options_node["qos"]["reliability"] = sub_topic_options.qos.reliability;
      sub_topic_options_node["qos"]["durability"] = sub_topic_options.qos.durability;
      sub_topic_options_node["qos"]["lifespan"] = sub_topic_options.qos.lifespan;
      sub_topic_options_node["qos"]["deadline"] = sub_topic_options.qos.deadline;
      sub_topic_options_node["qos"]["liveliness"] = sub_topic_options.qos.liveliness;
      sub_topic_options_node["qos"]["liveliness_lease_duration"] = sub_topic_options.qos.liveliness_lease_duration;
      node["sub_topics_options"].push_back(sub_topic_options_node);
    }

    return node;
  }

  static bool decodeQos(const Node& node, Options::QosOptions& qos) {
    if (node) {
      if (node["history"]) {
        qos.history = node["history"].as<std::string>();
      }
      if (node["depth"]) {
        qos.depth = node["depth"].as<int>();
      }
      if (node["reliability"]) {
        qos.reliability = node["reliability"].as<std::string>();
      }
      if (node["durability"]) {
        qos.durability = node["durability"].as<std::string>();
      }
      if (node["lifespan"]) {
        qos.lifespan = node["lifespan"].as<int>();
      }
      if (node["deadline"]) {
        qos.deadline = node["deadline"].as<int>();
      }
      if (node["liveliness"]) {
        qos.liveliness = node["liveliness"].as<std::string>();
      }
      if (node["liveliness_lease_duration"]) {
        qos.liveliness_lease_duration = node["liveliness_lease_duration"].as<int>();
      }
    }
    return true;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (node["pub_topics_options"] && node["pub_topics_options"].IsSequence()) {
      for (const auto& pub_topic_options_node : node["pub_topics_options"]) {
        auto pub_topic_options = Options::PubTopicOptions{
            .topic_name = pub_topic_options_node["topic_name"].as<std::string>()};
        if (pub_topic_options_node["qos"]) {
          decodeQos(pub_topic_options_node["qos"], pub_topic_options.qos);
        }
        rhs.pub_topics_options.emplace_back(std::move(pub_topic_options));
      }
    }

    if (node["sub_topics_options"] && node["sub_topics_options"].IsSequence()) {
      for (const auto& sub_topic_options_node : node["sub_topics_options"]) {
        auto sub_topic_options = Options::SubTopicOptions{
            .topic_name = sub_topic_options_node["topic_name"].as<std::string>()};
        if (sub_topic_options_node["qos"]) {
          decodeQos(sub_topic_options_node["qos"], sub_topic_options.qos);
        }
        rhs.sub_topics_options.emplace_back(std::move(sub_topic_options));
      }
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::ros2_plugin {

void Ros2ChannelBackend::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Ros2 channel backend can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  options_node = options_;
}

void Ros2ChannelBackend::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");

  for (auto& itr : ros2_subscribe_wrapper_map_) {
    static_cast<Ros2AdapterSubscription*>(itr.second.ros_sub_handle_ptr.get())->Start();
  }
}

void Ros2ChannelBackend::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  for (auto& itr : ros2_publish_type_wrapper_map_) {
    rcl_publisher_t& publisher = *(itr.second);
    rcl_ret_t ret = rcl_publisher_fini(
        &publisher,
        ros2_node_ptr_->get_node_base_interface()->get_shared_rcl_node_handle().get());
    if (ret != RMW_RET_OK) {
      AIMRT_WARN(
          "Publisher fini failed in ros2 channel backend, type '{}', error info: {}",
          itr.first.msg_type, rcl_get_error_string().str);
      rcl_reset_error();
    }
  }

  for (auto& itr : ros2_subscribe_wrapper_map_) {
    static_cast<Ros2AdapterSubscription*>(itr.second.ros_sub_handle_ptr.get())->Shutdown();
  }

  ros2_node_ptr_.reset();
}

bool Ros2ChannelBackend::RegisterPublishType(
    const runtime::core::channel::PublishTypeWrapper& publish_type_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kInit,
                            "Method can only be called when state is 'Init'.");

    const auto& info = publish_type_wrapper.info;

    rclcpp::QoS qos(10);
    // 读取配置中的QOS
    auto find_qos_option = std::find_if(
        options_.pub_topics_options.begin(), options_.pub_topics_options.end(),
        [&info](const Options::PubTopicOptions& pub_option) {
          try {
            return std::regex_match(info.topic_name.begin(), info.topic_name.end(),
                                    std::regex(pub_option.topic_name, std::regex::ECMAScript));
          } catch (const std::exception& e) {
            AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                       pub_option.topic_name, info.topic_name, e.what());
            return false;
          }
        });

    if (find_qos_option != options_.pub_topics_options.end()) {
      qos = GetQos(find_qos_option->qos);
    }

    // 前缀是ros2类型的消息
    if (CheckRosMsg(info.msg_type)) {
      Key key{
          .topic_name = info.topic_name,
          .msg_type = info.msg_type};

      if (ros2_publish_type_wrapper_map_.find(key) != ros2_publish_type_wrapper_map_.end()) {
        return true;
      }

      auto unique_ptr = std::make_unique<rcl_publisher_t>(rcl_get_zero_initialized_publisher());

      rcl_publisher_t* publisher_ptr = unique_ptr.get();
      ros2_publish_type_wrapper_map_.emplace(key, std::move(unique_ptr));

      std::string ros2_topic_name = rclcpp::extend_name_with_sub_namespace(
          info.topic_name,
          ros2_node_ptr_->get_sub_namespace());

      rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
      publisher_options.qos = qos.get_rmw_qos_profile();

      rcl_ret_t ret = rcl_publisher_init(
          publisher_ptr,
          ros2_node_ptr_->get_node_base_interface()->get_shared_rcl_node_handle().get(),
          static_cast<const rosidl_message_type_support_t*>(info.msg_type_support_ref.CustomTypeSupportPtr()),
          ros2_topic_name.c_str(),
          &publisher_options);

      if (ret != RMW_RET_OK) {
        AIMRT_WARN("Ros2 publisher init failed, type '{}', error info: {}",
                   info.msg_type, rcl_get_error_string().str);
        rcl_reset_error();
        return false;
      }

      AIMRT_INFO("ros2 backend register publish type for topic '{}' success.", info.topic_name);

      return true;
    }

    // 前缀不是ros2类型的消息
    std::string real_ros2_topic_name = info.topic_name + "/" + Ros2NameEncode(info.msg_type);

    // 先检查是否注册过了
    if (publisher_map_.find(real_ros2_topic_name) == publisher_map_.end()) {
      publisher_map_.emplace(
          real_ros2_topic_name,
          ros2_node_ptr_->create_publisher<ros2_plugin_proto::msg::RosMsgWrapper>(real_ros2_topic_name, qos));
    }

    AIMRT_INFO("ros2 backend register publish type for topic '{}' success, real ros2 topic name is '{}'.",
               info.topic_name, real_ros2_topic_name);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

bool Ros2ChannelBackend::Subscribe(
    const runtime::core::channel::SubscribeWrapper& subscribe_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kInit,
                            "Method can only be called when state is 'Init'.");

    const auto& info = subscribe_wrapper.info;

    rclcpp::QoS qos(1);
    // 读取配置中的QOS
    auto find_qos_option = std::find_if(
        options_.sub_topics_options.begin(), options_.sub_topics_options.end(),
        [&info](const Options::SubTopicOptions& sub_option) {
          try {
            return std::regex_match(info.topic_name.begin(), info.topic_name.end(),
                                    std::regex(sub_option.topic_name, std::regex::ECMAScript));
          } catch (const std::exception& e) {
            AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                       sub_option.topic_name, info.topic_name, e.what());
            return false;
          }
        });

    if (find_qos_option != options_.sub_topics_options.end()) {
      qos = GetQos(find_qos_option->qos);
    }

    // 前缀是ros2类型的消息
    if (CheckRosMsg(info.msg_type)) {
      Key key{
          .topic_name = info.topic_name,
          .msg_type = info.msg_type};

      auto find_itr = ros2_subscribe_wrapper_map_.find(key);
      if (find_itr != ros2_subscribe_wrapper_map_.end()) {
        find_itr->second.sub_tool_ptr->AddSubscribeWrapper(&subscribe_wrapper);
        return true;
      }

      auto sub_tool_unique_ptr = std::make_unique<aimrt::runtime::core::channel::SubscribeTool>();
      sub_tool_unique_ptr->AddSubscribeWrapper(&subscribe_wrapper);

      auto* sub_tool_ptr = sub_tool_unique_ptr.get();

      std::string ros2_topic_name = rclcpp::extend_name_with_sub_namespace(
          info.topic_name,
          ros2_node_ptr_->get_sub_namespace());

      auto node_topics_interface =
          rclcpp::node_interfaces::get_node_topics_interface(*ros2_node_ptr_);

      rclcpp::SubscriptionFactory factory{
          [&subscribe_wrapper, sub_tool_ptr](
              rclcpp::node_interfaces::NodeBaseInterface* node_base,
              const std::string& topic_name,
              const rclcpp::QoS& qos) -> rclcpp::SubscriptionBase::SharedPtr {
            const rclcpp::SubscriptionOptionsWithAllocator<
                std::allocator<void>>& options =
                rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();

            std::shared_ptr<Ros2AdapterSubscription> subscriber =
                std::make_shared<Ros2AdapterSubscription>(
                    node_base,
                    *static_cast<const rosidl_message_type_support_t*>(subscribe_wrapper.info.msg_type_support_ref.CustomTypeSupportPtr()),
                    topic_name,
                    // todo: ros2的bug，新版本修复后去掉模板参数
                    options.to_rcl_subscription_options<void>(qos),
                    subscribe_wrapper,
                    *sub_tool_ptr);
            return std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(subscriber);
          }};

      auto subscriber = node_topics_interface->create_subscription(ros2_topic_name, factory, qos);
      node_topics_interface->add_subscription(subscriber, nullptr);

      ros2_subscribe_wrapper_map_.emplace(
          key,
          RosSubWrapper{
              .sub_tool_ptr = std::move(sub_tool_unique_ptr),
              .ros_sub_handle_ptr = std::move(subscriber)});

      AIMRT_INFO("subscribe topic '{}' success.", info.topic_name);

      return true;
    }

    // 前缀不是ros2类型的消息
    std::string real_ros2_topic_name = info.topic_name + "/" + Ros2NameEncode(info.msg_type);

    auto find_itr = subscribe_wrapper_map_.find(real_ros2_topic_name);
    if (find_itr != subscribe_wrapper_map_.end()) {
      find_itr->second.sub_tool_ptr->AddSubscribeWrapper(&subscribe_wrapper);
      return true;
    }

    auto sub_tool_unique_ptr = std::make_unique<aimrt::runtime::core::channel::SubscribeTool>();
    sub_tool_unique_ptr->AddSubscribeWrapper(&subscribe_wrapper);

    auto* sub_tool_ptr = sub_tool_unique_ptr.get();

    auto ros_sub_handle_ptr = ros2_node_ptr_->create_subscription<ros2_plugin_proto::msg::RosMsgWrapper>(
        real_ros2_topic_name,
        qos,
        [this, topic_name = info.topic_name, sub_tool_ptr](
            ros2_plugin_proto::msg::RosMsgWrapper::UniquePtr wrapper_msg) {
          try {
            auto ctx_ptr = std::make_shared<aimrt::channel::Context>(aimrt_channel_context_type_t::AIMRT_CHANNEL_SUBSCRIBER_CONTEXT);

            const std::string& serialization_type = wrapper_msg->serialization_type;
            ctx_ptr->SetSerializationType(serialization_type);

            size_t context_size = wrapper_msg->context.size() / 2;
            for (size_t ii = 0; ii < context_size; ++ii) {
              const auto& key = wrapper_msg->context[ii * 2];
              const auto& val = wrapper_msg->context[ii * 2 + 1];
              ctx_ptr->SetMetaValue(key, val);
            }

            ctx_ptr->SetMetaValue(AIMRT_CHANNEL_CONTEXT_KEY_BACKEND, Name());

            sub_tool_ptr->DoSubscribeCallback(
                ctx_ptr, serialization_type, static_cast<const void*>(wrapper_msg->data.data()), wrapper_msg->data.size());

          } catch (const std::exception& e) {
            AIMRT_ERROR("{}", e.what());
          }
        });

    subscribe_wrapper_map_.emplace(
        real_ros2_topic_name,
        SubWrapper{
            .sub_tool_ptr = std::move(sub_tool_unique_ptr),
            .ros_sub_handle_ptr = std::move(ros_sub_handle_ptr)});

    AIMRT_INFO("subscribe topic '{}' success, real ros2 topic name is '{}'.",
               info.topic_name, real_ros2_topic_name);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

void Ros2ChannelBackend::Publish(runtime::core::channel::MsgWrapper& msg_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kStart,
                            "Method can only be called when state is 'Start'.");

    const auto& info = msg_wrapper.info;

    // 前缀是ros2类型的消息
    if (CheckRosMsg(info.msg_type)) {
      Key key{
          .topic_name = info.topic_name,
          .msg_type = info.msg_type};

      auto finditr = ros2_publish_type_wrapper_map_.find(key);
      AIMRT_CHECK_ERROR_THROW(
          finditr != ros2_publish_type_wrapper_map_.end(),
          "Publish msg type '{}' unregistered in ros2 channel backend, topic '{}', module '{}', lib path '{}'",
          info.msg_type, info.topic_name, info.module_name, info.pkg_path);

      rcl_publisher_t& publisher = *(finditr->second);

      aimrt::runtime::core::channel::CheckMsg(msg_wrapper);

      rcl_ret_t ret = rcl_publish(&publisher, msg_wrapper.msg_ptr, nullptr);
      if (ret != RMW_RET_OK) {
        AIMRT_WARN(
            "Publish msg type '{}' failed in ros2 channel backend, topic '{}', module '{}', lib path '{}', error info: {}",
            info.msg_type, info.topic_name, info.module_name, info.pkg_path,
            rcl_get_error_string().str);
        rcl_reset_error();
      }

      return;
    }

    // 前缀不是ros2类型的消息
    std::string real_ros2_topic_name = info.topic_name + "/" + Ros2NameEncode(info.msg_type);

    auto finditr = publisher_map_.find(real_ros2_topic_name);
    AIMRT_CHECK_ERROR_THROW(
        finditr != publisher_map_.end(),
        "Publish msg type '{}' unregistered in ros2 channel backend, topic '{}', module '{}', lib path '{}'",
        info.msg_type, info.topic_name, info.module_name, info.pkg_path);

    auto ros2_publisher_ptr = finditr->second;

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

    // 填内容，直接复制过去
    const auto* buffer_array_data = buffer_array_view_ptr->Data();
    const size_t buffer_array_len = buffer_array_view_ptr->Size();
    size_t msg_size = buffer_array_view_ptr->BufferSize();

    ros2_plugin_proto::msg::RosMsgWrapper wrapper_msg;
    wrapper_msg.serialization_type = serialization_type;

    // context
    const auto& keys = msg_wrapper.ctx_ref.GetMetaKeys();
    wrapper_msg.context.reserve(2 * keys.size());
    for (const auto& key : keys) {
      wrapper_msg.context.emplace_back(key);
      wrapper_msg.context.emplace_back(msg_wrapper.ctx_ref.GetMetaValue(key));
    }

    wrapper_msg.data.resize(msg_size);

    auto* cur_pos = wrapper_msg.data.data();
    for (size_t ii = 0; ii < buffer_array_len; ++ii) {
      memcpy(cur_pos, buffer_array_data[ii].data, buffer_array_data[ii].len);
      cur_pos += buffer_array_data[ii].len;
    }

    // 发送数据
    ros2_publisher_ptr->publish(wrapper_msg);
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

rclcpp::QoS Ros2ChannelBackend::GetQos(const Options::QosOptions& qos_option) {
  rclcpp::QoS qos(qos_option.depth);

  if (qos_option.history == "keep_all") {
    qos.keep_all();
  } else {
    qos.keep_last(qos_option.depth);
  }

  if (qos_option.reliability == "reliable") {
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  } else {
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  }

  if (qos_option.durability == "transient_local") {
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  } else {
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
  }

  if (qos_option.liveliness == "automatic") {
    qos.liveliness(rclcpp::LivelinessPolicy::Automatic);
  } else if (qos_option.liveliness == "manual_by_topic") {
    qos.liveliness(rclcpp::LivelinessPolicy::ManualByTopic);
  } else {
    qos.liveliness(rclcpp::LivelinessPolicy::SystemDefault);
  }

  if (qos_option.deadline != -1) {
    qos.deadline(rclcpp::Duration::from_nanoseconds(qos_option.deadline * 1000000));
  }

  if (qos_option.lifespan != -1) {
    qos.lifespan(rclcpp::Duration::from_nanoseconds(qos_option.lifespan * 1000000));
  }

  if (qos_option.liveliness_lease_duration != -1) {
    qos.liveliness_lease_duration(rclcpp::Duration::from_nanoseconds(qos_option.liveliness_lease_duration * 1000000));
  }
  return qos;
}

}  // namespace aimrt::plugins::ros2_plugin