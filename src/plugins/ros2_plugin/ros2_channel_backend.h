// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "core/channel/channel_backend_base.h"
#include "core/channel/channel_backend_tools.h"

#include "ros2_plugin/ros2_adapter_subscription.h"

#include "rclcpp/rclcpp.hpp"

#include "rcl/publisher.h"

#include "ros2_plugin_proto/msg/ros_msg_wrapper.hpp"

namespace aimrt::plugins::ros2_plugin {

class Ros2ChannelBackend : public runtime::core::channel::ChannelBackendBase {
 public:
  struct Options {
    struct QosOptions {
      /**
       * @brief history options
       * @param keep_last: Keep the most recent records (cache up to N records, which can be configured through the queue length option)
       * @param keep_all: Keep all records (caches all records, but is limited to the maximum resource that can be configured by the underlying middleware)
       * @param default: System default
       */
      std::string history = "default";

      /**
       * @brief queue depth option (only used with Keep_last)
       */
      int32_t depth = 10;

      /**
       * @brief Reliability Options
       * @param reliable: Reliable (when the message is lost, it will be resent and retransmitted repeatedly to ensure the successful data transmission)
       * @param best_effort: Try to transfer data but do not guarantee successful transmission, data may be lost when the network is unstable
       * @param default: System default
       */
      std::string reliability = "default";

      /**
       * @brief persistence options
       * @param transient_local: the publisher retains data for late-joining subscribers
       * @param volatile: no data is retained
       * @param default: System default
       */
      std::string durability = "default";

      /**
       * @brief The maximum amount of time expected between subsequent messages posted to the topic
       * @param ms Millisecond timestamp. -1 indicates that this parameter is disabled
       */
      int64_t deadline = -1;

      /**
       * @brief The maximum amount of time between message being published and received without treating the message as stale or expired (expired messages are silently discarded and never actually received).
       * @param ms Millisecond timestamp. -1 indicates that this parameter is disabled
       */
      int64_t lifespan = -1;

      /**
       * @param automatic: Automatic (ROS2 will judge based on the time interval between message publishing and receiving)
       * @param manual_by_topic: Publisher needs to declare regularly
       * @param default: System default
       */
      std::string liveliness = "default";

      /**
       * @brief The duration of the active lease period, if the publisher does not declare active beyond this time, it is considered inactive.
       * @param ms Millisecond timestamp. -1 indicates that this parameter is disabled
       */
      int64_t liveliness_lease_duration = -1;
    };

    struct PubTopicOptions {
      std::string topic_name;
      QosOptions qos;
    };

    std::vector<PubTopicOptions> pub_topics_options;

    struct SubTopicOptions {
      std::string topic_name;
      QosOptions qos;
    };

    std::vector<SubTopicOptions> sub_topics_options;
  };

 public:
  Ros2ChannelBackend() = default;
  ~Ros2ChannelBackend() override = default;

  std::string_view Name() const noexcept override { return "ros2"; }

  void Initialize(YAML::Node options_node) override;
  void Start() override;
  void Shutdown() override;

  void SetChannelRegistry(const runtime::core::channel::ChannelRegistry* channel_registry_ptr) noexcept override {
    channel_registry_ptr_ = channel_registry_ptr;
  }

  bool RegisterPublishType(
      const runtime::core::channel::PublishTypeWrapper& publish_type_wrapper) noexcept override;
  bool Subscribe(const runtime::core::channel::SubscribeWrapper& subscribe_wrapper) noexcept override;
  void Publish(runtime::core::channel::MsgWrapper& msg_wrapper) noexcept override;

  void SetNodePtr(const std::shared_ptr<rclcpp::Node>& ros2_node_ptr) {
    ros2_node_ptr_ = ros2_node_ptr;
  }

 private:
  static bool CheckRosMsg(std::string_view msg_type) {
    return (msg_type.substr(0, 5) == "ros2:");
  }
  rclcpp::QoS GetQos(const Options::QosOptions& qos_option);

 private:
  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

  Options options_;
  std::atomic<State> state_ = State::kPreInit;

  const runtime::core::channel::ChannelRegistry* channel_registry_ptr_ = nullptr;

  std::shared_ptr<rclcpp::Node> ros2_node_ptr_;

  struct Key {
    std::string topic_name;
    std::string msg_type;

    bool operator==(const Key& rhs) const {
      return topic_name == rhs.topic_name && msg_type == rhs.msg_type;
    }

    struct Hash {
      std::size_t operator()(const Key& k) const {
        return (std::hash<std::string>()(k.topic_name)) ^
               (std::hash<std::string>()(k.msg_type));
      }
    };
  };

  // ros2 msg
  std::unordered_map<
      Key,
      std::unique_ptr<rcl_publisher_t>,
      Key::Hash,
      std::equal_to<>>
      ros2_publish_type_wrapper_map_;

  struct RosSubWrapper {
    std::unique_ptr<aimrt::runtime::core::channel::SubscribeTool> sub_tool_ptr;
    std::shared_ptr<rclcpp::SubscriptionBase> ros_sub_handle_ptr;
  };

  std::unordered_map<
      Key,
      RosSubWrapper,
      Key::Hash,
      std::equal_to<>>
      ros2_subscribe_wrapper_map_;

  // other msg
  std::unordered_map<
      std::string,
      rclcpp::Publisher<ros2_plugin_proto::msg::RosMsgWrapper>::SharedPtr>
      publisher_map_;

  struct SubWrapper {
    std::unique_ptr<aimrt::runtime::core::channel::SubscribeTool> sub_tool_ptr;
    rclcpp::Subscription<ros2_plugin_proto::msg::RosMsgWrapper>::SharedPtr ros_sub_handle_ptr;
  };

  std::unordered_map<std::string, SubWrapper> subscribe_wrapper_map_;
};

}  // namespace aimrt::plugins::ros2_plugin