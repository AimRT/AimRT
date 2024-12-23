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
       * @brief 历史记录选项
       * @param keep_last:保留最近的记录(缓存最多N条记录，可通过队列长度选项来配置)
       * @param keep_all:保留所有记录(缓存所有记录，但受限于底层中间件可配置的最大资源)
       * @param default:系统默认
       */
      std::string history = "keep_last";

      /**
       * @brief 队列深度选项(只能与Keep_last配合使用)
       */
      int32_t depth = 10;

      /**
       * @brief 可靠性选项
       * @param reliable:可靠的(消息丢失时，会重新发送,反复重传以保证数据传输成功)
       * @param best_effort:尽力而为的(尝试传输数据但不保证成功传输,当网络不稳定时可能丢失数据)
       * @param default:系统默认
       */
      std::string reliability = "best_effort";

      /**
       * @brief 持续性选项
       * @param transient_local:局部瞬态(发布器为晚连接(late-joining)的订阅器保留数据)
       * @param volatile:易变态(不保留任何数据)
       * @param default:系统默认
       */
      std::string durability = "volatile";

      /**
       * @brief 后续消息发布到主题之间的预期最大时间量
       * @param ms级时间戳 -1为不设置
       */
      int64_t deadline = -1;

      /**
       * @brief 消息发布和接收之间的最大时间量，而不将消息视为陈旧或过期（过期的消息被静默地丢弃，并且实际上从未被接收）。
       * @param ms级时间戳 -1为不设置
       */
      int64_t lifespan = -1;

      /**
       * @brief 如何确定发布者是否活跃
       * @param automatic:自动(ROS2会根据消息发布和接收的时间间隔来判断)
       * @param manual_by_topic:需要发布者定期声明
       * @param default:系统默认
       */
      std::string liveliness = "default";

      /**
       * @brief 活跃性租期的时长，如果超过这个时间发布者没有声明活跃，则被认为是不活跃的。
       * @param ms级时间戳 -1为不设置
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