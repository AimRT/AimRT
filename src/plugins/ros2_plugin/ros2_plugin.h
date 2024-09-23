// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <thread>

#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"

#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aimrt::plugins::ros2_plugin {

class Ros2Plugin : public AimRTCorePluginBase {
 public:
  struct Options {
    std::string node_name;
    // SingleThreaded | StaticSingleThreaded | MultiThreaded
    std::string executor_type = "MultiThreaded";
    uint32_t executor_thread_num = 2;
  };

 public:
  Ros2Plugin() = default;
  ~Ros2Plugin() override = default;

  std::string_view Name() const noexcept override { return "ros2_plugin"; }

  bool Initialize(runtime::core::AimRTCore* core_ptr) noexcept override;
  void Shutdown() noexcept override;

 private:
  void SetPluginLogger();
  void RegisterRos2RpcBackend();
  void RegisterRos2ChannelBackend();

 private:
  runtime::core::AimRTCore* core_ptr_ = nullptr;

  Options options_;

  bool init_flag_ = false;

  std::shared_ptr<rclcpp::Node> ros2_node_ptr_;
  std::shared_ptr<rclcpp::Executor> ros2_node_executor_ptr_;
  std::unique_ptr<std::thread> ros2_thread_ptr_;
};

}  // namespace aimrt::plugins::ros2_plugin
