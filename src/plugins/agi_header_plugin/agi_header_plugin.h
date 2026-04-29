// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>
#include <unordered_map>

#include "agi_header_util.h"
#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"
#include "core/channel/channel_framework_async_filter.h"
#include "core/rpc/rpc_framework_async_filter.h"

namespace aimrt::plugins::agi_header_plugin {

class AgiHeaderPlugin : public AimRTCorePluginBase {
 public:
  AgiHeaderPlugin() = default;
  ~AgiHeaderPlugin() override = default;

  std::string_view Name() const noexcept override { return "agi_header_plugin"; }

  bool Initialize(runtime::core::AimRTCore* core_ptr) noexcept override;
  void Shutdown() noexcept override;

 private:
  void RegisterChannelFilter();
  void RegisterRpcFilter();

  void PublishHeaderFilter(
      aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
      aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h);

  void RpcClientHeaderFilter(
      const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
      aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h);
  void ScanRegisteredPublishTypes();
  void ScanRegisteredRpcClientFuncs();

 private:
  runtime::core::AimRTCore* core_ptr_ = nullptr;
  bool init_flag_ = false;

  struct PublishInfoKey {
    std::string msg_type;
    std::string topic_name;
    std::string pkg_path;
    std::string module_name;

    bool operator==(const PublishInfoKey& rhs) const {
      return msg_type == rhs.msg_type &&
             topic_name == rhs.topic_name &&
             pkg_path == rhs.pkg_path &&
             module_name == rhs.module_name;
    }

    struct Hash {
      std::size_t operator()(const PublishInfoKey& key) const {
        return std::hash<std::string>{}(key.msg_type) ^
               std::hash<std::string>{}(key.topic_name) ^
               std::hash<std::string>{}(key.pkg_path) ^
               std::hash<std::string>{}(key.module_name);
      }
    };
  };

  struct RpcClientInfoKey {
    std::string func_name;
    std::string pkg_path;
    std::string module_name;

    bool operator==(const RpcClientInfoKey& rhs) const {
      return func_name == rhs.func_name &&
             pkg_path == rhs.pkg_path &&
             module_name == rhs.module_name;
    }

    struct Hash {
      std::size_t operator()(const RpcClientInfoKey& key) const {
        return std::hash<std::string>{}(key.func_name) ^
               std::hash<std::string>{}(key.pkg_path) ^
               std::hash<std::string>{}(key.module_name);
      }
    };
  };

  using HeaderInfoMapByPublishKey = std::unordered_map<
      PublishInfoKey,
      runtime::core::util::AgiHeaderInfo,
      PublishInfoKey::Hash>;
  using HeaderInfoMapByRpcClientKey = std::unordered_map<
      RpcClientInfoKey,
      runtime::core::util::AgiHeaderInfo,
      RpcClientInfoKey::Hash>;

  HeaderInfoMapByPublishKey publish_header_info_map_;
  HeaderInfoMapByRpcClientKey rpc_request_header_info_map_;
};

}  // namespace aimrt::plugins::agi_header_plugin
