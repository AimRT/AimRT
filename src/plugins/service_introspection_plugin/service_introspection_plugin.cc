// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "service_introspection_plugin/service_introspection_plugin.h"
#include "aimrt_module_c_interface/rpc/rpc_context_base.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "core/aimrt_core.h"
#include "core/rpc/rpc_backend_tools.h"
#include "service_introspection.pb.h"
#include "service_introspection_plugin/global.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::service_introspection_plugin::ServiceIntrospectionPlugin::Options> {
  using Options = aimrt::plugins::service_introspection_plugin::ServiceIntrospectionPlugin::Options;

  static Node encode(const Options &rhs) {
    Node node;

    node["client_info_topic_name"] = rhs.client_info_topic_name;
    node["server_info_topic_name"] = rhs.server_info_topic_name;

    if (rhs.mode == Options::Mode::kMeta) {
      node["mode"] = "meta";
    } else if (rhs.mode == Options::Mode::khybrid) {
      node["mode"] = "hybrid";
    } else if (rhs.mode == Options::Mode::kFull) {
      node["mode"] = "full";
    }

    if (rhs.rpc_serialization_type == Options::RpcSerializationType::kJson) {
      node["rpc_serialization_type"] = "json";
    } else if (rhs.rpc_serialization_type == Options::RpcSerializationType::kAuto) {
      node["rpc_serialization_type"] = "auto";
    }

    return node;
  }

  static bool decode(const Node &node, Options &rhs) {
    if (!node.IsMap()) return false;

    auto mode = aimrt::common::util::StrToLower(node["mode"].as<std::string>());
    if (mode == "meta") {
      rhs.mode = Options::Mode::kMeta;
    } else if (mode == "hybrid") {
      rhs.mode = Options::Mode::khybrid;
    } else if (mode == "full") {
      rhs.mode = Options::Mode::kFull;
    } else {
      throw aimrt::common::util::AimRTException("Invalid mode: " + mode);
    }

    auto rpc_serialization_type = aimrt::common::util::StrToLower(node["rpc_serialization_type"].as<std::string>());
    if (rpc_serialization_type == "json") {
      rhs.rpc_serialization_type = Options::RpcSerializationType::kJson;
    } else if (rpc_serialization_type == "auto") {
      rhs.rpc_serialization_type = Options::RpcSerializationType::kAuto;
    } else {
      throw aimrt::common::util::AimRTException("Invalid rpc serialization type: " + rpc_serialization_type);
    }

    rhs.client_info_topic_name = node["client_info_topic_name"].as<std::string>();
    rhs.server_info_topic_name = node["server_info_topic_name"].as<std::string>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::service_introspection_plugin {

bool ServiceIntrospectionPlugin::Initialize(runtime::core::AimRTCore *core_ptr) noexcept {
  try {
    core_ptr_ = core_ptr;

    YAML::Node plugin_options_node = core_ptr_->GetPluginManager().GetPluginOptionsNode(Name());

    if (plugin_options_node && !plugin_options_node.IsNull()) {
      options_ = plugin_options_node.as<Options>();
    }

    init_flag_ = true;

    // Register filters
    RegisterClientFilter();
    RegisterServiceFilter();

    // Register publisher
    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostInitChannel,
                                [this] { RegisterPublisher(); });

    // Set logger
    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostInitLog,
                                [this] { SetPluginLogger(); });

    plugin_options_node = options_;
    core_ptr_->GetPluginManager().UpdatePluginOptionsNode(Name(), plugin_options_node);

    return true;
  } catch (const std::exception &e) {
    AIMRT_ERROR("Initialize failed, {}", e.what());
  }

  return false;
}

void ServiceIntrospectionPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;

    stop_flag_ = true;

  } catch (const std::exception &e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
  }
}

void ServiceIntrospectionPlugin::SetPluginLogger() {
  SetLogger(aimrt::logger::LoggerRef(
      core_ptr_->GetLoggerManager().GetLoggerProxy().NativeHandle()));
}

void ServiceIntrospectionPlugin::RegisterPublisher() {
  auto setup_publisher = [this](const std::string &topic_name)
      -> aimrt::channel::PublisherRef {
    auto publisher = aimrt::channel::ChannelHandleRef(
                         core_ptr_->GetChannelManager().GetChannelHandleProxy().NativeHandle())
                         .GetPublisher(topic_name);

    AIMRT_ASSERT(publisher, "Failed to get publisher for topic: {}", topic_name);

    bool ret = aimrt::channel::RegisterPublishType<
        aimrt::protocols::service_introspection::RpcMsgList>(publisher);
    AIMRT_ASSERT(ret, "Register publish type failed for topic: {}", topic_name);

    return publisher;
  };

  // set client_info publisher
  publisher_ = setup_publisher(options_.client_info_topic_name);
  client_info_publisher_ = std::make_unique<aimrt::channel::PublisherRef>(publisher_);

  // set server_info publisher
  if (options_.client_info_topic_name == options_.server_info_topic_name) {
    server_info_publisher_ = std::make_unique<aimrt::channel::PublisherRef>(publisher_);
  } else {
    publisher2_ = setup_publisher(options_.server_info_topic_name);
    server_info_publisher_ = std::make_unique<aimrt::channel::PublisherRef>(publisher2_);
  }
}

void ServiceIntrospectionPlugin::RegisterClientFilter() {
  core_ptr_->GetRpcManager().RegisterClientFilter("service_introspection", [this](const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper> &wrapper_ptr,
                                                                                  aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle &&h) {
    wrapper_ptr->callback =
        [this, wrapper_ptr, callback{std::move(wrapper_ptr->callback)}](aimrt::rpc::Status status) {
          // fill protocol
          aimrt::protocols::service_introspection::RpcMsgList rpc_msg_list;
          auto *rpc_msg = rpc_msg_list.add_rpc_msg();

          // fill metadata
          rpc_msg->set_event_type(aimrt::protocols::service_introspection::CLIENT);
          rpc_msg->set_time_stamp(aimrt::common::util::GetCurTimestampNs());
          rpc_msg->set_service_name(wrapper_ptr->info.func_name);
          rpc_msg->set_module_name(wrapper_ptr->info.module_name);
          rpc_msg->set_backend_type(std::string(wrapper_ptr->ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_BACKEND)));
          rpc_msg->set_sequence_number(std::stoull(std::string(wrapper_ptr->ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_REQUEST_ID))));
          rpc_msg->set_status_code(status.Code());

          std::string_view rpc_serialization_type;
          if (options_.rpc_serialization_type == Options::RpcSerializationType::kJson) {
            rpc_serialization_type = "json";
          } else {
            rpc_serialization_type = wrapper_ptr->ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE);
          }

          if (options_.mode != Options::Mode::kMeta) {  // fill request and response
            auto req_str = TrySerializeReqWithCache(*wrapper_ptr, rpc_serialization_type)->JoinToCharVector();
            auto rsp_str = TrySerializeRspWithCache(*wrapper_ptr, rpc_serialization_type)->JoinToCharVector();
            rpc_msg->set_request(req_str.data(), req_str.size());
            rpc_msg->set_response(rsp_str.data(), rsp_str.size());
          }

          // publish
          aimrt::channel::Publish(*client_info_publisher_, rpc_msg_list);

          // call user's callback
          callback(status);
        };

    h(wrapper_ptr);
  });
}
void ServiceIntrospectionPlugin::RegisterServiceFilter() {
  core_ptr_->GetRpcManager().RegisterServerFilter("service_introspection", [this](const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper> &wrapper_ptr,
                                                                                  aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle &&h) {
    wrapper_ptr->callback =
        [this, wrapper_ptr, callback{std::move(wrapper_ptr->callback)}](aimrt::rpc::Status status) {
          aimrt::protocols::service_introspection::RpcMsgList rpc_msg_list;
          auto *rpc_msg = rpc_msg_list.add_rpc_msg();

          // fill metadata
          rpc_msg->set_event_type(aimrt::protocols::service_introspection::SERVER);
          rpc_msg->set_time_stamp(aimrt::common::util::GetCurTimestampNs());
          rpc_msg->set_service_name(wrapper_ptr->info.func_name);
          rpc_msg->set_module_name(wrapper_ptr->info.module_name);
          rpc_msg->set_backend_type(std::string(wrapper_ptr->ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_BACKEND)));
          rpc_msg->set_sequence_number(std::stoull(std::string(wrapper_ptr->ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_RESPONSE_ID))));
          rpc_msg->set_status_code(status.Code());

          std::string_view rpc_serialization_type;
          if (options_.rpc_serialization_type == Options::RpcSerializationType::kJson) {
            rpc_serialization_type = "json";
          } else {
            rpc_serialization_type = wrapper_ptr->ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE);
          }

          if (options_.mode == Options::Mode::kFull) {  // fill request and response
            auto req_str = TrySerializeReqWithCache(*wrapper_ptr, rpc_serialization_type)->JoinToCharVector();
            auto rsp_str = TrySerializeRspWithCache(*wrapper_ptr, rpc_serialization_type)->JoinToCharVector();
            rpc_msg->set_request(req_str.data(), req_str.size());
            rpc_msg->set_response(rsp_str.data(), rsp_str.size());
          }

          // publish
          aimrt::channel::Publish(*server_info_publisher_, rpc_msg_list);

          // call user's callback
          callback(status);
        };

    h(wrapper_ptr);
  });
}

}  // namespace aimrt::plugins::service_introspection_plugin
