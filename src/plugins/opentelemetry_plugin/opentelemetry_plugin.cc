// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "opentelemetry_plugin/opentelemetry_plugin.h"

#include "core/aimrt_core.h"
#include "core/channel/channel_backend_tools.h"
#include "core/rpc/rpc_backend_tools.h"
#include "opentelemetry_plugin/context_carrier.h"
#include "opentelemetry_plugin/global.h"
#include "opentelemetry_plugin/util.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::opentelemetry_plugin::OpenTelemetryPlugin::Options> {
  using Options = aimrt::plugins::opentelemetry_plugin::OpenTelemetryPlugin::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["node_name"] = rhs.node_name;
    node["trace_otlp_http_exporter_url"] = rhs.trace_otlp_http_exporter_url;
    node["force_trace"] = rhs.force_trace;

    node["attributes"] = YAML::Node();
    for (const auto& attribute : rhs.attributes) {
      Node attribute_node;
      attribute_node["key"] = attribute.key;
      attribute_node["val"] = attribute.val;
      node["attributes"].push_back(attribute_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    rhs.node_name = node["node_name"].as<std::string>();
    rhs.trace_otlp_http_exporter_url = node["trace_otlp_http_exporter_url"].as<std::string>();

    if (node["force_trace"])
      rhs.force_trace = node["force_trace"].as<bool>();

    for (const auto& attribute_node : node["attributes"]) {
      auto attribute = Options::Attribute{
          .key = attribute_node["key"].as<std::string>(),
          .val = attribute_node["val"].as<std::string>(),
      };
      rhs.attributes.emplace_back(std::move(attribute));
    }

    return true;
  }
};
}  // namespace YAML

namespace trace_api = opentelemetry::trace;
namespace trace_sdk = opentelemetry::sdk::trace;
namespace resource = opentelemetry::sdk::resource;
namespace otlp = opentelemetry::exporter::otlp;

namespace aimrt::plugins::opentelemetry_plugin {

bool OpenTelemetryPlugin::Initialize(runtime::core::AimRTCore* core_ptr) noexcept {
  try {
    core_ptr_ = core_ptr;

    YAML::Node plugin_options_node = core_ptr_->GetPluginManager().GetPluginOptionsNode(Name());

    if (plugin_options_node && !plugin_options_node.IsNull()) {
      options_ = plugin_options_node.as<Options>();
    }

    init_flag_ = true;

    AIMRT_CHECK_ERROR_THROW(!options_.node_name.empty(), "node name is empty!");

    // init opentelemetry
    auto resource_attributes = resource::ResourceAttributes{
        {"service.name", options_.node_name}};
    for (auto& itr : options_.attributes) {
      resource_attributes.SetAttribute(itr.key, itr.val);
    }
    auto resource = resource::Resource::Create(resource_attributes);

    otlp::OtlpHttpExporterOptions opts;
    opts.url = options_.trace_otlp_http_exporter_url;
    auto exporter = otlp::OtlpHttpExporterFactory::Create(opts);

    trace_sdk::BatchSpanProcessorOptions bsp_opts{};
    auto processor = trace_sdk::BatchSpanProcessorFactory::Create(std::move(exporter), bsp_opts);

    provider_ = trace_sdk::TracerProviderFactory::Create(std::move(processor), resource);

    propagator_ = std::make_shared<trace_api::propagation::HttpTraceContext>();

    // register hook
    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostInitLog,
                                [this] { SetPluginLogger(); });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitChannel,
                                [this] { RegisterChannelFilter(); });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitRpc,
                                [this] { RegisterRpcFilter(); });

    plugin_options_node = options_;
    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("Initialize failed, {}", e.what());
  }

  return false;
}

void OpenTelemetryPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;

    provider_.reset();
    propagator_.reset();

  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
  }
}

void OpenTelemetryPlugin::SetPluginLogger() {
  SetLogger(aimrt::logger::LoggerRef(
      core_ptr_->GetLoggerManager().GetLoggerProxy().NativeHandle()));
}

void OpenTelemetryPlugin::RegisterChannelFilter() {
  auto& channel_manager = core_ptr_->GetChannelManager();

  channel_manager.SetPassedContextMetaKeys(
      {std::string(kCtxKeyStartNewTrace),
       std::string(kCtxKeyTraceParent),
       std::string(kCtxKeyTraceState)});

  channel_manager.RegisterPublishFilter(
      "otp_trace",
      [this](aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
             aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h) {
        ChannelFilter(trace_api::SpanKind::kProducer, true, msg_wrapper, std::move(h));
      });

  channel_manager.RegisterPublishFilter(
      "otp_simple_trace",
      [this](aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
             aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h) {
        ChannelFilter(trace_api::SpanKind::kProducer, false, msg_wrapper, std::move(h));
      });

  channel_manager.RegisterSubscribeFilter(
      "otp_trace",
      [this](aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
             aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h) {
        ChannelFilter(trace_api::SpanKind::kConsumer, true, msg_wrapper, std::move(h));
      });

  channel_manager.RegisterSubscribeFilter(
      "otp_simple_trace",
      [this](aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
             aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h) {
        ChannelFilter(trace_api::SpanKind::kConsumer, false, msg_wrapper, std::move(h));
      });
}

void OpenTelemetryPlugin::RegisterRpcFilter() {
  auto& rpc_manager = core_ptr_->GetRpcManager();

  rpc_manager.SetPassedContextMetaKeys(
      {std::string(kCtxKeyStartNewTrace),
       std::string(kCtxKeyTraceParent),
       std::string(kCtxKeyTraceState)});

  rpc_manager.RegisterClientFilter(
      "otp_trace",
      [this](const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
             aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h) {
        RpcFilter(trace_api::SpanKind::kClient, true, wrapper_ptr, std::move(h));
      });

  rpc_manager.RegisterClientFilter(
      "otp_simple_trace",
      [this](const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
             aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h) {
        RpcFilter(trace_api::SpanKind::kClient, false, wrapper_ptr, std::move(h));
      });

  rpc_manager.RegisterServerFilter(
      "otp_trace",
      [this](const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
             aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h) {
        RpcFilter(trace_api::SpanKind::kServer, true, wrapper_ptr, std::move(h));
      });

  rpc_manager.RegisterServerFilter(
      "otp_simple_trace",
      [this](const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
             aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h) {
        RpcFilter(trace_api::SpanKind::kServer, false, wrapper_ptr, std::move(h));
      });
}

void OpenTelemetryPlugin::ChannelFilter(
    opentelemetry::trace::SpanKind kind,
    bool upload_msg,
    aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
    aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h) {
  auto ctx_ref = msg_wrapper.ctx_ref;

  // 如果设置了全局强制trace，或者context强制设置了start_new_trace，或者上层传递了span，则新启动一个span
  bool start_new_trace = options_.force_trace;

  if (!start_new_trace) {
    start_new_trace = common::util::CheckIEqual(ctx_ref.GetMetaValue(kCtxKeyStartNewTrace), "true");
  } else {
    ctx_ref.SetMetaValue(kCtxKeyStartNewTrace, "true");
  }

  auto tracer = provider_->GetTracer(options_.node_name);
  ContextCarrier carrier(ctx_ref);

  // 解压传进来的context，得到父span
  trace_api::StartSpanOptions op{
      .kind = kind,
  };

  opentelemetry::context::Context input_ot_ctx;
  auto extract_ctx = propagator_->Extract(carrier, input_ot_ctx);

  auto extract_ctx_val = extract_ctx.GetValue(trace_api::kSpanKey);
  if (!::opentelemetry::nostd::holds_alternative<::opentelemetry::nostd::monostate>(extract_ctx_val)) {
    auto parent_span =
        ::opentelemetry::nostd::get<::opentelemetry::nostd::shared_ptr<::opentelemetry::trace::Span>>(extract_ctx_val);
    op.parent = parent_span->GetContext();
    start_new_trace = true;
  }

  // 不需要启动一个新trace
  if (!start_new_trace) {
    h(msg_wrapper);
    return;
  }

  // 需要启动一个新trace
  std::string span_name = msg_wrapper.info.topic_name + "/" + msg_wrapper.info.msg_type;
  auto span = tracer->StartSpan(ToNoStdStringView(span_name), op);

  // 将当前span的context打包
  opentelemetry::context::Context output_ot_ctx(trace_api::kSpanKey, span);
  propagator_->Inject(carrier, output_ot_ctx);

  // 添加context中的属性
  auto keys = ctx_ref.GetMetaKeys();
  for (auto& itr : keys) {
    span->SetAttribute(ToNoStdStringView(itr), ToNoStdStringView(ctx_ref.GetMetaValue(itr)));
  }

  h(msg_wrapper);

  if (upload_msg) {
    // 序列化包成json
    auto buf_ptr = aimrt::runtime::core::channel::TrySerializeMsgWithCache(msg_wrapper, "json");
    if (buf_ptr) {
      auto msg_str = buf_ptr->JoinToString();
      if (!msg_str.empty()) span->SetAttribute("msg_data", msg_str);
    }
  }

  span->End();
}

void OpenTelemetryPlugin::RpcFilter(
    opentelemetry::trace::SpanKind kind,
    bool upload_msg,
    const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
    aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h) {
  auto ctx_ref = wrapper_ptr->ctx_ref;

  // 如果设置了全局强制trace，或者context强制设置了start_new_trace，或者上层传递了span，则新启动一个span
  bool start_new_trace = options_.force_trace;

  if (!start_new_trace) {
    start_new_trace = common::util::CheckIEqual(ctx_ref.GetMetaValue(kCtxKeyStartNewTrace), "true");
  } else {
    ctx_ref.SetMetaValue(kCtxKeyStartNewTrace, "true");
  }

  auto tracer = provider_->GetTracer(options_.node_name);
  ContextCarrier carrier(ctx_ref);

  // 解压传进来的context，得到父span
  trace_api::StartSpanOptions op{
      .kind = kind,
  };

  opentelemetry::context::Context input_ot_ctx;
  auto extract_ctx = propagator_->Extract(carrier, input_ot_ctx);

  auto extract_ctx_val = extract_ctx.GetValue(trace_api::kSpanKey);
  if (!::opentelemetry::nostd::holds_alternative<::opentelemetry::nostd::monostate>(extract_ctx_val)) {
    auto parent_span =
        ::opentelemetry::nostd::get<::opentelemetry::nostd::shared_ptr<::opentelemetry::trace::Span>>(extract_ctx_val);
    op.parent = parent_span->GetContext();
    start_new_trace = true;
  }

  // 不需要启动一个新trace
  if (!start_new_trace) {
    h(wrapper_ptr);
    return;
  }

  // 需要启动一个新trace
  auto span = tracer->StartSpan(ToNoStdStringView(ctx_ref.GetFunctionName()), op);

  // 将当前span的context打包
  opentelemetry::context::Context output_ot_ctx(trace_api::kSpanKey, span);
  propagator_->Inject(carrier, output_ot_ctx);

  // 添加context中的属性
  auto keys = ctx_ref.GetMetaKeys();
  for (auto& itr : keys) {
    span->SetAttribute(ToNoStdStringView(itr), ToNoStdStringView(ctx_ref.GetMetaValue(itr)));
  }

  wrapper_ptr->callback =
      [upload_msg,
       wrapper_ptr,
       span{std::move(span)},
       callback{std::move(wrapper_ptr->callback)}](aimrt::rpc::Status status) {
        if (status.OK()) {
          span->SetStatus(trace_api::StatusCode::kOk);
        } else {
          span->SetStatus(trace_api::StatusCode::kError, status.ToString());
        }

        if (upload_msg) {
          // 序列化req/rsp为json
          auto req_buf_ptr = aimrt::runtime::core::rpc::TrySerializeReqWithCache(*wrapper_ptr, "json");
          if (req_buf_ptr) {
            auto req_json = req_buf_ptr->JoinToString();
            if (!req_json.empty()) span->SetAttribute("req_data", req_json);
          }

          auto rsp_buf_ptr = aimrt::runtime::core::rpc::TrySerializeRspWithCache(*wrapper_ptr, "json");
          if (rsp_buf_ptr) {
            auto rsp_json = rsp_buf_ptr->JoinToString();
            if (!rsp_json.empty()) span->SetAttribute("rsp_data", rsp_json);
          }
        }

        span->End();

        callback(status);
      };

  h(wrapper_ptr);
}

}  // namespace aimrt::plugins::opentelemetry_plugin
