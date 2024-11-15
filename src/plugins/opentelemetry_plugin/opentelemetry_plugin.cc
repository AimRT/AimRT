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

    node["metrics_otlp_http_exporter_url"] = rhs.metrics_otlp_http_exporter_url;
    node["metrics_export_interval_ms"] = rhs.metrics_export_interval_ms;
    node["metrics_export_timeout_ms"] = rhs.metrics_export_timeout_ms;
    node["rpc_time_cost_histogram_boundaries"] = rhs.rpc_time_cost_histogram_boundaries;

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

    if (node["trace_otlp_http_exporter_url"])
      rhs.trace_otlp_http_exporter_url = node["trace_otlp_http_exporter_url"].as<std::string>();

    if (node["force_trace"])
      rhs.force_trace = node["force_trace"].as<bool>();

    if (node["metrics_otlp_http_exporter_url"])
      rhs.metrics_otlp_http_exporter_url = node["metrics_otlp_http_exporter_url"].as<std::string>();

    if (node["metrics_export_interval_ms"])
      rhs.metrics_export_interval_ms = node["metrics_export_interval_ms"].as<uint32_t>();

    if (node["metrics_export_timeout_ms"])
      rhs.metrics_export_timeout_ms = node["metrics_export_timeout_ms"].as<uint32_t>();

    if (node["rpc_time_cost_histogram_boundaries"])
      rhs.rpc_time_cost_histogram_boundaries = node["rpc_time_cost_histogram_boundaries"].as<std::vector<double>>();

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
namespace metrics_api = opentelemetry::metrics;
namespace metric_sdk = opentelemetry::sdk::metrics;
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

    propagator_ = std::make_shared<trace_api::propagation::HttpTraceContext>();

    // trace
    if (!options_.trace_otlp_http_exporter_url.empty()) {
      enable_trace_ = true;

      otlp::OtlpHttpExporterOptions opts;
      opts.url = options_.trace_otlp_http_exporter_url;
      auto exporter = otlp::OtlpHttpExporterFactory::Create(opts);

      trace_sdk::BatchSpanProcessorOptions bsp_opts{};
      auto processor = trace_sdk::BatchSpanProcessorFactory::Create(std::move(exporter), bsp_opts);

      trace_provider_ = trace_sdk::TracerProviderFactory::Create(std::move(processor), resource);

      tracer_ = trace_provider_->GetTracer(options_.node_name);
    }

    // metrics
    if (!options_.metrics_otlp_http_exporter_url.empty()) {
      enable_metrics_ = true;

      otlp::OtlpHttpMetricExporterOptions opts;
      opts.url = options_.metrics_otlp_http_exporter_url;
      auto exporter = otlp::OtlpHttpMetricExporterFactory::Create(opts);

      metric_sdk::PeriodicExportingMetricReaderOptions reader_opts{};
      reader_opts.export_interval_millis = std::chrono::milliseconds(options_.metrics_export_interval_ms);
      reader_opts.export_timeout_millis = std::chrono::milliseconds(options_.metrics_export_timeout_ms);

      auto reader = metric_sdk::PeriodicExportingMetricReaderFactory::Create(std::move(exporter), reader_opts);

      std::vector<double> bucket_boundaries;
      if (!options_.rpc_time_cost_histogram_boundaries.empty()) {
        bucket_boundaries = options_.rpc_time_cost_histogram_boundaries;
      } else {
        bucket_boundaries.resize(32);
        double val = 1.0;
        std::generate(bucket_boundaries.begin(), bucket_boundaries.end(),
                      [&val]() {
                        double current = val;
                        val *= 2.0;
                        return current;
                      });
      }

      auto views = metric_sdk::ViewRegistryFactory::Create();
      // configure RPC client time cost histogram
      std::unique_ptr<metric_sdk::InstrumentSelector> client_instrument_selector = std::make_unique<metric_sdk::InstrumentSelector>(metric_sdk::InstrumentType::kHistogram, "rpc.client.time_cost", "us");
      std::unique_ptr<metric_sdk::MeterSelector> client_meter_selector = std::make_unique<metric_sdk::MeterSelector>(options_.node_name, "", "");

      std::shared_ptr<metric_sdk::HistogramAggregationConfig> client_config = std::make_shared<metric_sdk::HistogramAggregationConfig>();
      client_config->boundaries_ = bucket_boundaries;

      std::unique_ptr<metric_sdk::View> client_view = std::make_unique<metric_sdk::View>(
          "rpc_client_time_cost",
          "RPC client time cost histogram view",
          "us",
          metric_sdk::AggregationType::kHistogram,
          client_config);

      views->AddView(std::move(client_instrument_selector),
                     std::move(client_meter_selector),
                     std::move(client_view));

      // configure RPC server time cost histogram
      std::unique_ptr<metric_sdk::InstrumentSelector> server_instrument_selector = std::make_unique<metric_sdk::InstrumentSelector>(metric_sdk::InstrumentType::kHistogram, "rpc.server.time_cost", "us");
      std::unique_ptr<metric_sdk::MeterSelector> server_meter_selector = std::make_unique<metric_sdk::MeterSelector>(options_.node_name, "", "");

      std::shared_ptr<metric_sdk::HistogramAggregationConfig> server_config = std::make_shared<metric_sdk::HistogramAggregationConfig>();
      server_config->boundaries_ = bucket_boundaries;

      std::unique_ptr<metric_sdk::View> server_view = std::make_unique<metric_sdk::View>(
          "rpc_server_time_cost",
          "RPC server time cost histogram view",
          "us",
          metric_sdk::AggregationType::kHistogram,
          server_config);

      views->AddView(std::move(server_instrument_selector),
                     std::move(server_meter_selector),
                     std::move(server_view));

      auto context = metric_sdk::MeterContextFactory::Create(std::move(views), resource);
      context->AddMetricReader(std::move(reader));

      meter_provider_ = metric_sdk::MeterProviderFactory::Create(std::move(context));

      meter_ = meter_provider_->GetMeter(options_.node_name);
      // channel
      chn_pub_msg_num_counter_ = meter_->CreateUInt64Counter("chn.pub.msg_num", "Total num of channel publish msg");
      chn_sub_msg_num_counter_ = meter_->CreateUInt64Counter("chn.sub.msg_num", "Total num of channel subscribe msg");

      chn_pub_msg_size_counter_ = meter_->CreateUInt64Counter("chn.pub.msg_size", "Total size of channel publish msg", "bytes");
      chn_sub_msg_size_counter_ = meter_->CreateUInt64Counter("chn.sub.msg_size", "Total size of channel subscribe msg", "bytes");

      // rpc
      rpc_client_invoke_num_counter_ = meter_->CreateUInt64Counter("rpc.client.invoke_num", "Total num of rpc client invoke");
      rpc_server_invoke_num_counter_ = meter_->CreateUInt64Counter("rpc.server.invoke_num", "Total num of rpc server invoke");

      rpc_client_req_size_counter_ = meter_->CreateUInt64Counter("rpc.client.req_size", "Total size of rpc client request", "bytes");
      rpc_client_rsp_size_counter_ = meter_->CreateUInt64Counter("rpc.client.rsp_size", "Total size of rpc client response", "bytes");
      rpc_server_req_size_counter_ = meter_->CreateUInt64Counter("rpc.server.req_size", "Total size of rpc server request", "bytes");
      rpc_server_rsp_size_counter_ = meter_->CreateUInt64Counter("rpc.server.rsp_size", "Total size of rpc server response", "bytes");

      rpc_client_time_cost_histogram_ = meter_->CreateDoubleHistogram("rpc.client.time_cost", "Time cost of rpc client", "us");
      rpc_server_time_cost_histogram_ = meter_->CreateDoubleHistogram("rpc.server.time_cost", "Time cost of rpc server", "us");
    }

    // register hook
    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostInitLog,
                                [this] { SetPluginLogger(); });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitChannel,
                                [this] { RegisterChannelFilter(); });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitRpc,
                                [this] { RegisterRpcFilter(); });

    plugin_options_node = options_;
    core_ptr_->GetPluginManager().UpdatePluginOptionsNode(Name(), plugin_options_node);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("Initialize failed, {}", e.what());
  }

  return false;
}

void OpenTelemetryPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;

    meter_.reset();
    meter_provider_.reset();

    tracer_.reset();
    trace_provider_.reset();

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

  if (enable_trace_) {
    channel_manager.AddPassedContextMetaKeys(
        {std::string(kCtxKeyStartNewTrace),
         std::string(kCtxKeyTraceParent),
         std::string(kCtxKeyTraceState)});

    channel_manager.RegisterPublishFilter(
        "otp_trace",
        [this](aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
               aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h) {
          ChannelTraceFilter(ChannelFilterType::kPublisher, true, msg_wrapper, std::move(h));
        });

    channel_manager.RegisterPublishFilter(
        "otp_simple_trace",
        [this](aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
               aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h) {
          ChannelTraceFilter(ChannelFilterType::kPublisher, false, msg_wrapper, std::move(h));
        });

    channel_manager.RegisterSubscribeFilter(
        "otp_trace",
        [this](aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
               aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h) {
          ChannelTraceFilter(ChannelFilterType::kSubscriber, true, msg_wrapper, std::move(h));
        });

    channel_manager.RegisterSubscribeFilter(
        "otp_simple_trace",
        [this](aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
               aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h) {
          ChannelTraceFilter(ChannelFilterType::kSubscriber, false, msg_wrapper, std::move(h));
        });
  }

  if (enable_metrics_) {
    channel_manager.RegisterPublishFilter(
        "otp_metrics",
        [this](aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
               aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h) {
          ChannelMetricsFilter(ChannelFilterType::kPublisher, msg_wrapper, std::move(h));
        });

    channel_manager.RegisterSubscribeFilter(
        "otp_metrics",
        [this](aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
               aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h) {
          ChannelMetricsFilter(ChannelFilterType::kSubscriber, msg_wrapper, std::move(h));
        });
  }
}

void OpenTelemetryPlugin::RegisterRpcFilter() {
  auto& rpc_manager = core_ptr_->GetRpcManager();

  if (enable_trace_) {
    rpc_manager.AddPassedContextMetaKeys(
        {std::string(kCtxKeyStartNewTrace),
         std::string(kCtxKeyTraceParent),
         std::string(kCtxKeyTraceState)});

    rpc_manager.RegisterClientFilter(
        "otp_trace",
        [this](const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
               aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h) {
          RpcTraceFilter(RpcFilterType::kClient, true, wrapper_ptr, std::move(h));
        });

    rpc_manager.RegisterClientFilter(
        "otp_simple_trace",
        [this](const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
               aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h) {
          RpcTraceFilter(RpcFilterType::kClient, false, wrapper_ptr, std::move(h));
        });

    rpc_manager.RegisterServerFilter(
        "otp_trace",
        [this](const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
               aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h) {
          RpcTraceFilter(RpcFilterType::kServer, true, wrapper_ptr, std::move(h));
        });

    rpc_manager.RegisterServerFilter(
        "otp_simple_trace",
        [this](const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
               aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h) {
          RpcTraceFilter(RpcFilterType::kServer, false, wrapper_ptr, std::move(h));
        });
  }

  if (enable_metrics_) {
    rpc_manager.RegisterClientFilter(
        "otp_metrics",
        [this](const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
               aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h) {
          RpcMetricsFilter(RpcFilterType::kClient, wrapper_ptr, std::move(h));
        });

    rpc_manager.RegisterServerFilter(
        "otp_metrics",
        [this](const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
               aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h) {
          RpcMetricsFilter(RpcFilterType::kServer, wrapper_ptr, std::move(h));
        });
  }
}

void OpenTelemetryPlugin::ChannelTraceFilter(
    ChannelFilterType type,
    bool upload_msg,
    aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
    aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h) {
  auto ctx_ref = msg_wrapper.ctx_ref;
  const auto& info = msg_wrapper.info;

  // if global force trace is set, or context force sets start_new_trace, or an upper layer passes a span,
  // then start a new span
  bool start_new_trace = options_.force_trace;

  if (!start_new_trace) {
    start_new_trace = common::util::CheckIEqual(ctx_ref.GetMetaValue(kCtxKeyStartNewTrace), "true");
  } else {
    ctx_ref.SetMetaValue(kCtxKeyStartNewTrace, "true");
  }

  ContextCarrier carrier(ctx_ref);

  // unpack the incoming context to get the parent span
  trace_api::StartSpanOptions op;
  op.kind = (type == ChannelFilterType::kPublisher)
                ? trace_api::SpanKind::kProducer
                : trace_api::SpanKind::kConsumer;

  opentelemetry::context::Context input_ot_ctx;
  auto extract_ctx = propagator_->Extract(carrier, input_ot_ctx);

  auto extract_ctx_val = extract_ctx.GetValue(trace_api::kSpanKey);
  if (!std::holds_alternative<std::monostate>(extract_ctx_val)) {
    auto parent_span = std::get<std::shared_ptr<::opentelemetry::trace::Span>>(extract_ctx_val);
    op.parent = parent_span->GetContext();
    start_new_trace = true;
  }

  // no need to start a new trace
  if (!start_new_trace) {
    h(msg_wrapper);
    return;
  }

  // need to start a new trace
  std::string span_name = msg_wrapper.info.topic_name + "/" + msg_wrapper.info.msg_type;
  auto span = tracer_->StartSpan(span_name, op);

  // publish msg first
  h(msg_wrapper);

  // pack current span's context
  opentelemetry::context::Context output_ot_ctx(trace_api::kSpanKey, span);
  propagator_->Inject(carrier, output_ot_ctx);

  // add base info
  span->SetAttribute("module_name", info.module_name);

  // add context attributes
  auto keys = ctx_ref.GetMetaKeys();
  for (auto& item : keys) {
    span->SetAttribute(item, ctx_ref.GetMetaValue(item));
  }

  if (upload_msg) {
    // serialize msg to json
    auto buf_ptr = aimrt::runtime::core::channel::TrySerializeMsgWithCache(msg_wrapper, "json");
    if (buf_ptr) {
      auto msg_str = buf_ptr->JoinToString();
      if (!msg_str.empty()) span->SetAttribute("msg_data", msg_str);
    }
  }

  span->End();
}

void OpenTelemetryPlugin::RpcTraceFilter(
    RpcFilterType type,
    bool upload_msg,
    const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
    aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h) {
  auto ctx_ref = wrapper_ptr->ctx_ref;

  // if global force trace is set, or context force sets start_new_trace, or an upper layer passes a span,
  // then start a new span
  bool start_new_trace = options_.force_trace;

  if (!start_new_trace) {
    start_new_trace = common::util::CheckIEqual(ctx_ref.GetMetaValue(kCtxKeyStartNewTrace), "true");
  } else {
    ctx_ref.SetMetaValue(kCtxKeyStartNewTrace, "true");
  }

  ContextCarrier carrier(ctx_ref);

  // unpack the incoming context to get the parent span
  trace_api::StartSpanOptions op;
  op.kind = (type == RpcFilterType::kClient)
                ? trace_api::SpanKind::kClient
                : trace_api::SpanKind::kServer;

  opentelemetry::context::Context input_ot_ctx;
  auto extract_ctx = propagator_->Extract(carrier, input_ot_ctx);

  auto extract_ctx_val = extract_ctx.GetValue(trace_api::kSpanKey);
  if (!std::holds_alternative<std::monostate>(extract_ctx_val)) {
    auto parent_span = std::get<std::shared_ptr<::opentelemetry::trace::Span>>(extract_ctx_val);
    op.parent = parent_span->GetContext();
    start_new_trace = true;
  }

  // no need to start a new trace
  if (!start_new_trace) {
    h(wrapper_ptr);
    return;
  }

  // need to start a new trace
  auto span = tracer_->StartSpan(ctx_ref.GetFunctionName(), op);

  // pack current span's context
  opentelemetry::context::Context output_ot_ctx(trace_api::kSpanKey, span);
  propagator_->Inject(carrier, output_ot_ctx);

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

        auto ctx_ref = wrapper_ptr->ctx_ref;
        const auto& info = wrapper_ptr->info;

        // add base info
        span->SetAttribute("module_name", info.module_name);

        // add context attributes
        auto keys = ctx_ref.GetMetaKeys();
        for (auto& item : keys) {
          span->SetAttribute(item, ctx_ref.GetMetaValue(item));
        }

        if (upload_msg) {
          // serialize req/rsp to json
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

void OpenTelemetryPlugin::ChannelMetricsFilter(
    ChannelFilterType type,
    aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
    aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h) {
  // publish msg first
  h(msg_wrapper);
  // get counter
  auto& msg_num_counter_ptr = (type == ChannelFilterType::kPublisher)
                                  ? chn_pub_msg_num_counter_
                                  : chn_sub_msg_num_counter_;
  auto& msg_size_counter_ptr = (type == ChannelFilterType::kPublisher)
                                   ? chn_pub_msg_size_counter_
                                   : chn_sub_msg_size_counter_;

  // make labels
  auto ctx_ref = msg_wrapper.ctx_ref;
  const auto& info = msg_wrapper.info;

  std::map<std::string, std::string> labels{
      {"topic_name", info.topic_name},
      {"msg_type", info.msg_type},
      {"module_name", info.module_name},
  };

  // TODO 这里是否有必要？对性能影响有多大？
  auto keys = ctx_ref.GetMetaKeys();
  for (auto& item : keys) {
    labels.emplace(item, ctx_ref.GetMetaValue(item));
  }

  // msg num
  msg_num_counter_ptr->Add(1, labels);

  // msg size
  std::string_view serialization_type;
  size_t msg_size = 0;

  const auto& serialization_cache = msg_wrapper.serialization_cache;

  if (serialization_cache.size() == 1) {
    serialization_type = serialization_cache.begin()->first;
    msg_size = serialization_cache.begin()->second->BufferSize();
  } else {
    auto serialization_type_span = info.msg_type_support_ref.SerializationTypesSupportedListSpan();

    for (auto item : serialization_type_span) {
      auto cur_serialization_type = aimrt::util::ToStdStringView(item);

      auto finditr = serialization_cache.find(cur_serialization_type);
      if (finditr == serialization_cache.end()) [[unlikely]]
        continue;

      serialization_type = cur_serialization_type;
      msg_size = finditr->second->BufferSize();
      break;
    }
  }

  msg_size_counter_ptr->Add(msg_size, labels);
}

void OpenTelemetryPlugin::RpcMetricsFilter(
    RpcFilterType type,
    const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
    aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h) {
  auto begin_time = std::chrono::steady_clock::now();
  wrapper_ptr->callback = [this, &type, &wrapper_ptr,
                           &begin_time,
                           callback{std::move(wrapper_ptr->callback)}](aimrt::rpc::Status status) {
    auto end_time = std::chrono::steady_clock::now();

    auto time_cost = std::chrono::duration_cast<std::chrono::microseconds>(
                         end_time - begin_time)
                         .count();

    const auto& info = wrapper_ptr->info;
    std::map<std::string, std::string> labels{
        {"module_name", info.module_name},
        {"func_name", info.func_name},
        {"status", std::string(status.GetCodeMsg(status.Code()))},
    };

    // choose ptr depends on type
    auto& rpc_msg_num_counter_ptr = (type == RpcFilterType::kClient)
                                        ? rpc_client_invoke_num_counter_
                                        : rpc_server_invoke_num_counter_;

    auto& rpc_msg_req_size_counter_ptr = (type == RpcFilterType::kClient)
                                             ? rpc_client_req_size_counter_
                                             : rpc_server_req_size_counter_;

    auto& rpc_msg_rsp_size_counter_ptr = (type == RpcFilterType::kClient)
                                             ? rpc_client_rsp_size_counter_
                                             : rpc_server_rsp_size_counter_;

    auto& time_cost_histogram_ptr = (type == RpcFilterType::kClient)
                                        ? rpc_client_time_cost_histogram_
                                        : rpc_server_time_cost_histogram_;

    time_cost_histogram_ptr->Record(time_cost, labels, opentelemetry::context::Context());

    // msg num
    rpc_msg_num_counter_ptr->Add(1, labels);

    size_t req_msg_size = 0, rsp_msg_size = 0;

    // req msg size
    const auto& req_serialization_cache = wrapper_ptr->req_serialization_cache;
    const auto& req_serialization_type_span = info.req_type_support_ref.SerializationTypesSupportedListSpan();
    std::string_view req_serialization_type;

    if (req_serialization_cache.size() == 1) {
      req_serialization_type = req_serialization_cache.begin()->first;
      req_msg_size = req_serialization_cache.begin()->second->BufferSize();
    } else {
      for (auto item : req_serialization_type_span) {
        auto cur_serialization_type = aimrt::util::ToStdStringView(item);

        auto finditr = req_serialization_cache.find(cur_serialization_type);
        if (finditr == req_serialization_cache.end()) [[unlikely]]
          continue;

        req_serialization_type = cur_serialization_type;
        req_msg_size = finditr->second->BufferSize();
        break;
      }
    }

    rpc_msg_req_size_counter_ptr->Add(req_msg_size, labels);

    // rsp msg size
    const auto& rsp_serialization_cache = wrapper_ptr->rsp_serialization_cache;
    const auto& rsp_serialization_type_span = info.rsp_type_support_ref.SerializationTypesSupportedListSpan();
    std::string_view rsp_serialization_type;

    if (rsp_serialization_cache.size() == 1) {
      rsp_serialization_type = rsp_serialization_cache.begin()->first;
      rsp_msg_size = rsp_serialization_cache.begin()->second->BufferSize();
    } else {
      for (auto item : rsp_serialization_type_span) {
        auto cur_serialization_type = aimrt::util::ToStdStringView(item);

        auto finditr = rsp_serialization_cache.find(cur_serialization_type);
        if (finditr == rsp_serialization_cache.end()) [[unlikely]]
          continue;

        rsp_serialization_type = cur_serialization_type;
        rsp_msg_size = finditr->second->BufferSize();
        break;
      }
    }
    rpc_msg_rsp_size_counter_ptr->Add(rsp_msg_size, labels);

    callback(status);
  };

  h(wrapper_ptr);
}
}  // namespace aimrt::plugins::opentelemetry_plugin