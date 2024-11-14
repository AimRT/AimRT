// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <vector>

#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"

#include "opentelemetry/context/propagation/text_map_propagator.h"
#include "opentelemetry/exporters/otlp/otlp_environment.h"
#include "opentelemetry/exporters/otlp/otlp_http.h"
#include "opentelemetry/exporters/otlp/otlp_http_exporter_factory.h"
#include "opentelemetry/exporters/otlp/otlp_http_exporter_options.h"
#include "opentelemetry/exporters/otlp/otlp_http_metric_exporter_factory.h"
#include "opentelemetry/exporters/otlp/otlp_http_metric_exporter_options.h"
#include "opentelemetry/metrics/meter_provider.h"
#include "opentelemetry/metrics/provider.h"
#include "opentelemetry/sdk/metrics/export/periodic_exporting_metric_reader_factory.h"
#include "opentelemetry/sdk/metrics/export/periodic_exporting_metric_reader_options.h"
#include "opentelemetry/sdk/metrics/meter_context.h"
#include "opentelemetry/sdk/metrics/meter_context_factory.h"
#include "opentelemetry/sdk/metrics/meter_provider.h"
#include "opentelemetry/sdk/metrics/meter_provider_factory.h"
#include "opentelemetry/sdk/metrics/metric_reader.h"
#include "opentelemetry/sdk/metrics/push_metric_exporter.h"
#include "opentelemetry/sdk/metrics/state/filtered_ordered_attribute_map.h"
#include "opentelemetry/sdk/metrics/view/view_registry.h"
#include "opentelemetry/sdk/metrics/view/view_registry_factory.h"
#include "opentelemetry/sdk/trace/batch_span_processor_factory.h"
#include "opentelemetry/sdk/trace/batch_span_processor_options.h"
#include "opentelemetry/sdk/trace/exporter.h"
#include "opentelemetry/sdk/trace/processor.h"
#include "opentelemetry/sdk/trace/recordable.h"
#include "opentelemetry/sdk/trace/simple_processor_factory.h"
#include "opentelemetry/sdk/trace/tracer_provider.h"
#include "opentelemetry/sdk/trace/tracer_provider_factory.h"
#include "opentelemetry/sdk/version/version.h"
#include "opentelemetry/trace/propagation/http_trace_context.h"
#include "opentelemetry/trace/provider.h"
#include "opentelemetry/trace/scope.h"
#include "opentelemetry/trace/span.h"
#include "opentelemetry/trace/span_context.h"
#include "opentelemetry/trace/span_metadata.h"
#include "opentelemetry/trace/tracer.h"
#include "opentelemetry/trace/tracer_provider.h"

#include "core/channel/channel_framework_async_filter.h"
#include "core/rpc/rpc_framework_async_filter.h"

namespace aimrt::plugins::opentelemetry_plugin {

class OpenTelemetryPlugin : public AimRTCorePluginBase {
 public:
  struct Options {
    std::string node_name;

    std::string trace_otlp_http_exporter_url;
    bool force_trace = false;

    std::string metrics_otlp_http_exporter_url;
    uint32_t metrics_export_interval_ms = 15000;
    uint32_t metrics_export_timeout_ms = 5000;
    std::vector<double> rpc_time_cost_histogram_boundaries;

    struct Attribute {
      std::string key;
      std::string val;
    };
    std::vector<Attribute> attributes;
  };

 public:
  OpenTelemetryPlugin() = default;
  ~OpenTelemetryPlugin() override = default;

  std::string_view Name() const noexcept override { return "opentelemetry_plugin"; }

  bool Initialize(runtime::core::AimRTCore* core_ptr) noexcept override;
  void Shutdown() noexcept override;

 private:
  void SetPluginLogger();
  void RegisterChannelFilter();
  void RegisterRpcFilter();

  enum class ChannelFilterType {
    kPublisher,
    kSubscriber
  };

  enum class RpcFilterType {
    kClient,
    kServer
  };

  void ChannelTraceFilter(
      ChannelFilterType type,
      bool upload_msg,
      aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
      aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h);

  void RpcTraceFilter(
      RpcFilterType type,
      bool upload_msg,
      const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
      aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h);

  void ChannelMetricsFilter(
      ChannelFilterType type,
      aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
      aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h);

  void RpcMetricsFilter(
      RpcFilterType type,
      const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
      aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h);

 private:
  runtime::core::AimRTCore* core_ptr_ = nullptr;

  Options options_;

  bool init_flag_ = false;

  std::shared_ptr<opentelemetry::context::propagation::TextMapPropagator> propagator_;

  // trace
  bool enable_trace_ = false;
  std::shared_ptr<opentelemetry::trace::TracerProvider> trace_provider_;
  std::shared_ptr<opentelemetry::trace::Tracer> tracer_;

  // metrics
  bool enable_metrics_ = false;
  std::shared_ptr<opentelemetry::metrics::MeterProvider> meter_provider_;
  std::shared_ptr<opentelemetry::metrics::Meter> meter_;

  using u64_counter = std::unique_ptr<opentelemetry::metrics::Counter<uint64_t>>;
  using db_histogram = std::unique_ptr<opentelemetry::metrics::Histogram<double>>;

  u64_counter chn_pub_msg_num_counter_;
  u64_counter chn_sub_msg_num_counter_;
  u64_counter chn_pub_msg_size_counter_;
  u64_counter chn_sub_msg_size_counter_;

  u64_counter rpc_client_invoke_num_counter_;
  u64_counter rpc_server_invoke_num_counter_;
  u64_counter rpc_client_req_size_counter_;
  u64_counter rpc_client_rsp_size_counter_;
  u64_counter rpc_server_req_size_counter_;
  u64_counter rpc_server_rsp_size_counter_;
  db_histogram rpc_client_time_cost_histogram_;
  db_histogram rpc_server_time_cost_histogram_;
};

}  // namespace aimrt::plugins::opentelemetry_plugin