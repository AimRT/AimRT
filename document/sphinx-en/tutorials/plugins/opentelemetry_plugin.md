

# opentelemetry Plugin

## Related Links

Reference examples:
- {{ '[opentelemetry_plugin]({}/src/examples/plugins/opentelemetry_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**opentelemetry_plugin** is an [OpenTelemetry](https://opentelemetry.io/)-based plugin that provides framework-level observability capabilities for AimRT. It primarily works through the RPC/Channel Framework Filter in AimRT. For concepts about Filters, please refer to the relevant section in the [Basic Concepts of AimRT](../concepts/concepts.md) documentation.

In the current version, **opentelemetry_plugin** only supports trace functionality and partial metrics functionality for RPC/Channel. Future plans include improving metrics functionality for executors and services.

**opentelemetry_plugin** provides the following RPC/Channel Framework Filters:
- **Client filter**:
  - **otp_trace**: Used for RPC client-side tracing, reports req/rsp data (higher overhead)
  - **otp_simple_trace**: Used for lightweight RPC client-side tracing (lower overhead, no req/rsp reporting)
- **Server filter**:
  - **otp_trace**: Used for RPC server-side tracing, reports req/rsp data (higher overhead)
  - **otp_simple_trace**: Used for lightweight RPC server-side tracing (lower overhead, no req/rsp reporting)
- **Publish filter**:
  - **otp_trace**: Used for Channel publish-side tracing, reports msg data (higher overhead)
  - **otp_simple_trace**: Used for lightweight Channel publish-side tracing (lower overhead, no msg reporting)
- **Subscribe filter**:
  - **otp_trace**: Used for Channel subscribe-side tracing, reports msg data (higher overhead)
  - **otp_simple_trace**: Used for lightweight Channel subscribe-side tracing (lower overhead, no msg reporting)

Plugin configuration parameters:

| Node                      | Type      | Optional | Default      | Purpose |
| ----                      | ----      | ----     | ----         | ----    |
| node_name                 | string    | Required | ""           | Node name for reporting (cannot be empty) |
| trace_otlp_http_exporter_url  | string    | Optional | ""           | OTLP HTTP exporter URL for trace reporting |
| metrics_otlp_http_exporter_url  | string    | Optional | ""           | OTLP HTTP exporter URL for metrics reporting |
| rpc_time_cost_histogram_boundaries | array     | Optional | [1, 2 ,4, ... ,2147483648] | Histogram bucket boundaries for RPC call time metrics (microseconds) |
| force_trace               | bool      | Optional | false        | Enable forced trace reporting |
| attributes                | array     | Optional | []           | Custom key-value attributes for reporting |
| attributes[i].key         | string    | Required | ""           | Attribute key |
| attributes[i].val         | string    | Required | ""           | Attribute value |

After configuring the plugin:
- For trace functionality: Register `otp_trace` or `otp_simple_trace` filters in `enable_filters` under `rpc`/`channel` nodes
- For metrics functionality: Register `otp_metrics` filters in `enable_filters` under `rpc`/`channel` nodes

### Trace Example
A simple example demonstrating RPC/Channel communication with trace tracking using local backend:
```yaml
aimrt:
  plugin:
    plugins:
      - name: opentelemetry_plugin
        path: ./libaimrt_opentelemetry_plugin.so
        options:
          node_name: example_node
          trace_otlp_http_exporter_url: http://localhost:4318/v1/traces
          force_trace: true
          attributes:
            - key: sn
              val: 123456
  rpc:
    backends:
      - type: local
    clients_options:
      - func_name: "(.*)"
        enable_backends: [local]
        enable_filters: [otp_trace]
    servers_options:
      - func_name: "(.*)"
        enable_backends: [local]
        enable_filters: [otp_trace]
  channel:
    backends:
      - type: local
        options:
          subscriber_use_inline_executor: true
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [local]
        enable_filters: [otp_trace]
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [local]
        enable_filters: [otp_trace]
  module:
    # ...
```

Trace activation scenarios for RPC/Channel:

1. Force trace for all nodes: Set `force_trace` to `true` in plugin config

2. Force trace from RPC Client/Channel Publish:
  - RPC:
  ```cpp
  auto ctx_ptr = client_proxy->NewContextSharedPtr();
  ctx_ptr->SetMetaValue("aimrt_otp-start_new_trace", "True");

  auto status = co_await client_proxy->GetFooData(ctx_ptr, req, rsp);
  // ...
  ```
  - Channel:
  ```cpp
  auto ctx_ptr = publisher_proxy.NewContextSharedPtr();
  ctx_ptr->SetMetaValue("aimrt_otp-start_new_trace", "True");

  publisher_proxy.Publish(ctx_ptr, msg);
  // ...
  ```

3. Continue trace from upstream RPC Server/Channel Subscribe:
  - RPC:
  ```cpp
  // RPC Server Handle
  co::Task<rpc::Status> GetFooData(rpc::ContextRef server_ctx, const GetFooDataReq& req, GetFooDataRsp& rsp) {
    // ...

    // 继承上游 Server 的 Context 信息
    auto client_ctx_ptr = client_proxy->NewContextSharedPtr(server_ctx);

    auto status = co_await client_proxy->GetFooData(client_ctx_ptr, req, rsp);
    // ...
  }

  ```
  - Channel:
  ```cpp
  // Channel Subscribe Handle
  void EventHandle(channel::ContextRef subscribe_ctx, const std::shared_ptr<const ExampleEventMsg>& data) {
    // ...

    // 继承上游 Subscribe 的 Context 信息
    auto publishe_ctx = publisher_proxy.NewContextSharedPtr(subscribe_ctx);

    publisher_proxy.Publish(publishe_ctx, msg);
    // ...
  }
  ```

### Metrics Example
A simple example demonstrating metrics tracking with custom histogram boundaries (microseconds):
```yaml
aimrt:
  plugin:
    plugins:
      - name: opentelemetry_plugin
        path: ./libaimrt_opentelemetry_plugin.so
        options:
          node_name: example_node
          metrics_otlp_http_exporter_url: http://localhost:4318/v1/metrics
          rpc_time_cost_histogram_boundaries: [0, 50.0, 150.0, 350.0, 750.0, 1350.0] # unit: us, optional
          attributes:
            - key: sn
              val: 123456
  rpc:
    backends:
      - type: local
    clients_options:
      - func_name: "(.*)"
        enable_backends: [local]
        enable_filters: [otp_metrics]
    servers_options:
      - func_name: "(.*)"
        enable_backends: [local]
        enable_filters: [otp_metrics]
  channel:
    backends:
      - type: local
        options:
          subscriber_use_inline_executor: true
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [local]
        enable_filters: [otp_metrics]
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [local]
        enable_filters: [otp_metrics]
  module:
    # ...
```

## Common Practices

OpenTelemetry focuses on data collection and standardization. For data usage/storage/visualization/alerts, we recommend:
- Prometheus + Grafana for metrics
- Jaeger for distributed tracing

### Collector

Typical production deployment uses local OpenTelemetry Collector for aggregated reporting. Download from [opentelemetry-collector](https://github.com/open-telemetry/opentelemetry-collector) or use Docker.

Sample collector configuration:
```yaml
receivers:
  otlp:
    protocols:
      grpc:
        endpoint: 0.0.0.0:4317
      http:
        endpoint: 0.0.0.0:4318

processors:
  batch:
    timeout: 5s
    send_batch_size: 1024

exporters:
  otlphttp:
    endpoint: http://xx.xx.xx.xx:4318

service:
  pipelines:
    traces:
      receivers: [otlp]
      processors: [batch]
      exporters: [otlphttp]
```

Start collector:
```shell
otelcol --config=my-otel-collector-config.yaml
```

Docker deployment:
```shell
docker run -itd -p 4317:4317 -p 4318:4318 -v /path/to/my-otel-collector-config.yaml:/etc/otelcol/config.yaml otel/opentelemetry-collector
```

### Jaeger

Start Jaeger instance via Docker:
```shell
docker run -d \
  -e COLLECTOR_ZIPKIN_HOST_PORT=:9411 \
  -p 16686:16686 \
  -p 4317:4317 \
  -p 4318:4318 \
  -p 9411:9411 \
  jaegertracing/all-in-one:latest
```

Configure `trace_otlp_http_exporter_url` or collector to point to Jaeger's 4318 port. Access Jaeger UI at port 16686 for trace visualization.