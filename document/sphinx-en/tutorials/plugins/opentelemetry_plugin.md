# opentelemetry Plugin

## Related Links

Reference example:
- {{ '[opentelemetry_plugin]({}/src/examples/plugins/opentelemetry_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**opentelemetry_plugin** is a plugin based on [OpenTelemetry](https://opentelemetry.io/) that provides framework-level observability capabilities for AimRT. It primarily operates through the RPC/Channel Framework Filter in AimRT. For concepts about Filters, please refer to the relevant sections in the [Basic Concepts of AimRT](../concepts/concepts.md) documentation.

In the current version, **opentelemetry_plugin** only supports trace functionality and partial metrics functionality for RPC and Channel. Future plans include improving metrics functionality for executors and services.

**opentelemetry_plugin** provides the following RPC/Channel Framework Filters:
- **Client filter**:
  - **otp_trace**: Used for RPC Client-side trace tracking, reporting data such as req and rsp, which is more resource-intensive;
  - **otp_simple_trace**: Used for RPC Client-side trace tracking, does not report data such as req and rsp, which is lightweight and has minimal performance impact;
- **Server filter**:
  - **otp_trace**: Used for RPC Server-side trace tracking, reporting data such as req and rsp, which is more resource-intensive;
  - **otp_simple_trace**: Used for RPC Server-side trace tracking, does not report data such as req and rsp, which is lightweight and has minimal performance impact;
- **Publish filter**:
  - **otp_trace**: Used for Channel Publish-side trace tracking, reporting msg data, which is more resource-intensive;
  - **otp_simple_trace**: Used for Channel Publish-side trace tracking, does not report msg data, which is lightweight and has minimal performance impact;
- **Subscribe filter**:
  - **otp_trace**: Used for Channel Subscribe-side trace tracking, reporting msg data, which is more resource-intensive;
  - **otp_simple_trace**: Used for Channel Subscribe-side trace tracking, does not report msg data, which is lightweight and has minimal performance impact;

The plugin configuration items are as follows:

| Node                      | Type      | Optional | Default      | Purpose |
| ----                      | ----      | ----  | ----        | ---- |
| node_name                 | string    | Required  | ""          | Node name for reporting, cannot be empty |
| trace_otlp_http_exporter_url  | string    | Optional  | ""          | URL for reporting trace via otlp http exporter. If trace reporting is not needed, this can be left unconfigured |
| metrics_otlp_http_exporter_url  | string    | Optional  | ""          | URL for reporting metrics via otlp http exporter. If metrics reporting is not needed, this can be left unconfigured |
| rpc_time_cost_histogram_boundaries | array     | Optional  | [1, 2 , 4, ... , 2147483648]          | List of boundary values for the histogram used when reporting RPC call times, in microseconds (us) |
| force_trace               | bool      | Optional  | false       | Whether to force trace reporting |
| attributes                | array     | Optional  | []          | List of key-value attributes attached when reporting from this node |
| attributes[i].key         | string    | Required  | ""          | Key value of the attribute |
| attributes[i].val         | string    | Required  | ""          | Value of the attribute |

After configuring the plugin:
- For trace functionality, you also need to register `otp_trace` or `otp_simple_trace` type filters in the `enable_filters` configuration under the `rpc`/`channel` node to enable trace tracking before and after rpc/channel calls.
- For metrics functionality, you also need to register `otp_metrics` type filters in the `enable_filters` configuration under the `rpc`/`channel` node to enable metrics tracking before and after rpc/channel calls.

### Trace Example
Here is a simple example of RPC and Channel communication based on the local backend with trace tracking enabled:
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

There are several ways to enable RPC/Channel trace functionality:

1. Force enable all traces under a node: Set the `force_trace` option in the plugin configuration to `true`.

2. Force enable trace tracking from an RPC Client or Channel Publish by setting `aimrt_otp-start_new_trace` to `True` in the Context's Meta information, for example:
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

3. Continue tracing a link from an RPC Client or Channel Publish by following the upper-layer RPC Server or Channel Subscribe, inheriting the Context from the upstream RPC Server/Channel Subscribe, for example:
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
Here is a simple example of RPC and Channel communication based on the local backend with metrics tracking enabled, setting `rpc_time_cost_histogram_boundaries` to define the boundary value list for the histogram used when reporting RPC call times, in microseconds (us):
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

OpenTelemetry has a clear positioning: unifying data collection and standard specifications. It does not cover how data is used, stored, displayed, or alerted. Currently, we recommend using Prometheus + Grafana for Metrics storage and display, and Jaeger for distributed trace storage and display. For detailed information about OpenTelemetry, Prometheus, and Jaeger, please refer to their respective official websites.

### Collector

Generally, if each service on a machine reports separately, it can lead to performance waste. In production practice, a local collector is usually used to gather all reporting information locally before uniformly reporting to a remote platform. OpenTelemetry officially provides a collector, which can be downloaded as a binary executable from the [opentelemetry-collector official website](https://github.com/open-telemetry/opentelemetry-collector) or installed via Docker.

Before starting the collector, a configuration file is needed. Refer to the following:
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

After creating the configuration file, the collector can be started:
```shell
otelcol --config=my-otel-collector-config.yaml
```

Or started via Docker:
```shell
docker run -itd -p 4317:4317 -p 4318:4318 -v /path/to/my-otel-collector-config.yaml:/etc/otelcol/config.yaml otel/opentelemetry-collector
```

### Jaeger

[Jaeger](https://www.jaegertracing.io/) is a distributed tracing and analysis platform compatible with the OpenTelemetry reporting standard. A Jaeger Docker instance can be simply started with the following command:
```shell
docker run -d \
  -e COLLECTOR_ZIPKIN_HOST_PORT=:9411 \
  -p 16686:16686 \
  -p 4317:4317 \
  -p 4318:4318 \
  -p 9411:9411 \
  jaegertracing/all-in-one:latest
```

After starting, configure the `trace_otlp_http_exporter_url` of the opentelemetry plugin or the exporters configuration of the collector to point to Jaeger's port 4318, thereby reporting trace information to the Jaeger platform. You can access Jaeger's web page on port 16686 to view trace information.