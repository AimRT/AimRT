# opentelemetry plugin

## Related Links

Reference example:
- {{ '[opentelemetry_plugin]({}/src/examples/plugins/opentelemetry_plugin)'.format(code_site_root_path_url) }}


## Plugin Overview


**opentelemetry_plugin** is a plugin based on [OpenTelemetry](https://opentelemetry.io/) that provides framework-level observability features for AimRT. It mainly works based on the RPC/Channel Framework Filter in AimRT. For the concept of Filter, please refer to the relevant chapters in [Basic Concepts in AimRT](../concepts/concepts.md) documentation.


In the current version, **opentelemetry_plugin** only supports the trace function and part of the metrics functions for rpc and channel. In the future, it is planned to improve the metrics functions for the executor and service parts.


**opentelemetry_plugin** provides the following RPC/Channel Framework Filters:
- **Client filter**:
  - **otp_trace**: Used for RPC Client-side link tracing, will report req, rsp and other data, relatively heavy;
  - **otp_simple_trace**: Used for RPC Client-side link tracing, will not report req, rsp and other data, relatively lightweight, with less impact on performance;
- **Server filter**:
  - **otp_trace**: Used for RPC Server-side link tracing, will report req, rsp and other data, relatively heavy;
  - **otp_simple_trace**: Used for RPC Server-side link tracing, will not report req, rsp and other data, relatively lightweight, with less impact on performance;
- **Publish filter**:
  - **otp_trace**: Used for Channel Publish-side link tracing, will report msg data, relatively heavy;
  - **otp_simple_trace**: Used for Channel Publish-side link tracing, will not report msg data, relatively lightweight, with less impact on performance;
- **Subscribe filter**:
  - **otp_trace**: Used for Channel Subscribe-side link tracing, will report msg data, relatively heavy;
  - **otp_simple_trace**: Used for Channel Subscribe-side link tracing, will not report msg data, relatively lightweight, with less impact on performance;


The plugin configuration items are as follows:

| Node                      | Type      | Optional| Default      | Function |
| ----                      | ----      | ----  | ----        | ---- |
| node_name                 | string    | Required  | ""          | Node name when reporting, cannot be empty |
| trace_otlp_http_exporter_url  | string    | Optional  | ""          | URL for reporting trace via otlp http exporter, can be omitted if trace reporting is not needed |
| metrics_otlp_http_exporter_url  | string    | Optional  | ""          | URL for reporting metrics via otlp http exporter, can be omitted if metrics reporting is not needed |
| rpc_time_cost_histogram_boundaries | array     | Optional  | [1, 2 , 4, ... , 2147483648]          | Boundary value list of histogram used when reporting RPC call time, unit is us |
| force_trace               | bool      | Optional  | false       | Whether to force trace reporting |
| attributes                | array     | Optional  | []          | List of kv attributes attached when this node reports |
| attributes[i].key         | string    | Required  | ""          | Key value of the attribute |
| attributes[i].val         | string    | Required  | ""          | Value of the attribute |



After configuring the plugin,
- For the trace function, you also need to register filters of type `otp_trace` or `otp_simple_trace` in the `enable_filters` configuration under the `rpc`/`channel` node to perform trace tracking before and after rpc/channel calls
- For the metrics function, you also need to register filters of type `otp_metrics` in the `enable_filters` configuration under the `rpc`/`channel` node to perform metrics tracking before and after rpc/channel calls

### trace example
Below is a simple example of RPC and Channel communication based on the local backend with trace tracking:

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


The ways to enable trace functionality for RPC/Channel are divided into the following cases:

1. Force enable all traces under a node: At this time, you can set the `force_trace` option in the plugin configuration to `true`.

2. Force enable link tracing from an RPC Client or a Channel Publish, at this time you need to set `aimrt_otp-start_new_trace` to `True` in the Meta information of the Context, for example:
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


3. Continue tracing a link from an RPC Client or a Channel Publish following the upper-level RPC Server or Channel Subscribe, at this time you need to inherit the Context of the upstream RPC Server/Channel Subscribe, for example:
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


### metrics example
Below is a simple example of RPC and Channel communication based on the local backend with metrics tracking, setting `rpc_time_cost_histogram_boundaries`, the boundary value list of histogram used for reporting RPC call time, unit is us:

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

OpenTelemetry has a clear positioning: data collection and standard specification unification. For how to use, store, display, and alert on data, the official does not involve it. We currently recommend using Prometheus + Grafana for Metrics storage and display, and Jaeger for distributed tracing storage and display. For detailed introductions to OpenTelemetry, Prometheus, and Jaeger, please refer to the official websites of the corresponding components.


### collector

Generally speaking, if every service on a machine reports separately, it will cause performance waste. In production practice, a local collector is usually used to collect all local reporting information, and then uniformly report to the remote platform. OpenTelemetry officially provides a collector, which can be downloaded as a binary executable file from the [opentelemetry-collector official website](https://github.com/open-telemetry/opentelemetry-collector), or installed via docker.


Before starting the collector, a configuration file is also needed, reference as follows:

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


After creating the configuration file, you can start the collector:

```shell
otelcol --config=my-otel-collector-config.yaml
```


Or start via docker:

```shell
docker run -itd -p 4317:4317 -p 4318:4318 -v /path/to/my-otel-collector-config.yaml:/etc/otelcol/config.yaml otel/opentelemetry-collector
```



### Jaeger

[Jaeger](https://www.jaegertracing.io/) is a distributed tracing and analysis platform compatible with the opentelemetry reporting standard. You can simply start a Jaeger docker instance with the following command:

```shell
docker run -d \
  -e COLLECTOR_ZIPKIN_HOST_PORT=:9411 \
  -p 16686:16686 \
  -p 4317:4317 \
  -p 4318:4318 \
  -p 9411:9411 \
  jaegertracing/all-in-one:latest
```


After starting, you can point the trace_otlp_http_exporter_url configuration of the opentelemetry plugin, or the exporters configuration of the collector, to port 4318 opened by Jaeger, thereby reporting trace information to the Jaeger platform. You can visit Jaeger's web page on port 16686 to view trace information.