
# opentelemetry插件

## 相关链接

参考示例：
- {{ '[opentelemetry_plugin]({}/src/examples/plugins/opentelemetry_plugin)'.format(code_site_root_path_url) }}


## 插件概述


**opentelemetry_plugin**是一个基于[OpenTelemetry](https://opentelemetry.io/)的插件，为 AimRT 提供框架层面的可观测性功能。它主要基于 AimRT 中的 RPC/Channel Framework Filter 进行工作，关于 Filter 的概念请参考[AimRT 中的基本概念](../concepts/concepts.md)文档中的相关章节。


当前版本，**opentelemetry_plugin**仅支持了 trace 功能，后续还计划完善 mertric 等功能。


**opentelemetry_plugin**提供了以下这些 RPC/Channel Framework Filter：
- **Client filter**:
  - **otp_trace**：用于进行 RPC Client 端链路追踪，会上报 req、rsp 等数据，比较重；
  - **otp_simple_trace**：用于进行 RPC Client 端链路追踪，不会上报 req、rsp 等数据，比较轻量，对性能影响较小；
- **Server filter**:
  - **otp_trace**：用于进行 RPC Server 端链路追踪，会上报 req、rsp 等数据，比较重；
  - **otp_simple_trace**：用于进行 RPC Server 端链路追踪，不会上报 req、rsp 等数据，比较轻量，对性能影响较小；
- **Publish filter**:
  - **otp_trace**：用于进行 Channel Publish 端链路追踪，会上报 msg 数据，比较重；
  - **otp_simple_trace**：用于进行 Channel Publish 端链路追踪，不会上报 msg 数据，比较轻量，对性能影响较小；
- **Subscribe filter**:
  - **otp_trace**：用于进行 Channel Subscribe 端链路追踪，会上报 msg 数据，比较重；
  - **otp_simple_trace**：用于进行 Channel Subscribe 端链路追踪，不会上报 msg 数据，比较轻量，对性能影响较小；


插件的配置项如下：

| 节点                      | 类型      | 是否可选| 默认值      | 作用 |
| ----                      | ----      | ----  | ----        | ---- |
| node_name                 | string    | 必选  | ""          | 上报时的节点名称，不可为空 |
| trace_otlp_http_exporter_url  | string    | 必选  | ""          | 基于 otlp http exporter 上报 trace 时的 url |
| force_trace               | bool      | 可选  | false       | 是否强制上报 trace |
| attributes                | array     | 可选  | []          | 本节点上报时附带的 kv 属性列表 |
| attributes[i].key         | string    | 必选  | ""          | 属性的 key 值 |
| attributes[i].val         | string    | 必选  | ""          | 属性的 val 值 |


在配置了插件后，还需要在`rpc`/`channel`节点下的的`enable_filters`配置中注册`otp_trace`或`otp_simple_trace`类型的过滤器，才能在 rpc/channel 调用前后进行 trace 跟踪。


以下是一个简单的基于 local 后端进行 RPC、Channel 通信，并进行 trace 跟踪的示例：
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

RPC/Channel 的 trace 功能开启方式分为以下几种情况：

1. 强制开启一个节点下所有的 trace：此时可以将插件配置中的 `force_trace` 选项设置为 `true`。

2. 从一个 RPC Clinet 或一个 Channel Publish 强制开启链路追踪，此时需要向 Context 的 Meta 信息中设置`aimrt_otp-start_new_trace`为`True`，例如：
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

3. 从一个 RPC Clinet 或一个 Channel Publish 跟随上层 RPC Server 或 Channel Subscribe 继续追踪一个链路，此时需要继承上游的 RPC Server/Channel Subscribe 的 Context，例如：
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


## 常用实践

OpenTelemetry 的自身定位很明确：数据采集和标准规范的统一，对于数据如何去使用、存储、展示、告警，官方是不涉及的，我们目前推荐使用 Prometheus + Grafana 做 Metrics 存储、展示，使用 Jaeger 做分布式跟踪的存储和展示。关于 OpenTelemetry、Prometheus、Jaeger 的详细介绍，请参考对应组件的官网。


### collector

一般来说，如果一台机器上的每一个服务都单独去上报，会造成性能上的浪费，在生产实践中一般是用一个本地的 collector ，收集本地所有的上报信息，然后再统一上报到远端平台。OpenTelemetry 官方提供了一个 collector，可以在[opentelemetry-collector官网地址](https://github.com/open-telemetry/opentelemetry-collector)下载二进制可执行文件，或者通过 docker 安装。


在启动 collector 之前，还需要一个配置文件，参考如下：
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

在创建好配置文件之后，即可启动 collector：
```shell
otelcol --config=my-otel-collector-config.yaml
```

或者通过 docker 启动：
```shell
docker run -itd -p 4317:4317 -p 4318:4318 -v /path/to/my-otel-collector-config.yaml:/etc/otelcol/config.yaml otel/opentelemetry-collector
```


### Jaeger

[Jaeger](https://www.jaegertracing.io/)是一个兼容 opentelemetry 上报标准的分布式跟踪、分析平台，可以简单的使用以下命令启动一个 Jaeger docker 实例：
```shell
docker run -d \
  -e COLLECTOR_ZIPKIN_HOST_PORT=:9411 \
  -p 16686:16686 \
  -p 4317:4317 \
  -p 4318:4318 \
  -p 9411:9411 \
  jaegertracing/all-in-one:latest
```

启动之后，即可将 opentelemetry 插件的 trace_otlp_http_exporter_url 配置、或者是 collector 的 exporters 配置指向 Jaeger 所开的 4318 端口，从而将 trace 信息上报到 Jaeger 平台上。可以访问 Jaeger 在 16686 端口上的 web 页面查看 trace 信息。

