# Zenoh 插件


## 相关链接

参考示例：
- {{ '[zenoh_plugin]({}/src/examples/plugins/zenoh_plugin)'.format(code_site_root_path_url) }}


## 插件概述

**zenoh_plugin**是一个基于[zenoh.c](https://github.com/eclipse-zenoh/zenoh-c)实现的轻量级的、高效的、实时的数据传输插件，它旨在为分布式系统提供低延迟、高吞吐量的数据传输和处理能力。此插件提供了以下组件：
- `zenoh`类型 Channel 后端


在当前版本，本插件没有插件级的配置。由于 zenoh 自带的服务发现机制，用户不必手动配置对端地址。


**请注意**，该插件在编译过程依赖于[Rust](https://www.rust-lang.org/)，请确保运行环境中存在 Rust 编译器，否则该插件将不会编译。


以下是一个简单的示例：

```yaml
aimrt:
  plugin:
    plugins:
      - name: zenoh_plugin
        path: ./libaimrt_zenoh_plugin.so
```

## zenoh 类型 Channel 后端

`zenoh`类型的 Channel 后端是**zenoh_plugin**中提供的一种 Channel 后端，主要用来构建发布和订阅模型，当前版本暂时没有可配置项。


以下是一个简单的发布端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: zenoh_plugin
        path: ./libaimrt_zenoh_plugin.so
  channel:
    backends:
      - type: zenoh
    pub_topics_options:
      - topic_name: "(.*)" 
        enable_backends: [zenoh]
```


以下是一个简单的订阅端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: zenoh_plugin
        path: ./libaimrt_zenoh_plugin.so
channel:
    backends:
      - type: zenoh
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [zenoh]
```

以上示例中都使用 zeonh 的服务发现机制，即在统一网络中的两个端点可自动发现彼此并建立连接，因此在配置过程中不需要用户手动输入任何参数，降低使用复杂度。

在这个过程中，底层使用的 Zenoh Topic 名称格式为：`channel/${topic_name}/${message_type}`。其中，`${topic_name}`为 AimRT 的 Topic 名称，`${message_type}`为 url 编码后的 AimRT 消息名称。这个 Topic 被设置成为 Zenoh 的键表达式（Keyxpr），这是 Zenoh 的提供的资源标识符，只有键表达式匹配的订阅者和发布者才能够进行通信。

例如，AimRT Topic 名称为`test_topic`，消息类型为`pb:aimrt.protocols.example.ExampleEventMsg`，则最终 Zenoh 的 topic 名称为：`channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsg`。如果订阅者和发布者的 Topic 均为`channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsg`，则二者可以进行通信。

在AimRT发布端发布数据到订阅端这个链路上，Zenoh 数据包格式整体分 3 段：
- 序列化类型，一般是`pb`或`json`
- context 区
  - context 数量，1 字节，最大 255 个 context
  - context_1 key, 2 字节长度 + 数据区
  - context_2 key, 2 字节长度 + 数据区
  - ...
- 数据

数据包格式如下：
```
| n(0~255) [1 byte] | content type [n byte]
| context num [1 byte]
| context_1 key size [2 byte] | context_1 key data [key_1_size byte]
| context_1 val size [2 byte] | context_1 val data [val_1_size byte]
| context_2 key size [2 byte] | context_2 key data [key_2_size byte]
| context_2 val size [2 byte] | context_2 val data [val_2_size byte]
| ...
| msg data [len - 1 - n byte] |
```