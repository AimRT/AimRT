
# Iceoryx 插件


## 相关链接

参考示例：
- {{ '[iceoryx_plugin]({}/src/examples/plugins/iceoryx_plugin)'.format(code_site_root_path_url) }}


## 插件概述

**iceoryx_plugin**是一个基于[iceoryx](https://github.com/eclipse-iceoryx/iceoryx)实现的高性能通信插件，其提供了一种零拷贝共享内存的方式来在不同的进程之间通信，从而实现低延迟的数据传输，适用于对时延、频率有较高要求的本体通信场景。


编译**iceoryx_plugin**需要依赖**libacl**，在 ubuntu 上可以通过以下命令安装，若未安装则默认会不编译 iceoryx 插件：
```shell
sudo apt install libacl1-dev
```


**iceoryx_plugin**插件基于[iceoryx](https://github.com/eclipse-iceoryx/iceoryx)进行通信，在使用时，一定要先启动 iceoryx 提供的守护进程 Roudi，AimRT 在编译时也会默认编译 Roudi。用户在可终端输入如下命令启动：
```bash
./iox-roudi --config-file=/path/to/you/cfg/iox_cfg.toml
```

用户可以在命令行输入 --config-file=<YOUR_CONFIG_FILE_PATH> ，这是一个可选项，如果用户不输入，则使用默认配置。关于 Roudi 的配置，具体可以参考[iceoryx overview.md](https://github.com/eclipse-iceoryx/iceoryx/blob/main/doc/website/getting-started/overview.md)。需要强调的是，配置选项中可以通过 toml 配置文件对共享内存池进行配置，用户在使用前一定要确保开辟的共享内存池的大小适配实际的硬件资源，下面是一个例子：
```toml
# Adapt this config to your needs and rename it to e.g. roudi_config.toml
# Reference to https://github.com/eclipse-iceoryx/iceoryx/blob/main/iceoryx_posh/etc/iceoryx/roudi_config_example.toml

[general]
version = 1

[[segment]]

[[segment.mempool]]
size = 128
count = 10000

[[segment.mempool]]
size = 1024
count = 5000

[[segment.mempool]]
size = 16384
count = 1000

[[segment.mempool]]
size = 131072
count = 200
```
iceoryx 官方还提供了名为 `iox-introspection-client` 的监测工具，用于查看共享内存池的使用情况等信息， AimRT 在编译时也会默认编译该工具， 用户可以通过以下命令启动：
```bash
./iox-introspection-client --all
```


**iceoryx_plugin**提供了以下组件：
- `iceoryx`类型 Channel 后端


**iceoryx_plugin**的配置项如下：

|     节点      |     类型     | 是否可选 | 默认值 |                       作用                       |
| :-----------: | :----------: | :------: | :----: | :----------------------------------------------: |
| shm_init_size | unsigned int |   可选   |  1024  | 初始向共享内存池中租赁的共享内存大小，单位：byte |


以下是一个简单的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: iceoryx_plugin
        path: ./libaimrt_iceoryx_plugin.so
        options:
          shm_init_size: 2048
```


关于**iceoryx_plugin**的配置，使用注意点如下：
- `shm_init_size`表示初始分配的共享内存大小，默认 1k bytes。注意，在实际运行过程中，可能数据的尺寸大于所分配的共享内存大小，AimRT 采用一种动态扩容机制，会以 2 倍的增长速率进行扩容，直到满足数据需求，之后的共享内存申请大小会按照扩容后的大小进行申请。



## iceoryx 类型 Channel 后端


`iceoryx`类型的 Channel 后端是**iceoryx_plugin**中提供的一种 Channel 后端，用于通过 iceoryx 提供的共享内存方式来发布和订阅消息,其所有的配置项如下：

| 节点参数                         | 类型   | 是否可选 | 默认值 | 作用                                                         |
| -------------------------------- | ------ | -------- | ------ | ------------------------------------------------------------ |
| sub_default_executor             | string | 必选     | ""     | 订阅端用于监听的默认线程名称，用于处理订阅消息回调           |
| sub_topics_options               | array  | 可选     | []     | 订阅主题的配置选项数组，用于自定义每个主题的处理方式         |
| sub_topics_options[i].topic_name | string | 必选     | ""     | 订阅主题的名称，指定要监听的特定主题。支持正则匹配           |
| sub_topics_options[i].executor   | string | 必选     | ""     | 该主题使用的执行器名称，如果未指定则使用sub_default_executor |

- sub_default_executor: 订阅端用于监听的默认线程名称，用于处理订阅消息回调, 如果 sub_topics_options 未设置则所有使用 iceoryx 后端的话题均使用该默认执行器进行监听。
- sub_topics_options[i].topic_name 支持正则匹配，例如："(.*)" 表示匹配所有主题。
- sub_default_executor 和 sub_topics_options[i].executor 均需要在配置文件的 executor 字段先声明执行器。
- sub_default_executor 和 sub_topics_options[i].executor 的执行器线程数应设置为1。


以下是一个简单的发布端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: iceoryx_plugin
        path: ./libaimrt_iceoryx_plugin.so
        options:
          shm_init_size: 2048
  executor:
    executors:
      - name: iox_default_executor
        type: asio_thread
        options:
          thread_num: 1
          thread_bind_cpu: [8]
      - name: test_topic_0_executor
        type: asio_thread
        options:
          thread_num: 1
          thread_bind_cpu: [6]
  channel:
    backends:
      - type: iceoryx
        options:
          sub_default_executor: iox_default_executor
          sub_topics_options:
              - topic_name: "test_topic_0"
                executor: "test_topic_0_executor"
    pub_topics_options:
      - topic_name: "(.*)" 
        enable_backends: [iceoryx]

```

以下则是一个简单的订阅端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: iceoryx_plugin
        path: ./libaimrt_iceoryx_plugin.so
        options:
          shm_init_size: 2048
  executor:
  channel:
    backends:
      - type: iceoryx
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [iceoryx]
```


在 AimRT 发布端发布数据到订阅端这个链路上，Iceoryx 数据包格式整体分 4 段：
- 数据包长度， 4 字节
- 序列化类型，一般是`pb`或`json`
- context 区
  - context数量，1 字节，最大 255 个 context
  - context_1 key, 2 字节长度 + 数据区
  - context_2 key, 2 字节长度 + 数据区
  - ...
- 数据

```
| msg len [4 byte]
| n(0~255) [1 byte] | content type [n byte]
| context num [1 byte]
| context_1 key size [2 byte] | context_1 key data [key_1_size byte]
| context_1 val size [2 byte] | context_1 val data [val_1_size byte]
| context_2 key size [2 byte] | context_2 key data [key_2_size byte]
| context_2 val size [2 byte] | context_2 val data [val_2_size byte]
| ...
| msg data [len - 1 - n byte] |
```
