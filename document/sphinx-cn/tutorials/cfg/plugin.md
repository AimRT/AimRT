# aimrt.plugin

## 配置项概述

`aimrt.plugin`配置项用于配置插件。其中的细节配置项说明如下：

| 节点                    | 类型          | 是否可选| 默认值 | 作用 |
| ----                    | ----          | ----  | ----  | ---- |
| plugins                 | array         | 可选  | []    | 各个插件的配置 |
| plugins[i].name         | string        | 必选  | ""    | 插件名称 |
| plugins[i].path         | string        | 可选  | ""    | 插件路径。如果是硬编码注册的插件不需要填 |
| plugins[i].options      | map           | 可选  | -     | 传递给插件的初始化配置，具体内容在各个插件章节介绍 |


`aimrt.plugin`使用注意点如下：
- `plugins`是一个数组，用于配置各个插件。
  - `plugins[i].name`用于配置插件名称。不允许出现重复的插件名称。
  - 如果配置了`plugins[i].path`，AimRT 框架会从该路径下加载对应的插件动态库文件。如果使用者基于 App 模式硬编码注册插件，则不需要配置此项。
  - `plugins[i].options`是 AimRT 传递给插件的初始化参数，这部分配置格式由各个插件定义，请参考对应插件的文档。


## 使用示例

以下是一个简单的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: xxx_plugin
        path: ./libxxx.so
        options:
          xxx_key: xxx_val
          yyy_key: yyy_val
```
