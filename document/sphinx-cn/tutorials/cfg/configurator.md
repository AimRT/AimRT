# aimrt.configurator

## 配置项概述

`aimrt.configurator`配置项用于确定模块配置功能的一些细节。其中的细节配置项说明如下：


| 节点            | 类型          | 是否可选| 默认值 | 作用 |
| ----            | ----          | ----  | ----  | ---- |
| temp_cfg_path   | string        | 可选  | "./cfg/tmp" | 生成的临时模块配置文件存放路径 |


`aimrt.configurator`节点的使用说明如下：
- `temp_cfg_path`用于配置生成的临时模块配置文件存放路径。
  - 需要配置一个目录形式的路径。如果配置的目录不存在，则 AimRT 框架会创建一个。如果创建失败，会抛异常。
  - 如果使用者未为某个模块配置该项，但 AimRT 框架配置文件中存在以该模块名称命名的根节点，框架会在`temp_cfg_path`所配置的路径下为该模块生成一个临时配置文件，将 AimRT 框架配置文件中该模块的配置拷贝到此临时配置文件中。

## 使用示例

以下是一个简单的示例：
```yaml
aimrt:
  configurator:
    temp_cfg_path: ./cfg/tmp
```
