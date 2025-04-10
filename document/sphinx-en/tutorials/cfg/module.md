# aimrt.module

## 配置项概述

`aimrt.module`配置项主要用于配置模块的加载信息，以及模块对各个其他组件的特殊配置。这是一个可选配置项，其中的细节配置项说明如下：


| 节点                      | 类型          | 是否可选| 默认值 | 作用 |
| ----                      | ----          | ----  | ----  | ---- |
| pkgs                     | array         | 可选  | []    | 要加载的 Pkg 动态库配置 |
| pkgs[i].path             | string        | 必选  | ""    | 要加载的 Pkg 动态库路径 |
| pkgs[i].enable_modules   | string array  | 可选  | []    | 此动态库中要加载的模块名称，不可与 disable_modules 选项同时使用 |
| pkgs[i].disable_modules  | string array  | 可选  | []    | 此动态库中要屏蔽的模块名称，不可与 enable_modules 选项同时使用 |
| modules                  | array         | 可选  | []    | 模块详细配置 |
| modules[i].name          | string        | 必选  | ""    | 模块名称 |
| modules[i].enable        | bool          | 可选  | True  | 是否启用 |
| modules[i].log_lvl       | string        | 可选  | ${aimrt.log.default_module_lvl}    | 模块日志级别 |
| modules[i].cfg_file_path | string        | 可选  | ""    | 自定义模块配置文件路径 |


使用时请注意，在`aimrt.module`节点下：
- `pkg`是一个数组，用于要加载的 Pkg 动态库。
  - `pkgs[i].path`用于配置要加载的 Pkg 动态库路径。不允许出现重复的 Pkg 路径。如果 Pkg 文件不存在，AimRT 进程会抛出异常。
  - `pkgs[i].enable_modules`和`pkgs[i].disable_modules`用于配置要加载/屏蔽的模块，其生效逻辑如下：
    - 如果没有配置`enable_modules`或`disable_modules`，则加载全部模块；
    - 如果仅配置了`enable_modules`，则加载`enable_modules`中的所有模块；
    - 如果仅配置了`disable_modules`，则加载除了`disable_modules`中的其他所有模块；
    - 如果同时配置了`enable_modules`和`disable_modules`，则加载`enable_modules`中的所有模块，忽略`disable_modules`选项，并在初始化时告警；
- `modules`是一个数组，用于配置各个模块。
  - `modules[i].name`表示模块名称。不允许出现重复的模块名称。
  - `modules[i].log_lvl`用以配置模块日志等级。
    - 如果未配置此项，则默认值是`aimrt.log.default_module_lvl`节点所配置的值。
    - 关于可以配置的日志等级，请参考[aimrt.log](./log.md)文档。
  - `modules[i].cfg_file_path`用以配置自定义模块配置文件路径，此处配置关系到 Module 接口中`configurator`组件`config_file_path`方法返回的结果，其规则如下：
    - 如果使用者配置了此项，则`configurator`组件的`config_file_path`方法将返回此处配置的字符串内容；
    - 如果使用者未配置此项，且 AimRT 框架配置文件中也不存在以该模块名称命名的根节点，则`configurator`组件的`config_file_path`方法将返回空字符串。
    - 如果使用者未配置此项，但 AimRT 框架配置文件中存在以该模块名称命名的根节点，则`configurator`组件的`config_file_path`方法将返回一个临时配置文件路径，此临时配置文件将包含 AimRT 框架配置文件该模块名称节点下的内容。

## 使用示例

以下是一个简单的示例：
```yaml
aimrt:
  module:
    pkgs:
      - path: /path/to/libxxx_pkg.so
        enable_modules: [FooModule, BarModule]
    modules:
      - name: FooModule
        enable: True
        log_lvl: INFO
        cfg_file_path: /path/to/foo_module_cfg.yaml
      - name: BarModule
        enable: True
        log_lvl: WARN

BarModule:
  foo_key: foo_val
  bar_key: bar_val
```
