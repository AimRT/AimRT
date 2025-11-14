# Tutorials

## 快速开始

通过此部分文档，您可以了解到如何引用、安装 AimRT，并通过创建一个`Hello World`程序来快速体验 AimRT。

```{toctree}
:maxdepth: 1

quick_start/devcontainer.md
quick_start/installation_cpp.md
quick_start/installation_py.md
quick_start/helloworld_cpp.md
quick_start/helloworld_py.md
```

## 概念

通过此部分文档，您可以了解到 AimRT 中的一些核心概念和设计思想。

```{toctree}
:maxdepth: 1

concepts/cmake.md
concepts/concepts.md
concepts/core_design.md
concepts/interface.md
concepts/protocols.md
```

## CPP 接口文档

您可以通过以下文档了解 C++ 接口的用法。

```{toctree}
:maxdepth: 1

interface_cpp/common.md
interface_cpp/runtime.md
interface_cpp/core_ref.md
interface_cpp/module_base.md
interface_cpp/configurator.md
interface_cpp/executor.md
interface_cpp/logger.md
interface_cpp/parameter.md
interface_cpp/channel.md
interface_cpp/rpc.md
interface_cpp/context.md
```

## Python 接口文档

您可以通过以下文档了解 Python 接口的用法。

```{toctree}
:maxdepth: 1

interface_py/common.md
interface_py/runtime.md
interface_py/core_ref.md
interface_py/module_base.md
interface_py/configurator.md
interface_py/executor.md
interface_py/logger.md
interface_py/channel.md
interface_py/rpc.md
```

## 配置文档

您可以通过以下文档了解各个组件详细的配置方法。

```{toctree}
:maxdepth: 1

cfg/common.md
cfg/module.md
cfg/configurator.md
cfg/plugin.md
cfg/main_thread.md
cfg/guard_thread.md
cfg/executor.md
cfg/log.md
cfg/channel.md
cfg/rpc.md
```

## 插件

AimRT 提供了大量官方插件，您可以通过以下文档了解各个插件的功能和配置方法。

```{toctree}
:maxdepth: 1

plugins/net_plugin.md
plugins/mqtt_plugin.md
plugins/ros2_plugin.md
plugins/parameter_plugin.md
plugins/time_manipulator_plugin.md
plugins/log_control_plugin.md
plugins/topic_logger_plugin.md
plugins/opentelemetry_plugin.md
plugins/record_playback_plugin.md
plugins/zenoh_plugin.md
plugins/iceoryx_plugin.md
plugins/grpc_plugin.md
plugins/echo_plugin.md
plugins/proxy_plugin.md
```

如果开发者想定制开发自己的插件，可以参考以下文档。

```{toctree}
:maxdepth: 1

plugins/how_to_dev_plugin.md
```

## CLI 工具

AimRT 提供了一个命令行工具，可以帮助开发者快速完成一些操作。

```{toctree}
:maxdepth: 1

cli_tool/cli_tool.md
cli_tool/gen_prj.md
```

## 示例

AimRT 提供了详细且全面的示例，开发者可以基于示例进行深入学习。

```{toctree}
:maxdepth: 1

examples/examples_cpp.md
examples/examples_py.md
examples/examples_plugins.md
```

## 其他

```{toctree}
:maxdepth: 1

misc/questions.md
misc/performance_test/0.8.0/performance.md
misc/performance_test/0.10.0/cpp/performance_test_cpp.md
misc/performance_test/0.10.0/py/performance_test_py.md
misc/performance_test/1.0.0/cpp/performance_test_cpp.md
misc/performance_test/1.0.0/py/performance_test_py.md
misc/performance_test/1.2.0/cpp/performance_test_cpp.md
misc/performance_test/1.3.0/cpp/performance_test_cpp.md
```
