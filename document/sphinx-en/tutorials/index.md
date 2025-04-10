

# Tutorials

## Quick Start

Through this section of the documentation, you can learn how to reference and install AimRT, and quickly experience it by creating a `Hello World` program.

```{toctree}
:maxdepth: 1

quick_start/installation_cpp.md
quick_start/installation_py.md
quick_start/helloworld_cpp.md
quick_start/helloworld_py.md
```

## Concepts

Through this section of the documentation, you can understand some core concepts and design philosophies in AimRT.

```{toctree}
:maxdepth: 1

concepts/cmake.md
concepts/concepts.md
concepts/core_design.md
concepts/interface.md
```

## C++ Interface Documentation

You can learn about the usage of C++ interfaces through the following documentation.

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
```

## Python Interface Documentation

You can learn about the usage of Python interfaces through the following documentation.

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

## Configuration Documentation

You can learn detailed configuration methods for each component through the following documentation.

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

## Plugins

AimRT provides numerous official plugins. You can understand the functionality and configuration methods of each plugin through the following documentation.

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

If developers want to customize and develop their own plugins, they can refer to the following documentation.
```{toctree}
:maxdepth: 1

plugins/how_to_dev_plugin.md
```

## CLI Tool

AimRT provides a command-line tool to help developers quickly complete various operations.

```{toctree}
:maxdepth: 1

cli_tool/cli_tool.md
cli_tool/gen_prj.md
```

## Examples

AimRT offers detailed and comprehensive examples that developers can use for in-depth learning.

```{toctree}
:maxdepth: 1

examples/examples_cpp.md
examples/examples_py.md
examples/examples_plugins.md
```

## Others

```{toctree}
:maxdepth: 1

misc/questions.md
misc/performance_test/0.8.0/performance.md
misc/performance_test/0.10.0/cpp/performance_test_cpp.md
misc/performance_test/0.10.0/py/performance_test_py.md
```