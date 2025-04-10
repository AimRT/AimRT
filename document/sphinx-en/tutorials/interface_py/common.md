# Common Information


## Python 接口概览

AimRT 基于 pybind11 对 CPP 接口进行了封装，提供了一套 Python 接口，包名为`aimrt_py`，其风格与 CPP 接口非常类似。


与 Cpp 接口类似，Python 接口也包括两部分：
1. 用于业务逻辑开发的接口，核心是`CoreRef`句柄的使用；
2. 用于部署运行的接口，核心是如何获取`aimrt::CoreRef`句柄并区分 AimRT 运行阶段；


Python 业务逻辑开发接口的核心是`CoreRef`句柄，开发者可以通过此句柄来调用 AimRT 的各种能力，例如日志、RPC、Channel 等。关于`CoreRef`句柄的相关信息，请参考[CoreRef](./core_ref.md)文档。


与 Cpp 接口不同的是，Python 部署运行接口只支持 App 模式，详情请参考[Runtime Interface](./runtime.md)文档。


## Python 接口中的引用类型

Python 接口中，大部分句柄都是一种引用类型，具有以下特点：
- 这种引用类型一般比较轻量，拷贝传递不会有较大开销。
- 这种引用类型一般都提供了一个`operator bool()`的重载来判断引用是否有效。
- 调用这种引用类型中提供的接口时，如果引用为空，会抛出一个异常。


## AimRT 生命周期以及接口调用时机


参考[接口概述](../concepts/interface.md)，AimRT在运行时有三个主要的阶段：
- **Initialize** 阶段
- **Start** 阶段
- **Shutdown** 阶段

一些接口只能在某些特定阶段里被调用。后续文档中，如果不对接口做特殊说明，则默认该接口在所有阶段都可以被调用。


## 大部分接口的实际表现需要根据部署运行配置而定

在逻辑实现阶段，开发者只需要知道此接口在抽象意义上代表什么功能即可，至于在实际运行时的表现，则要根据部署运行配置而定，在逻辑开发阶段也不应该关心太多。

例如，开发者可以使用`log`接口打印一行日志，但这个日志最终打印到文件里还是控制台上，则需要根据运行时配置而定，开发者在写业务逻辑时不需要关心。


## Python 接口支持的协议

AimRT Python 接口目前只支持 Protobuf 协议作为 RPC、Channel 的消息类型，这与 CPP 接口有所不同。使用时依赖`protobuf`包，请确保本地 Python 环境中有合适版本的`protobuf`包。

