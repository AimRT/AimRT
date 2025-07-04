# 常见问题

## AimRT 对于不同操作系统的支持如何？

AimRT 使用标准 C++ 编写，理论上只要支持 C++20 编译器的平台都可以支持。AimRT 官方测试过支持的编译器列表可以参考[引用与安装（CPP）](../quick_start/installation_cpp.md)文档中的说明。

当前阶段，AimRT 主要支持 linux 平台。对 Windows 平台的支持比较有限，当前仅能保证 AimRT 框架主体和部分插件能够在 Windows 平台上编译，并且一些插件所依赖的第三方库本身不支持 Windows 平台。大部分 Example 也仅在 linux 平台上验证过，Windows 平台上经过验证的 Example 较少。

AimRT 对 macos 等其他平台的支持还在计划中，暂时没有验证过。

## RPC 调用中，为什么在服务未实现的情况下，框架侧返回 AIMRT_RPC_STATUS_OK 而不是服务端提供的错误码？

使用 **ROS2 RPC 后端和 ROS2 Srv 结合**时，由于 ROS2 本身不支持返回除 request_id 和 response 之外的其他字段，所以框架侧不会返回服务端提供的错误码，而是直接返回一个 AIMRT_RPC_STATUS_OK。
例如，服务端某服务未实现，本应返回一个 AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED 错误码，但是由于上述组合自身的限制，框架侧只会给客户端返回 AIMRT_RPC_STATUS_OK。

另外，`Status` 中的错误信息一般仅表示框架层面的错误，例如服务未找到、网络错误或者序列化错误等，供开发者排查框架层面的问题。如果开发者需要返回业务层面的错误，建议在业务包中添加相应的字段。


## fmt 版本使用问题

如果您需要在代码中使用 fmt 库，可以根据版本需求选择以下方案：

- **使用 AimRT 内置版本（10.2.1）**：开启 `AIMRT_USE_FMT_LIB` 选项并链接到 `aimrt::common::util` 对象
- **使用自定义版本**：关闭 `AIMRT_USE_FMT_LIB` 选项并自行管理 fmt 库的链接


