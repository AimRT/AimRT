# Frequently Asked Questions

## How does AimRT support different operating systems?

AimRT is written in standard C++ and theoretically supports any platform with a C++20 compatible compiler. For the officially tested compiler list, please refer to the [References and Installation (CPP)](../quick_start/installation_cpp.md) documentation.

At the current stage, AimRT primarily supports Linux platforms. Windows platform support is relatively limited - currently only the main AimRT framework and some plugins can be compiled on Windows, while some plugins depend on third-party libraries that don't support Windows. Most Examples have only been verified on Linux platforms, with fewer Examples validated on Windows.

Support for other platforms like macOS is still planned and hasn't been verified yet.

## In RPC calls, why does the framework return AIMRT_RPC_STATUS_OK instead of the server-provided error code when a service isn't implemented?

When using the **ROS2 RPC backend combined with ROS2 Srv**, since ROS2 itself doesn't support returning fields other than request_id and response, the framework side cannot return server-provided error codes and will directly return AIMRT_RPC_STATUS_OK instead.  
For example, when a service isn't implemented on the server side (which should return AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED), due to the aforementioned combination's limitations, the framework will only return AIMRT_RPC_STATUS_OK to the client.

Additionally, errors in the `Status` field generally only indicate framework-level issues, such as service not found, network errors, or serialization problems, helping developers troubleshoot framework-related problems. If developers need to return business-level errors, it's recommended to add corresponding fields in the business package.