

# Frequently Asked Questions

## How does AimRT support different operating systems?

AimRT is written in standard C++ and theoretically supports any platform with a C++20 compatible compiler. For officially tested compiler support lists, please refer to the [References & Installation (CPP)](../quick_start/installation_cpp.md) documentation.

At the current stage, AimRT primarily supports Linux platforms. Windows platform support is relatively limited - only the framework core and some plugins can be compiled on Windows, while some third-party libraries required by plugins themselves lack Windows support. Most examples have only been verified on Linux, with fewer examples validated on Windows.

Support for macOS and other platforms is still planned and currently unverified.

## In RPC calls, why does the framework return AIMRT_RPC_STATUS_OK instead of server-provided error codes when services are unimplemented?

When using the **ROS2 RPC backend combined with ROS2 Srv**, since ROS2 itself doesn't support returning fields other than request_id and response, the framework cannot return server-provided error codes and will directly return AIMRT_RPC_STATUS_OK. 

For example, when a server-side service is not implemented, it should return AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED. However, due to the inherent limitations of this combination, the framework will only return AIMRT_RPC_STATUS_OK to the client.

Additionally, the `Status` error information generally only reflects framework-level errors (e.g., service not found, network errors, or serialization errors) for developers to troubleshoot framework issues. For business-level error reporting, developers are advised to add corresponding fields in business packages.