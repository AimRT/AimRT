# Frequently Asked Questions

## How does AimRT support different operating systems?

AimRT is written in standard C++ and theoretically supports any platform with a C++20 compiler. The list of officially tested compilers can be found in the [Installation (CPP)](../quick_start/installation_cpp.md) documentation.

Currently, AimRT primarily supports the Linux platform. Support for Windows is limited; only the core AimRT framework and some plugins can be compiled on Windows, and some third-party libraries that plugins depend on do not support Windows. Most examples have only been validated on Linux, with few verified on Windows.

Support for macOS and other platforms is planned but has not yet been validated.

## In RPC calls, why does the framework return AIMRT_RPC_STATUS_OK instead of the error code provided by the server when the service is not implemented?

When using the **ROS2 RPC backend combined with ROS2 Srv**, because ROS2 itself does not support returning fields other than request_id and response, the framework will not return the error code provided by the server, but will directly return AIMRT_RPC_STATUS_OK.
For example, if a service on the server is not implemented and should return AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED, due to the limitations of this combination, the framework will only return AIMRT_RPC_STATUS_OK to the client.

Additionally, the error information in `Status` generally only indicates framework-level errors, such as service not found, network errors, or serialization errors, to help developers troubleshoot framework-level issues. If developers need to return business-level errors, it is recommended to add corresponding fields in the business package.

## fmt version usage issues

If you need to use the fmt library in your code, you can choose from the following options based on version requirements:

- **Use AimRT's built-in version (10.2.1)**: Enable the `AIMRT_USE_FMT_LIB` option and link to the `aimrt::common::util` target
- **Use a custom version**: Disable the `AIMRT_USE_FMT_LIB` option and manage the fmt library linking yourself