# Common Information

## Python Interface Overview

AimRT provides a set of Python interfaces encapsulated based on pybind11 for the CPP interfaces, with the package name `aimrt_py`. Its style is very similar to the CPP interfaces.

Similar to the Cpp interfaces, the Python interfaces also consist of two parts:
1. Interfaces for business logic development, with the core being the use of the `CoreRef` handle;
2. Interfaces for deployment and runtime, with the core being how to obtain the `aimrt::CoreRef` handle and distinguish the AimRT runtime phases.

The core of the Python business logic development interface is the `CoreRef` handle. Developers can use this handle to call various capabilities of AimRT, such as logging, RPC, Channel, etc. For related information about the `CoreRef` handle, please refer to the [CoreRef](./core_ref.md) documentation.

Unlike the Cpp interfaces, the Python deployment and runtime interfaces only support the App mode. For details, please refer to the [Runtime Interface](./runtime.md) documentation.

## Reference Types in Python Interfaces

In the Python interfaces, most handles are a type of reference with the following characteristics:
- These reference types are generally lightweight, and copying them does not incur significant overhead.
- These reference types usually provide an overloaded `operator bool()` to check if the reference is valid.
- When calling the interfaces provided by these reference types, if the reference is null, an exception will be thrown.

## AimRT Lifecycle and Interface Call Timing

Refer to the [Interface Overview](../concepts/interface.md). AimRT has three main phases during runtime:
- **Initialize** phase
- **Start** phase
- **Shutdown** phase

Some interfaces can only be called during specific phases. In subsequent documentation, if no special note is made about an interface, it means the interface can be called in all phases by default.

## Actual Behavior of Most Interfaces Depends on Deployment Runtime Configuration

During the logic implementation phase, developers only need to understand what functionality the interface represents in an abstract sense. As for its actual behavior during runtime, it depends on the deployment runtime configuration, and developers should not concern themselves too much with this during the logic development phase.

For example, developers can use the `log` interface to print a line of logs, but whether the log is ultimately printed to a file or the console depends on the runtime configuration. Developers do not need to worry about this when writing business logic.

## Protocols Supported by Python Interfaces

Currently, the AimRT Python interfaces only support the Protobuf protocol as the message type for RPC and Channel, which differs from the CPP interfaces. Usage depends on the `protobuf` package, so please ensure that the local Python environment has a suitable version of the `protobuf` package installed.