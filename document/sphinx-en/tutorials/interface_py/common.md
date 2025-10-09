# Common Information


## Python Interface Overview

AimRT provides a set of Python interfaces by wrapping the C++ interfaces with pybind11, packaged as `aimrt_py`, whose style is very similar to the C++ interfaces.


Similar to the C++ interfaces, the Python interfaces also consist of two parts:
1. Interfaces for business logic development, centered around the use of the `CoreRef` handle;
2. Interfaces for deployment and runtime, centered on how to obtain the `aimrt::CoreRef` handle and distinguish the AimRT runtime phases;


The core of the Python business logic development interface is the `CoreRef` handle, through which developers can call various capabilities of AimRT, such as logging, RPC, Channel, etc. For details about the `CoreRef` handle, please refer to the [CoreRef](./core_ref.md) document.


Unlike the C++ interfaces, the Python deployment and runtime interface only supports App mode. For details, please refer to the [Runtime Interface](./runtime.md) document.


## Reference Types in Python Interface

In the Python interface, most handles are reference types with the following characteristics:
- These reference types are generally lightweight, and copying or passing them incurs little overhead.
- These reference types typically provide an overload of `operator bool()` to check whether the reference is valid.
- When calling the interfaces provided by such reference types, if the reference is empty, an exception will be thrown.


## AimRT Lifecycle and Interface Call Timing


Refer to [Interface Overview](../concepts/interface.md), AimRT has three main phases at runtime:
- **Initialize** phase
- **Start** phase
- **Shutdown** phase

Some interfaces can only be called in specific phases. In subsequent documents, unless otherwise specified, it is assumed that the interface can be called in all phases.


## Actual Behavior of Most Interfaces Depends on Deployment Configuration

During the logic implementation phase, developers only need to know what the interface represents in an abstract sense. The actual runtime behavior depends on the deployment configuration, and developers should not concern themselves too much with it during the logic development phase.

For example, developers can use the `log` interface to print a line of log, but whether this log is ultimately printed to a file or to the console depends on the runtime configuration, which developers do not need to worry about when writing business logic.


## Protocols Supported by Python Interface

The AimRT Python interface currently only supports the Protobuf protocol as the message type for RPC and Channel, which differs from the C++ interface. It depends on the `protobuf` package, so please ensure that a suitable version of the `protobuf` package is available in your local Python environment.