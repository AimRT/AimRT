

# Common Information

## Python Interface Overview

AimRT provides Python interfaces through pybind11 wrapper, packaged as `aimrt_py`, maintaining similar style to CPP interfaces.

Similar to Cpp interfaces, Python interfaces also consist of two parts:
1. Interfaces for business logic development, centered around `CoreRef` handle usage;
2. Interfaces for deployment runtime, focusing on obtaining `aimrt::CoreRef` handle and distinguishing AimRT runtime phases.

The core of Python business logic development interface is the `CoreRef` handle. Developers can use this handle to access various AimRT capabilities like logging, RPC, Channels, etc. For `CoreRef` handle details, refer to [CoreRef](./core_ref.md) documentation.

Different from Cpp interfaces, Python deployment runtime interface only supports App mode. See [Runtime Interface](./runtime.md) for details.

## Reference Types in Python Interface

In Python interface, most handles are reference types with following characteristics:
- These reference types are generally lightweight with minimal copy overhead
- They typically provide an `operator bool()` overload to check validity
- Calling interfaces on invalid references will throw an exception

## AimRT Lifecycle and Interface Call Timing

Referring to [Interface Overview](../concepts/interface.md), AimRT runtime has three main phases:
- **Initialize** phase
- **Start** phase
- **Shutdown** phase

Some interfaces can only be called during specific phases. In subsequent documentation, unless otherwise specified, interfaces are assumed callable in all phases.

## Actual Interface Behavior Depends on Runtime Configuration

During logic implementation, developers only need to understand the abstract functionality represented by interfaces. Actual runtime behavior depends on deployment configurations, which shouldn't concern developers during logic development phase.

For example, developers can use `log` interface to print logs, but final output destination (file/console) depends on runtime configuration and requires no attention during development.

## Supported Protocols in Python Interface

AimRT Python interface currently only supports Protobuf protocol for RPC and Channel messages, differing from CPP interface. Usage requires `protobuf` package - ensure proper version is installed in Python environment.