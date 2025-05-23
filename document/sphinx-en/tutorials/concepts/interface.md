# AimRT Interface Overview

This document introduces some basic knowledge when using the AimRT interfaces.

## AimRT Interface Overview

AimRT provides interfaces for various development and application scenarios, which can be broadly categorized into the following aspects:
- **Module Logic Development Interfaces**: Used for developing specific business modules, including C, CPP, and Python interfaces. The CPP interface is encapsulated based on the C interface, and the Python interface is encapsulated based on the CPP interface.
- **Instance Deployment and Runtime Interfaces**: Mainly used for deployment and execution in APP mode, including CPP and Python interfaces. The Python interface is encapsulated based on the CPP interface.
- **Runtime Configuration**: AimRT runtime configuration in Yaml format.
- **Plugin Development Interfaces**: Used for developing AimRT plugins, only providing CPP interfaces.

An important concept in AimRT is the separation of **logic implementation** from actual **deployment and execution**. When developing business logic, users do not need to care about the final deployment method, and changes during deployment do not require modifications to the business logic code.

For example, when writing RPC Client and Server logic, users only need to know that requests initiated by the Client will definitely be received by the Server, without worrying about where the Client and Server will be deployed or how the underlying data will communicate during runtime. During deployment, users need to decide whether the Client and Server will be deployed on the edge or in the cloud, or on a single physical node or multiple nodes, and then choose the appropriate underlying communication method, such as shared memory or network communication.

Therefore, corresponding to this design philosophy, AimRT broadly divides its interfaces into two main parts:
- Interfaces that users need to know when developing **business logic**, such as how to log, how to call RPC, etc.
- Interfaces/configurations that users need to know during **deployment and execution**, such as how to integrate modules, how to select underlying communication methods, etc. Note: This includes not only code-based interfaces in languages like C++/Python but also configuration file items.

## AimRT Interface Compatibility Policy

AimRT's current compatibility policies for various interfaces are as follows:
- **Module Logic Development Interfaces (C, CPP, Python)**: After the official release of v1.0.0, strict API interface compatibility will be guaranteed within each major version.
- **Instance Deployment and Runtime Interfaces**:
  - **Python**: After the official release of v1.0.0, strict API interface compatibility will be guaranteed within each major version.
  - **CPP**: After the official release of v1.0.0, strict API interface compatibility will be guaranteed for some interfaces within each major version. Refer to the comments of each interface for details.
- **Runtime Configuration (Yaml)**: After the official release of v1.0.0, strict configuration compatibility will be guaranteed within each major version.
- **Plugin Development Interfaces (CPP)**: After the official release of v1.0.0, strict API interface compatibility will be guaranteed for some interfaces within each major version. Refer to the comments of each interface for details. This part follows the same policy as the **Instance Deployment and Runtime Interfaces (CPP)**.

## C, C++, and Python Interfaces

AimRT currently supports interfaces in C, C++, and Python. However, the C interface is mainly used to address ABI stability issues during module logic development, while the primary development languages are C++ and Python. AimRT provides comprehensive documentation for both, and programs written using either C++ or Python interfaces share the same configuration files.

In AimRT, the CPP interface is more comprehensive and performs better, while the Python interface is a pybind11 encapsulation of the CPP interface and does not fully support all features, with slightly worse performance. Users should choose the appropriate development language and interface based on actual scenarios.

## AimRT Runtime Lifecycle

The AimRT framework has three major phases during runtime: **Initialize**, **Start**, and **Shutdown**. The significance and tasks of each phase are as follows:
- **Initialize Phase**:
  - Initialize the AimRT framework.
  - Preliminary initialization of the business, applying for resources required by the business within the AimRT framework.
  - All initialization is completed sequentially in the main thread without spawning business threads, ensuring thread safety for all code.
  - Some interfaces or resources cannot be used in this phase and must wait until the **Start** phase.
- **Start Phase**:
  - Complete business initialization.
  - Start business-related logic.
  - All AimRT resources can now be used, such as initiating RPC or submitting tasks to the thread pool.
  - Businesses are started sequentially in the main thread and can then be scheduled into multi-threaded environments.
  - Some interfaces can only be called during the **Initialize Phase** and not in this phase.
- **Shutdown Phase**:
  - Typically triggered by signals like ctrl-c.
  - Gracefully stop the business.
  - Gracefully stop the AimRT framework.
  - Blocking wait in the main thread for all business logic to complete.
  - Most interfaces cannot be used during this phase.

In short, operations that apply for AimRT resources can only be performed during the **Initialize** phase to ensure that no new resource requests or lock operations occur during the business runtime, guaranteeing the efficiency and stability of the **Start** phase.

However, note that the **Initialize** phase here refers only to the initialization of the AimRT framework. Some business initialization may require calling interfaces that are only available during the **Start** phase of the AimRT framework. Therefore, business initialization may only be completed after the AimRT framework enters the **Start** phase and should not be confused with the **Initialize** phase of the AimRT framework.