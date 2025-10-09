# AimRT Interface Overview

This document introduces some basic points to know when using the AimRT interface.

## AimRT Interface Overview

AimRT provides interfaces for a variety of development and deployment scenarios, generally including the following aspects:
- **Module Logic Development Interface**: Used to develop specific business modules, including C, CPP, and Python interfaces. The CPP interface is a wrapper around the C interface, and the Python interface is a wrapper around the CPP interface.
- **Instance Deployment and Runtime Interface**: Mainly used for deployment and runtime in APP mode, including CPP and Python interfaces. The Python interface is a wrapper around the CPP interface.
- **Instance Runtime Configuration**: AimRT runtime configuration in Yaml format.
- **Plugin Development Interface**: Used to develop AimRT plugins, provided only as a CPP interface.

In AimRT, an important idea is to separate **logic implementation** from actual **deployment and runtime**. Users do not need to care about the final deployment method when developing business logic, and changes that occur during final deployment do not require modification of the business logic code.

For example, when users write RPC Client and Server logic, they only need to know that the request initiated by the Client will definitely reach the Server, without needing to care where the Client and Server will be deployed or how the underlying data will be communicated at runtime. At deployment time, the user then decides whether the Client and Server are deployed on the edge or in the cloud, on a single physical node or across multiple physical nodes, and then selects an appropriate underlying communication method based on the deployment scenario, such as shared memory communication or network communication.

Therefore, corresponding to this design philosophy, the broad interfaces in AimRT are divided into two main parts:
- Interfaces that users need to know when developing **business logic**, such as how to log, how to call RPC, etc.
- Interfaces/configurations that users need to know when **deploying and running**, such as how to integrate modules, how to select underlying communication methods, etc. Note that what needs to be concerned here includes not only code-form interfaces based on languages like C++/Python, but also configuration items in configuration files.

## AimRT Interface Compatibility Policy

AimRT's current compatibility policy for the above interfaces is as follows:
- **Module Logic Development Interface (C, CPP, Python)**: After the official v1.0.0 release, strict API interface compatibility will be guaranteed within each major version.
- **Instance Deployment and Runtime Interface**:
  - **Python**: After the official v1.0.0 release, strict API interface compatibility will be guaranteed within each major version.
  - **CPP**: After the official v1.0.0 release, within each major version, strict API interface compatibility will be guaranteed for some interfaces. Please refer to the comments of each interface for details.
- **Instance Runtime Configuration (Yaml)**: After the official v1.0.0 release, strict configuration compatibility will be guaranteed within each major version.
- **Plugin Development Interface (CPP)**: After the official v1.0.0 release, within each major version, strict API interface compatibility will be guaranteed for some interfaces. Please refer to the comments of each interface for details. This part follows the same policy as **Instance Deployment and Runtime Interface (CPP)**.

## C Interface, C++ Interface, and Python Interface

AimRT currently supports interfaces in C, C++, and Python. However, overall, the C interface is mainly used for ABI stability issues during module logic development. The main development languages are still C++ and Python. AimRT provides comprehensive documentation for both, and regardless of whether the program is written using the C++ or Python interface, the configuration files are universal.

In AimRT, the CPP interface is more comprehensive and performs better. The Python interface is a wrapper around the CPP interface using pybind11 and does not fully support all features, with slightly worse performance. Users need to choose the appropriate development language and interface based on actual scenarios.

## AimRT Runtime Lifecycle

The AimRT framework has three major stages at runtime: **Initialize**, **Start**, **Shutdown**. The meaning of these three stages and what is done in each stage are as follows:
- **Initialize Stage**:
  - Initialize the AimRT framework;
  - Preliminarily initialize the business, applying for resources needed by the business within the AimRT framework;
  - Complete all initializations sequentially in the main thread, without starting business threads, all code is thread-safe;
  - Some interfaces or resources cannot be used in this stage and can only be used in the **Start** stage;
- **Start Stage**:
  - Fully initialize the business;
  - Start business-related logic;
  - Can start using all resources in AimRT, such as initiating RPC, dispatching tasks to thread pools, etc.;
  - Start each business sequentially in the main thread, businesses can then be scheduled to multi-threaded environments;
  - Some interfaces can only be called in the **Initialize** stage and cannot be called in this stage;
- **Shutdown Stage**:
  - Usually triggered by signals like ctrl-c;
  - Gracefully stop the business;
  - Gracefully stop the AimRT framework;
  - Block in the main thread and wait for all business logic to end;
  - Most interfaces cannot be used in this stage;

In short, some operations that apply for AimRT resources can only be done in the **Initialize** stage, to ensure that the AimRT framework will not have new resource applications or lock operations during business runtime, ensuring the efficiency and stability of the **Start** stage.

However, note that the **Initialize** stage here only refers to the initialization of AimRT. Some business initializations may require calling interfaces that the AimRT framework only opens in the **Start** stage to complete. Therefore, business initialization may need to be completed after the AimRT framework enters the **Start** stage and should not be confused with the **Initialize** of the AimRT framework.