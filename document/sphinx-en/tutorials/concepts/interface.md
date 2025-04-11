

# AimRT Interface Overview

This document introduces essential considerations when using AimRT interfaces.

## AimRT Interface Overview

AimRT provides interfaces for various development and application scenarios, including:
- **Module Logic Development Interfaces**: Used for developing business modules, supporting C, CPP, and Python interfaces. The CPP interface is wrapped based on C interface, and Python interface is wrapped based on CPP interface;
- **Instance Deployment and Runtime Interfaces**: Primarily used for APP mode deployment, supporting CPP and Python interfaces. The Python interface is wrapped based on CPP interface;
- **Runtime Configuration**: Yaml-based configuration for AimRT runtime;
- **Plugin Development Interfaces**: For developing AimRT plugins, only providing CPP interfaces;

A core philosophy in AimRT is the separation of **logic implementation** from actual **deployment execution**. Developers don't need to consider deployment methods during business logic development, while deployment adjustments don't require modifying business logic code.

For example, when developing RPC Client and Server logic, developers only need to ensure the Server receives Client requests, without caring about deployment locations or underlying communication methods. During deployment, users can choose communication methods (shared memory or network) based on actual deployment scenarios (edge/cloud, single/multiple nodes).

Corresponding to this design philosophy, AimRT interfaces are divided into two main parts:
- Interfaces for **business logic development** (e.g., logging, RPC invocation);
- Interfaces/configurations for **deployment execution** (e.g., module integration, communication method selection). Note: This includes both code interfaces (C++/Python) and configuration files;

## AimRT Interface Compatibility Strategy

AimRT's current compatibility strategies:
- **Module Logic Development Interfaces (C, CPP, Python)**: Strict API compatibility within each major version after v1.0.0 release;
- **Instance Deployment Interfaces**:
  - **Python**: Strict API compatibility within each major version after v1.0.0 release;
  - **CPP**: Partial interface compatibility within each major version after v1.0.0 release. Refer to interface-specific documentation;
- **Runtime Configuration (Yaml)**: Strict configuration compatibility within each major version after v1.0.0 release;
- **Plugin Development Interfaces (CPP)**: Partial interface compatibility within each major version after v1.0.0 release, consistent with **Instance Deployment Interfaces (CPP)**;

## C, C++ and Python Interfaces

AimRT currently supports C, C++, and Python interfaces. The C interface mainly addresses ABI stability in module development, while C++ and Python are primary development languages with complete documentation. Configuration files are universal across both languages.

The C++ interface offers more comprehensive features and better performance. The Python interface (wrapped via pybind11) doesn't fully support all features and has lower performance. Users should choose appropriate interfaces based on actual requirements.

## AimRT Runtime Lifecycle

AimRT runtime consists of three phases: **Initialize**, **Start**, and **Shutdown**:
- **Initialize Phase**:
  - Framework initialization;
  - Preliminary business initialization with resource allocation;
  - Thread-safe execution in main thread;
  - Some interfaces/resources remain unavailable;
- **Start Phase**:
  - Complete business initialization;
  - Launch business logic;
  - Full access to AimRT resources (RPC, thread pool tasks);
  - Main thread initiates business operations that may utilize multithreading;
  - Some Initialize-phase interfaces become unavailable;
- **Shutdown Phase**:
  - Typically triggered by signals (e.g., Ctrl+C);
  - Graceful termination of business and framework;
  - Main thread waits for all business logic to complete;
  - Most interfaces become unavailable;

Key principle: Resource allocation operations should only occur during **Initialize** phase to ensure **Start** phase efficiency and stability. Note that business initialization might require **Start** phase interfaces, meaning complete business initialization may occur after framework **Start** phase.