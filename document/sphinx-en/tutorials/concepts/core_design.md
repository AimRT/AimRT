# AimRT Core Design Philosophy

## Initialize Phase and Start Phase

In most domains, the operation process of a resident service is typically divided into two phases:
- **Initialization Phase**: Performs some lengthy initialization logic, occupying only the very beginning segment of the entire operation process. After successful initialization, it transitions to the operation phase.
- **Operation Phase**: Performs cyclic, efficient task processing, occupying most of the operation time until the process is stopped for some reason.

Based on the characteristics of these two phases, it becomes evident that ensuring the execution efficiency of the **operation phase** is more critical, while the execution efficiency of the **initialization phase** is relatively less important.

AimRT builds upon this theoretical premise by dividing the entire runtime into the `Initialize` phase and the `Start` phase. During the `Initialize` phase, it strives to allocate, register, and initialize all required runtime resources as much as possible, ensuring minimal additional resource allocation, registry locking, or similar operations during the `Start` phase, thereby optimizing the efficiency of the operation phase.

Concretely, in AimRT usage, this manifests as certain interfaces being callable only during the `Initialize` phase, while others are restricted to the `Start` phase. Additionally, AimRT guarantees that the `Initialize` phase is single-threaded, reducing developers' additional efforts in areas like thread safety. Furthermore, upon completion of `Initialize`, AimRT can clearly identify the required resources and communication relationships, generating an initialization report. Developers can view this report in logs or export it via certain parameters/interfaces.

It's important to note that AimRT's `Initialize` phase is solely the initialization phase of the AimRT framework itself, which may only be a part of the entire service's **initialization phase**. Users might still need to initialize some of their business logic during AimRT's `Start` phase, such as checking upstream/downstream resources or performing certain configurations, before truly entering the service's **operation phase**. The relationships between these phases are illustrated below:

![](./picture/pic_6.png)

## Separation of Logic Implementation and Deployment Execution

A key design philosophy of AimRT is decoupling logic development from actual deployment execution. When developers implement specific business logic—that is, when writing `Module` code—they don't need to concern themselves with the eventual **deployment method** or **communication method** during runtime. For example, when developing an RPC client module and an RPC server module, users only need to know that requests sent by the client will definitely be received and processed by the server, without worrying about where the client and server modules are ultimately deployed or how data is communicated between them. This is illustrated below:

![](./picture/pic_1.jpg)

After development is complete, users can decide the deployment and communication scheme based on actual requirements. For example:
- If the two modules can be compiled together, communication between the client and server can directly pass data pointers.
- If stability decoupling is required later, they can be deployed as two processes on the same server, with communication between the client and server occurring via shared memory or local loopback.
- If one module needs to be deployed on the robot side and the other on the cloud, communication between the client and server can be achieved via HTTP, TCP, etc.

These changes only require users to modify configurations or make minor adjustments to some code in the Pkg or Main functions, without altering any original logic code.

## Thread Resources in AimRT

AimRT advocates allocating all required resources during the initialization phase, and thread resources are no exception. Users need to declare them in the configuration file in advance. Generally, when an AimRT process runs, all active threads originate from three sources:
- Threads launched by the AimRT framework itself;
- Threads actively configured by users in the startup configuration file;
- Threads launched by plugins loaded by the AimRT framework;
- Threads launched by users in their business modules.

The first two belong to threads managed by the AimRT framework, the third to threads managed by plugins, and the last to threads managed by users themselves.

Without any configuration or plugin loading, the AimRT framework itself uses only two threads:
- **Main Thread**: The thread that calls the AimRT `Initialize` and `Start` methods, typically the main thread where the `main` function resides. After startup, AimRT usually blocks the main thread, waiting for a stop signal to call the `Shutdown` method.
- **Daemon Thread**: After startup, AimRT launches a daemon thread that periodically monitors the runtime status, issues alerts for prolonged blockages, and serves as the default logging thread.

If users configure executors in the configuration file, the AimRT framework will create corresponding threads or thread pool resources during the initialization phase for use during business operation. Developers are recommended to use thread resources uniformly managed by the AimRT framework rather than manually creating threads. Specific usage methods can be found in the executor-related documentation.

Beyond this, threads introduced by plugins should be referenced in the respective plugin documentation. Users can also use native thread APIs to create their own thread resources in their business code.

## Compatibility with Third-Party Ecosystems

AimRT delegates underlying communication to plugins, which also enables compatibility with third-party ecosystems. For example, when a `Module` publishes a message externally via a `Channel`, the plugin layer can encode it as a ROS2 message and send it to the native ROS2 system, enabling interoperability with native ROS2 nodes. Moreover, AimRT's underlying layer can load multiple plugins, allowing simultaneous compatibility with different third-party ecosystems. This is illustrated below:

![](./picture/pic_2.jpg)