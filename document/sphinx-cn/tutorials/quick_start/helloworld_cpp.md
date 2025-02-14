
# HelloWorld CPP

本章将以一个简单的 Demo 来介绍如何建立一个最基本的 AimRT CPP 工程。

本 Demo 将演示以下几项基本功能：
- 基于 CMake FetchContent 通过源码引用 AimRT；
- 编写一个基础的基于 AimRT CPP 接口的`Module`；
- 使用基础的日志功能；
- 使用基础的配置功能；
- 以 App 模式集成`Module`；
- 编译项目，并运行进程以执行`Module`中的逻辑。


请注意：本文档演示的 APP 模式，仅能在源码引用 AimRT 这种方式下构建，无法在二进制安装、find_package 引用方式下构建。


## STEP1: 确保本地环境符合要求

请先确保本地的编译环境、网络环境符合要求，具体请参考[引用与安装（CPP）](installation_cpp.md)中的要求。


注意，示例本身是跨平台的，但本文档基于 linux 进行演示。


## STEP2: 创建目录结构，添加基本文件

参照以下目录结构创建文件：
```
├── CMakeLists.txt
├── cmake
│   └── GetAimRT.cmake
└── src
    ├── CMakeLists.txt
    ├── install
    │   └── cfg
    │       └── helloworld_cfg.yaml
    ├── module
    │   └── helloworld_module
    │       ├── CMakeLists.txt
    │       ├── helloworld_module.cc
    │       └── helloworld_module.h
    └── app
        └── helloworld_app
            ├── CMakeLists.txt
            └── main.cc
```

请注意，此处仅是一个供参考的路径结构，并非强制要求。但推荐您在搭建自己的工程时，为以下几个领域单独建立文件夹：
- install：存放部署时的一些配置、启动脚本等；
- module：存放业务逻辑代码；
- app：app 模式下，main 函数存放处，在 main 函数中注册业务 module；
- pkg：pkg 模式下，pkg 动态库入口方法存放处，在 pkg 中注册业务 module；


### File 1 : /CMakeLists.txt
根 CMake ，用于构建工程。
```cmake
cmake_minimum_required(VERSION 3.24)

project(helloworld LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

include(cmake/GetAimRT.cmake)

add_subdirectory(src)
```

### File 2 : /cmake/GetAimRT.cmake
此文件用于获取 AimRT，注意需要将`GIT_TAG`版本改为你想引用的版本：

```cmake
include(FetchContent)

FetchContent_Declare(
  aimrt
  GIT_REPOSITORY https://github.com/AimRT/aimrt.git
  GIT_TAG v1.x.x)

FetchContent_GetProperties(aimrt)

if(NOT aimrt_POPULATED)
  FetchContent_MakeAvailable(aimrt)
endif()
```

### File 3 : /src/CMakeLists.txt
引用 src 下的各个子目录。
```cmake
add_subdirectory(module/helloworld_module)
add_subdirectory(app/helloworld_app)
```

### File 4 : /src/module/helloworld_module/CMakeLists.txt
创建`helloworld_module`静态库。
```cmake
file(GLOB_RECURSE src ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

add_library(helloworld_module STATIC)
add_library(helloworld::helloworld_module ALIAS helloworld_module)

target_sources(helloworld_module PRIVATE ${src})

target_include_directories(
  helloworld_module
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)

# 引用aimrt_module_cpp_interface
target_link_libraries(
  helloworld_module
  PRIVATE yaml-cpp::yaml-cpp
  PUBLIC aimrt::interface::aimrt_module_cpp_interface)
```

### File 5 : /src/app/helloworld_app/CMakeLists.txt
创建`helloworld_app`可执行文件。
```cmake
file(GLOB_RECURSE src ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

add_executable(helloworld_app)

target_sources(helloworld_app PRIVATE ${src})

target_include_directories(
  helloworld_app
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(
  helloworld_app
  PRIVATE aimrt::runtime::core
          helloworld::helloworld_module)
```

## STEP3: 编写业务代码

业务逻辑主要通过 Module 来承载，参考以下代码实现一个简单的 Module，解析传入的配置文件并打印一些简单的日志。

### File 6 : /src/module/helloworld_module/helloworld_module.h
```cpp
#pragma once

#include "aimrt_module_cpp_interface/module_base.h"

class HelloWorldModule : public aimrt::ModuleBase {
public:
  HelloWorldModule() = default;
  ~HelloWorldModule() override = default;

  aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "HelloWorldModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

private:
  aimrt::CoreRef core_;
};
```

### File 7 : /src/module/helloworld_module/helloworld_module.cc
```cpp
#include "helloworld_module/helloworld_module.h"

#include "yaml-cpp/yaml.h"

bool HelloWorldModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  // Log
  AIMRT_HL_INFO(core_.GetLogger(), "Init.");

  try {
    // Read cfg
  auto file_path = core_.GetConfigurator().GetConfigFilePath();
  if (!file_path.empty()) {
    YAML::Node cfg_node = YAML::LoadFile(file_path.data());
    for (const auto& itr : cfg_node) {
      std::string k = itr.first.as<std::string>();
      std::string v = itr.second.as<std::string>();
      AIMRT_HL_INFO(core_.GetLogger(), "cfg [{} : {}]", k, v);
    }
  }

  } catch (const std::exception& e) {
    AIMRT_HL_ERROR(core_.GetLogger(), "Init failed, {}", e.what());
    return false;
  }

  AIMRT_HL_INFO(core_.GetLogger(), "Init succeeded.");

  return true;
}

bool HelloWorldModule::Start() {
  AIMRT_HL_INFO(core_.GetLogger(), "Start succeeded.");
  return true;
}

void HelloWorldModule::Shutdown() {
  AIMRT_HL_INFO(core_.GetLogger(), "Shutdown succeeded.");
}
```

## STEP4: 确定部署方案和配置

我们使用 App 模式，手动编写 Main 函数，将 HelloWorldModule 通过硬编码的方式注册到 AimRT 框架中。然后编写一份配置，以确定一些运行时细节。

### File 8 : /src/app/helloworld_app/main.cc
在以下示例 main 函数中，我们捕获了 kill 信号，以完成优雅退出。
```cpp
#include <csignal>
#include <iostream>

#include "core/aimrt_core.h"
#include "helloworld_module/helloworld_module.h"

using namespace aimrt::runtime::core;

AimRTCore *global_core_ptr_ = nullptr;

void SignalHandler(int sig) {
  if (global_core_ptr_ && (sig == SIGINT || sig == SIGTERM)) {
    global_core_ptr_->Shutdown();
    return;
  }
  raise(sig);
};

int32_t main(int32_t argc, char **argv) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  std::cout << "AimRT start." << std::endl;

  try {
    AimRTCore core;
    global_core_ptr_ = &core;

    // register module
    HelloWorldModule helloworld_module;
    core.GetModuleManager().RegisterModule(helloworld_module.NativeHandle());

    AimRTCore::Options options;
    options.cfg_file_path = argv[1];
    core.Initialize(options);

    core.Start();

    core.Shutdown();

    global_core_ptr_ = nullptr;
  } catch (const std::exception &e) {
    std::cout << "AimRT run with exception and exit. " << e.what() << std::endl;
    return -1;
  }

  std::cout << "AimRT exit." << std::endl;
  return 0;
}
```

### File 9 : /src/install/cfg/helloworld_cfg.yaml
以下是一个简单的示例配置文件。这个配置文件中的其他内容将在后续章节中介绍，这里关注两个地方：
- `aimrt.log`节点：此处指定了日志的一些细节。
- `HelloWorldModule`节点：此处为`HelloWorldModule`的配置，可以在模块中读取到。

```yaml
aimrt:
  log: # log配置
    core_lvl: INFO # 内核日志等级，可选项：Trace/Debug/Info/Warn/Error/Fatal/Off，不区分大小写
    backends: # 日志后端
      - type: console # 控制台日志

# 模块自定义配置，框架会为每个模块生成临时配置文件，开发者通过Configurator接口获取该配置文件路径
HelloWorldModule:
  key1: val1
  key2: val2
```

## STEP5: 启动并测试

完善代码之后，在 linux 上执行以下命令完成编译：
```shell
# cd to root path of project
cmake -B build
cd build
make -j
```

编译完成后，将生成的可执行文件`helloworld_app`和配置文件`helloworld_cfg.yaml`拷贝到一个目录下，然后执行以下命令运行进程，观察打印出来的日志：
```shell
./helloworld_app ./helloworld_cfg.yaml
```
