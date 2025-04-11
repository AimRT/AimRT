

# HelloWorld CPP

This chapter will demonstrate how to establish a basic AimRT CPP project through a simple Demo.

This Demo will showcase the following fundamental functionalities:
- Reference AimRT via CMake FetchContent through source code
- Write a basic `Module` based on AimRT CPP interface
- Use basic logging functionality
- Use basic configuration functionality
- Integrate `Module` in App mode
- Compile the project and run the process to execute logic in `Module`

**Note**: The APP mode demonstrated in this document can only be built through source code reference of AimRT, and cannot be built using binary installation or find_package reference methods.

## STEP1: Ensure Local Environment Meets Requirements

First ensure that your local compilation environment and network environment meet the requirements. For details, please refer to [Reference and Installation (CPP)](installation_cpp.md).

**Note**: The example itself is cross-platform, but this documentation demonstrates based on Linux.

## STEP2: Create Directory Structure and Add Basic Files

Create files according to the following directory structure:
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

Please note that this is just a reference directory structure, not mandatory. However, it's recommended to create separate folders for the following areas when building your own project:
- install: Stores deployment configurations and startup scripts
- module: Stores business logic code
- app: In App mode, stores main function that registers business modules
- pkg: In Pkg mode, stores pkg dynamic library entry methods that register business modules

### File 1 : /CMakeLists.txt
Root CMake file for building the project.
```cmake
cmake_minimum_required(VERSION 3.24)

project(helloworld LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
endif()

include(cmake/GetAimRT.cmake)

add_subdirectory(src)
```

### File 2 : /cmake/GetAimRT.cmake
This file fetches AimRT. Note to modify the `GIT_TAG` version to your desired version:
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
References subdirectories under src.
```cmake
add_subdirectory(module/helloworld_module)
add_subdirectory(app/helloworld_app)
```

### File 4 : /src/module/helloworld_module/CMakeLists.txt
Creates `helloworld_module` static library.
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
Creates `helloworld_app` executable.
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

## STEP3: Implement Business Logic

Business logic is mainly implemented through Modules. Implement a simple Module with following code to parse configuration files and print basic logs.

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

## STEP4: Determine Deployment Solution and Configuration

Using App mode, manually write main function to register HelloWorldModule into AimRT framework through hardcoding. Then create a configuration file to determine runtime details.

### File 8 : /src/app/helloworld_app/main.cc
In the following example main function, we capture kill signals for graceful exit.
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
A simple example configuration file. Other contents will be explained in subsequent chapters. Focus on two sections:
- `aimrt.log` node: Specifies logging details
- `HelloWorldModule` node: Configuration for HelloWorldModule, accessible within the module
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

## STEP5: Launch and Test

After completing the code, execute the following commands on Linux for compilation:
```shell
# cd to root path of project
cmake -B build
cd build
make -j
```

After compilation, copy the generated executable `helloworld_app` and configuration file `helloworld_cfg.yaml` to a directory. Then execute the following command to run the process and observe the output logs:
```shell
./helloworld_app ./helloworld_cfg.yaml
```