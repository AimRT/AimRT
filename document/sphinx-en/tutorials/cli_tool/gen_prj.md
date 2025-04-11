

# Generate Scaffolding Code for New Projects

## Introduction
**aimrt_cli** can automatically generate required engineering files by executing commands based on configured yaml files.

A basic usage example is shown below:
```
aimrt_cli gen -p [your_required_configuration].yaml -o [your_required_output_folder]
```

You can also use `aimrt_cli -h/--help` to view supported command-line options. Note that the output folder should preferably be an empty folder or non-existent folder. If aimrt_cli encounters files with same names, it will report errors and terminate generation. A typical scenario is an initial repository folder containing README.md. In this case, you need to manually delete the README.md file before executing the generation command.

## Configuration File Interpretation
The code repository provides a configuration file example `configuration_example.yaml` under the `aimrt_cli` folder.

Here's the interpretation:

### Basic Information Configuration
```
# 基础信息
base_info:
  project_name: test_prj
  build_mode_tags: ["EXAMPLE", "SIMULATION", "TEST_CAMERA"] # 构建模式标签
  aimrt_import_options: # 引入aimrt时的一些选型
    AIMRT_BUILD_RUNTIME: 'ON'
    AIMRT_USE_FMT_LIB: 'ON'
    AIMRT_BUILD_WITH_PROTOBUF: 'ON' # 注意引号是必须的，否则pyyaml会将其解析为True
    AIMRT_USE_LOCAL_PROTOC_COMPILER: 'OFF'
    AIMRT_USE_PROTOC_PYTHON_PLUGIN: 'OFF'
    AIMRT_BUILD_WITH_ROS2: 'ON'
    # ...
```
`base_info` is a mandatory configuration item that specifies basic information for your project:
+ `project_name` is your project's name and also serves as the namespace name in code.
+ `build_mode_tags` are customizable compilation options. Keep it as an empty list if no custom options are needed. Note the compilation option format is `{PROJECT_NAME}_{OPTION_NAME}`
+ `aimrt_import_options` introduces aimrt's built-in compilation option configurations. These options must exist in aimrt - incorrect definitions will cause errors.


### Standard Module Dependency Configuration:
```
# 依赖的标准模块
depends_std_modules:
  - name: xxx
    git_repository: https://github.com/xxx/xxx.git
    git_tag: v0.1.5
    import_options:
      XXX: 'ON'
  - name: yyy
    git_repository: https://github.com/yyy/yyy.git
    git_tag: v0.1.11
```
Here you can specify external standard modules your project depends on.  
`depends_std_modules` is optional and can be deleted or left empty if not needed.
+ The `name` option is the dependent standard module name, which should match the library name to be pulled. Otherwise, the actual pulled library name will be used.
+ `git_repository` is the dependency library address.
+ `git_tag` specifies the library version to pull.
+ `import_options` are import options, currently unsupported.

### Protocol Configuration:
```
# 协议
protocols:
  - name: my_proto
    type: protobuf
    options:
      xxx: xxx
  - name: my_ros2_proto
    type: ros2
    options:
      zzz: zzz
    build_mode_tag: ["EXAMPLE"]
  - name: example_proto
    type: protobuf
    build_mode_tag: ["EXAMPLE"] #仅在EXAMPLE模式为true时构建。build_mode_tag未设置则表示默认在所有模式下都构建
```
Here you can customize protocols and their types needed in your project.  
The code generator will create corresponding protocol modules based on this configuration.  
`protocols` is optional and can be deleted or left empty if not needed.
+ `name` is your defined data protocol name.
+ `type` specifies protocol category: `protobuf` or `ros2`, generating corresponding protocol modules.
+ `options` are optional parameters, currently unsupported.
+ `build_mode_tag` is the compilation option for this protocol. The protocol will only be compiled when this option is specified. Default compilation occurs if unspecified.


### Module Configuration
```
# 模块
modules:
  - name: my_foo_module
  - name: my_bar_module
  - name: exmaple_module
    build_mode_tag: ["EXAMPLE"]
    options:
      aaa: aaa
```
Configure custom modules needed in your project here. `modules` is optional.  
The code generator will create a standard module template containing `<module_name>.cc, <module_name>.h, CMakeLists.txt` based on your configuration. Modify and develop it as needed.
+ `name` is your defined module name.
+ `build_mode_tag` is the compilation option for this module. The module will only be compiled when this option is specified. Default compilation occurs if unspecified.
+ `options` are optional parameters, currently unsupported.

### Module Package Configuration
```
# pkg
pkgs:
  - name: pkg1
    modules:
      - name: my_foo_module
        namespace: local
      - name: my_bar_module
        namespace: local
    options:
      sss: sss
  - name: pkg2
    modules:
      - name: exmaple_module
        namespace: local
      - name: ep_example_bar_module
        namespace: ep_example_aimrt_module
    build_mode_tag: ["EXAMPLE"]
    options:
      sss: sss
```
In aimrt, a package is defined as a collection of modules and the smallest deployment unit. `pkgs` is optional.  
The system checks module existence (only for custom modules).  
External modules will trigger warnings, requiring user verification.
+ `name` is your defined package name.
+ `modules` lists modules associated with this package:
  + Sub-item `name` refers to the associated module name
  + Sub-item `namespace` specifies the module's namespace. Use `local` for custom modules, and actual namespace for external modules.
+ `build_mode_tag` is the compilation option for this package. The package will only be compiled when this option is specified. Default compilation occurs if unspecified.
+ `options` are optional parameters, currently unsupported.

### Deployment Configuration
```
# 部署
deploy_modes:
  - name: exmaple_mode
    build_mode_tag: ["EXAMPLE"]
    deploy_ins:
      - name: local_ins_1
        pkgs:
          - name: pkg1
            options:
              disable_modules: []
      - name: local_ins_2
      - name: remote_ins_123

  - name: deploy_mode_1
  - name: deploy_mode_2
```
Specify project deployment configurations here. Tell how module packages should be deployed and run. `deploy_modes` is optional.  
+ `name` is your defined deployment class name.
+ `build_mode_tag` serves as a verification option. Only takes effect when associated packages are default-generated or meet the same compilation option.
+ `deploy_ins` contains concrete deployment configurations:
  + Sub-item `name` specifies deployment name.
  + Sub-item `pkgs` lists dependent package names. Deployment configuration files will associate these packages' dynamic libraries. No configuration files will be generated if no packages are configured.
  + Sub-item `pkgs` can configure `options`, currently supporting `disable_modules` to specify excluded module names in this deployment. These modules won't be loaded at runtime.


The final generated project structure from `configuration_example.yaml` is:
```
.
├── build.sh
├── cmake
│   ├── GetAimRT.cmake
│   ├── GetGTest.cmake
│   └── NamespaceTool.cmake
├── CMakeLists.txt
├── format.sh
├── README.md
├── src
│   ├── CMakeLists.txt
│   ├── install
│   │   ├── linux
│   │   │   └── bin
│   │   │       ├── cfg
│   │   │       │   └── exmaple_mode_local_ins_1_cfg.yaml
│   │   │       └── start_exmaple_mode_local_ins_1.sh
│   │   └── win
│   │       └── bin
│   │           ├── cfg
│   │           │   └── exmaple_mode_local_ins_1_cfg.yaml
│   │           └── start_exmaple_mode_local_ins_1.bat
│   ├── module
│   │   ├── exmaple_module
│   │   │   ├── CMakeLists.txt
│   │   │   ├── exmaple_module.cc
│   │   │   └── exmaple_module.h
│   │   ├── my_bar_module
│   │   │   ├── CMakeLists.txt
│   │   │   ├── my_bar_module.cc
│   │   │   └── my_bar_module.h
│   │   └── my_foo_module
│   │       ├── CMakeLists.txt
│   │       ├── my_foo_module.cc
│   │       └── my_foo_module.h
│   ├── pkg
│   │   ├── pkg1
│   │   │   ├── CMakeLists.txt
│   │   │   └── pkg_main.cc
│   │   └── pkg2
│   │       ├── CMakeLists.txt
│   │       └── pkg_main.cc
│   └── protocols
│       ├── example_proto
│       │   ├── CMakeLists.txt
│       │   └── example_proto.proto
│       ├── my_proto
│       │   ├── CMakeLists.txt
│       │   └── my_proto.proto
│       └── my_ros2_proto
│           ├── CMakeLists.txt
│           ├── msg
│           │   └── RosChangeMe.msg
│           └── package.xml
└── test.sh
```