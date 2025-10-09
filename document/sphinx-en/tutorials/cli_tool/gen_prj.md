# Generate Scaffolding Code for a New Project


## Introduction
**aimrt_cli** can automatically generate the required project files by executing commands based on a configured yaml file.

A basic usage example is as follows:

```
aimrt_cli gen -p [your_required_configuration].yaml -o [your_required_output_folder]
```


You can also use `aimrt_cli -h/--help` to view supported command-line options. It is worth noting that the output folder should preferably be an empty folder or a non-existent folder; otherwise, aimrt_cli will report an error and terminate generation when encountering files with the same name. A typical case is an initial repository folder containing README.md. In this situation, you need to manually delete the README.md file before executing the generation command.

## Configuration File Explanation
The repository provides a configuration file example `configuration_example.yaml` under the `aimrt_cli` folder.

Here is an explanation:

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

`base_info` is a required item in the configuration file, specifying some basic information about your desired project, where:
+ `project_name` is the name of your project and also the namespace name generated in the code.
+ `build_mode_tags` are some compilation options you can customize. If there are no options to customize, set it to an empty list. Note that the compilation option generation format is `{PROJECT_NAME}_{OPTION_NAME}`.
+ `aimrt_import_options` are the compilation option configurations introduced by aimrt itself. Note that the compilation options here must be from aimrt; if defined incorrectly, an error will be reported.

Note that pyyaml will directly parse ON in the yaml file as True, so when specifying option parameters in the yaml file, you need to add single quotes.

### Configuration of Dependent Standard Modules:

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

Here, you can specify some external standard modules that your project depends on.
`depends_std_modules` is not a required item; if not needed, it can be deleted or its content set to empty.
+ The `name` option is the name of the dependent standard module, which should be consistent with the name of the library to be pulled; otherwise, the name of the pulled library shall prevail.
+ `git_repository` is the address of the dependent library.
+ `git_tag` is the version of the library to be pulled.
+ `import_options` are import options, not supported yet.

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

Here, you can customize the protocol content and types required in your project.
The code generation tool will generate corresponding protocol modules based on this configuration.
`protocols` is not a required item; if not needed, it can be deleted or its content set to empty.
+ `name` is the name of the data protocol you need to define.
+ `type` is the category of the protocol you define, divided into `protobuf` and `ros2` types, generating corresponding protocol modules respectively.
+ `options` are optional parameters, not supported yet.
+ `build_mode_tag` is the compilation option for this protocol. Only when this option is specified during compilation will this protocol be compiled. If not specified, it will be compiled by default.

Please note that the protocol configuration will only generate corresponding protocol modules based on the configuration content. Please customize the data types you need in the generated module files.

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

Here, you can configure the modules you need to customize in your project. `modules` is not a required item.
The code generation tool will generate a standard module template containing `<module_name>.cc, <module_name>.h, CMakeLists.txt` based on your configuration.
You can then modify and develop it according to your needs.
+ `name` is the name of the module you need to define.
+ `build_mode_tag` is the compilation option for this module. Only when this option is specified during compilation will this module be compiled. If not specified, it will be compiled by default.
+ `options` are optional parameters, not supported yet.

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

In aimrt, a module package is defined as a collection of modules and is the smallest unit of deployment. `pkgs` is not a required item.
Here, it will check whether the module exists, and the detection is only for custom modules.
If an external module cannot be recognized, a warning will be issued, and the user needs to ensure it themselves.
+ `name` is the name of the module package you need to define.
+ `modules` are the modules associated with this module package.
  + The sub-item `name` refers to the name of the associated module.
  + The sub-item `namespace` refers to the namespace name where the module is located. If it is an associated custom module, it should be set to `local`; if it is an external module, it should be set to its namespace name.
+ `build_mode_tag` is the compilation option for this module package. Only when this option is specified during compilation will this module package be compiled. If not specified, it will be compiled by default.
+ `options` are optional parameters, not supported yet.

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

Here, you specify the deployment configuration for your project, telling the module package how to deploy and run again. `deploy_modes` is not a required item.
Where:
+ `name` is the name of the deployment class you need to define.
+ `build_mode_tag` is the compilation option corresponding to this deployment class. This option serves as a check here. Only when the associated pkgs are generated by default or meet the same compilation option can it take effect.
+ `deploy_ins` is the specific deployment configuration:
  + The sub-item `name` specifies the name of the specific deployment.
  + The sub-item `pkgs` specifies the names of the module packages that the deployment depends on. This will associate the dynamic libraries of the modules when automatically generating the deployment configuration file. Note that if no module packages are configured, no specific deployment configuration will be generated.
  + The sub-item `pkgs` can also configure the `options` tag. Currently, `options` supports configuring the `disable_modules` option, which can specify the module names that will not be included in the pkg under this deployment. At runtime, these modules will not be loaded.

Note that the code automatic generation tool does not generate specific configuration items for associated modules in the specific deployment configuration file generated based on the deployment configuration. You need to specify them yourself.

Finally, the specific project will be generated in the specified directory. The project structure generated by `configuration_example.yaml` is as follows:

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
