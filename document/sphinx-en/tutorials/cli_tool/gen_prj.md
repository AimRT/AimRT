# Generating Scaffold Code for New Projects

## Introduction
**aimrt_cli** can automatically generate the required project files by executing commands based on the configured yaml file.

A basic usage example is as follows:
```
aimrt_cli gen -p [your_required_configuration].yaml -o [your_required_output_folder]
```

You can also use `aimrt_cli -h/--help` to view the supported command-line options. It's worth noting that the output folder should preferably be an empty folder or a non-existent folder. Otherwise, aimrt_cli will report an error and terminate generation when encountering files with the same name. A typical case is an initial repository folder containing README.md. In this situation, you need to manually delete the README.md file before executing the generation command.

## Configuration File Interpretation
The code repository provides a configuration file example `configuration_example.yaml` in the `aimrt_cli` folder.

Here's its interpretation:

### Basic Information Configuration
```
# Basic information
base_info:
  project_name: test_prj
  build_mode_tags: ["EXAMPLE", "SIMULATION", "TEST_CAMERA"] # Build mode tags
  aimrt_import_options: # Some options when importing aimrt
    AIMRT_BUILD_RUNTIME: 'ON'
    AIMRT_USE_FMT_LIB: 'ON'
    AIMRT_BUILD_WITH_PROTOBUF: 'ON' # Note that single quotes are required, otherwise pyyaml will parse it as True
    AIMRT_USE_LOCAL_PROTOC_COMPILER: 'OFF'
    AIMRT_USE_PROTOC_PYTHON_PLUGIN: 'OFF'
    AIMRT_BUILD_WITH_ROS2: 'ON'
    # ...
```
`base_info` is a mandatory item in the configuration file, specifying some basic information about your project, where:
+ `project_name` is your project's name and also the namespace name in the generated code.
+ `build_mode_tags` are some customizable compilation options. If no custom options are needed, please set it as an empty list. Note that the compilation option generation format is `{PROJECT_NAME}_{OPTION_NAME}`.
+ `aimrt_import_options` introduces the compilation option configurations that come with aimrt. Note that the compilation options here must be from aimrt; defining incorrect ones will result in an error.

Note that pyyaml will directly parse ON in yaml files as True, so when specifying option parameters in yaml files, single quotes need to be added.

### Dependency Configuration for Standard Modules:
```
# Standard modules that your project depends on
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
`depends_std_modules` is not mandatory; if not needed, you can delete it or leave its content empty.
+ The `name` option is the name of the dependent standard module, which should match the name of the library to be pulled; otherwise, the name of the pulled library will be used.
+ `git_repository` is the address of the dependent library.
+ `git_tag` is the version of the library to be pulled.
+ `import_options` are import options, currently not supported.

### Protocol Configuration:
```
# Protocols
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
Here, you can customize the protocol content and types needed in your project.
The code generation tool will generate corresponding protocol modules based on this configuration.
`protocols` is not mandatory; if not needed, you can delete it or leave its content empty.
+ `name` is the name of the data protocol you want to define.
+ `type` is the category of the protocol you define, divided into `protobuf` and `ros2` types, generating corresponding protocol modules respectively.
+ `options` are optional parameters, currently not supported.
+ `build_mode_tag` is the compilation option for this protocol. Only when this option is specified in the compilation will this protocol be compiled. If not specified, it will be compiled by default.

Please note that the protocol configuration will only generate the corresponding protocol module based on the configuration content. Please customize the required data types in the generated module file.

### Module Configuration
```
# Modules
modules:
  - name: my_foo_module
  - name: my_bar_module
  - name: exmaple_module
    build_mode_tag: ["EXAMPLE"]
    options:
      aaa: aaa
```
Here, you can configure the modules you need to customize in your project. `modules` is not mandatory.
The code generation tool will generate a standard module template containing `<module_name>.cc, <module_name>.h, CMakeLists.txt` based on your configuration.
You can then modify and develop it according to your needs.
+ `name` is the name of the module you want to define.
+ `build_mode_tag` is the compilation option for this module. Only when this option is specified in the compilation will this module be compiled. If not specified, it will be compiled by default.
+ `options` are optional parameters, currently not supported.

### Module Package Configuration
```
# Module packages
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
In aimrt, a module package is defined as a collection of modules and is the smallest unit of deployment. `pkgs` is not mandatory.
Here, checks will be performed to see if the modules exist, but only for custom modules.
If an unrecognized external module is encountered, a warning will be issued, and the user must ensure its correctness.
+ `name` is the name of the module package you want to define.
+ `modules` are the modules associated with this module package.
  + The sub-item `name` refers to the name of the associated module.
  + The sub-item `namespace` refers to the namespace name where the module is located. If it's a custom module, it should be set to `local`; if it's an external module, it should be set to its namespace name.
+ `build_mode_tag` is the compilation option for this module package. Only when this option is specified in the compilation will this module package be compiled. If not specified, it will be compiled by default.
+ `options` are optional parameters, currently not supported.

### Deployment Configuration
```
# Deployment
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
Here, you can specify the deployment configuration for your project, telling how the module packages should be deployed and run. `deploy_modes` is not mandatory.
Where:
+ `name` is the name of the deployment class you want to define.
+ `build_mode_tag` is the compilation option for this type of deployment. This option here serves as a check; only when its associated pkgs are generated by default or meet the same compilation option will it take effect.
+ `deploy_ins` are the specific deployment configurations:
  + The sub-item `name` specifies the name of the specific deployment.
  + The sub-item `pkgs` specifies the names of the module packages that the deployment depends on. This will associate the modules' dynamic libraries when automatically generating the deployment configuration file. Note that if no module packages are configured, no specific deployment configuration will be generated.
  + Under the sub-item `pkgs`, you can also configure the `options` tag. Currently, `options` supports the `disable_modules` option, which can specify the module names in the pkg that will not be included in this deployment. During runtime, these modules will not be loaded.

Note that the code auto-generation tool does not generate specific configuration items for the associated modules in the deployment configuration file generated based on the deployment configuration. You need to specify them yourself.

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