

# HelloWorld Python

This chapter will demonstrate how to create a basic AimRT Python project through a simple Demo.

AimRT is built upon pybind11, wrapping a Python interface layer over the C++ API. This Demo will showcase the following fundamental features:
- Installing AimRT via pip
- Creating and using AimRT instances directly in the Main method using App mode
- Utilizing basic logging capabilities
- Employing basic configuration functionalities
- Executing Python scripts to experience AimRT's capabilities

## STEP1: Ensure Local Environment Requirements

First verify that your local Python environment meets the requirements and has the `aimrt_py` package installed. Refer to the steps in [References & Installation (Python)](installation_py.md) for details.


## STEP2: Write Business Code

Refer to the following code to create a Python file `helloworld_app.py`, which creates an AimRT instance, parses configuration files, and prints simple logs.

```python
import argparse
import threading
import time
import aimrt_py
import yaml


def main():
    parser = argparse.ArgumentParser(description='Helloworld app.')
    parser.add_argument('--cfg_file_path', type=str, default="", help='config file path')
    args = parser.parse_args()

    # create aimrt core
    core = aimrt_py.Core()

    # init aimrt core
    core_options = aimrt_py.CoreOptions()
    core_options.cfg_file_path = args.cfg_file_path
    core.Initialize(core_options)

    # create module handle
    module_handle = core.CreateModule("HelloWorldPyModule")

    # use cfg and log
    module_cfg_file_path = module_handle.GetConfigurator().GetConfigFilePath()
    with open(module_cfg_file_path, 'r') as file:
        data = yaml.safe_load(file)
        key1 = str(data["key1"])
        key2 = str(data["key2"])
        aimrt_py.info(module_handle.GetLogger(), "key1: {}, key2: {}.".format(key1, key2))

    # start aimrt core
    thread = threading.Thread(target=core.Start)
    thread.start()
    time.sleep(1)

    # use log
    count = 0
    while(count < 10):
        count = count + 1
        aimrt_py.info(module_handle.GetLogger(), "Conut : {}.".format(count))

    # shutdown aimrt core
    time.sleep(1)
    core.Shutdown()

    thread.join()


if __name__ == '__main__':
    main()

```

## STEP3: Create Configuration File

Below is a sample configuration file `helloworld_cfg.yaml`. Other contents in this configuration will be explained in subsequent chapters. Focus on two sections here:
- The `aimrt.log` node: Specifies logging details
- The `HelloWorldPyModule` node: Contains configurations for `HelloWorldPyModule` that can be read within the module

```yaml
aimrt:
  log: # log配置
    core_lvl: INFO # 内核日志等级，可选项：Trace/Debug/Info/Warn/Error/Fatal/Off，不区分大小写
    backends: # 日志backends
      - type: console # 控制台日志

# 模块自定义配置，框架会为每个模块生成临时配置文件，开发者通过Configurator接口获取该配置文件路径
HelloWorldPyModule:
  key1: val1
  key2: val2
```

## STEP4: Launch and Test

Copy both the Python script `helloworld_app.py` and configuration file `helloworld_cfg.yaml` to a directory, then execute the following command to run the script and observe the output logs:
```shell
python3 ./helloworld_app.py --cfg_file_path=./helloworld_cfg.yaml
```