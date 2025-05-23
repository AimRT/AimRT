# HelloWorld Python

This chapter will introduce how to set up the most basic AimRT Python project through a simple demo.

AimRT is built upon pybind11, with a Python interface layer wrapped on top of the CPP API layer. This demo will demonstrate the following basic functionalities:
- Installing AimRT via pip;
- Creating an AimRT instance directly in the Main method using App mode and utilizing its features;
- Using basic logging functionality;
- Using basic configuration functionality;
- Running Python scripts to experience AimRT's capabilities.

## STEP1: Ensure Local Environment Meets Requirements

First, please ensure your local Python environment meets the requirements and has the `aimrt_py` package installed. For specific steps, refer to the [References & Installation (Python)](installation_py.md) section.


## STEP2: Write Business Code

Refer to the following code to create a Python file `helloworld_app.py`, which initializes an AimRT instance, parses the input configuration file, and prints some simple logs.

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

## STEP3: Prepare Configuration File

Below is a simple example configuration file `helloworld_cfg.yaml`. Other contents in this configuration file will be introduced in later chapters. Here we focus on two sections:
- The `aimrt.log` node: Specifies details about logging.
- The `HelloWorldPyModule` node: Contains configurations for `HelloWorldPyModule`, which can be read within the module.

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

Copy both the Python script `helloworld_app.py` and the configuration file `helloworld_cfg.yaml` into the same directory, then execute the following command to run the script and observe the printed logs:
```shell
python3 ./helloworld_app.py --cfg_file_path=./helloworld_cfg.yaml
```