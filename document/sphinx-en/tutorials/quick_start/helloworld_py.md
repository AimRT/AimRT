# HelloWorld Python

This chapter will use a simple demo to introduce how to build the most basic AimRT Python project.

AimRT is based on pybind11 and wraps a Python interface on top of the CPP interface layer. This demo will demonstrate the following basic features:
- Install AimRT via pip;
- Use App mode to directly create an AimRT instance in the Main method and use its features;
- Use basic logging functionality;
- Use basic configuration functionality;
- Run a Python script to experience AimRT's features.

## STEP1: Ensure the local environment meets requirements

Please first ensure that your local Python environment meets the requirements and that the `aimrt_py` package is installed. For specific steps, please refer to [References and Installation (Python)](installation_py.md).

Note that the example itself is cross-platform, but this document is demonstrated based on Linux.

## STEP2: Write business code

Refer to the following code to write a Python file `helloworld_app.py`, which creates an aimrt instance, parses the incoming configuration file, and prints some simple logs.


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


## STEP3: Write configuration file
Below is a simple example configuration file `helloworld_cfg.yaml`. Other contents in this configuration file will be introduced in subsequent chapters. Here we focus on two points:
- `aimrt.log` node: specifies some details of the log.
- `HelloWorldPyModule` node: configuration for `HelloWorldPyModule`, which can be read within the module.


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


## STEP4: Launch and test

Copy the Python code execution file `helloworld_app.py` and the configuration file `helloworld_cfg.yaml` to the same directory, then execute the following command to run the script and observe the printed logs:

```shell
python3 ./helloworld_app.py --cfg_file_path=./helloworld_cfg.yaml
```
