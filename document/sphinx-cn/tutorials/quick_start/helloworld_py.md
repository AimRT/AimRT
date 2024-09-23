
# HelloWorld Python

本章将以一个简单的 Demo 来介绍如何建立一个最基本的 AimRT Python 工程。


AimRT 基于 pybind11，在 CPP 接口层之上包装了一层 python 接口。本 Demo 将演示以下几项基本功能：
- 基于 pip 安装 AimRT；
- 基于 App 模式，直接在 Main 方法中创建 AimRT 实例并使用其中的功能；
- 使用基础的日志功能；
- 使用基础的配置功能；
- 运行 Python 脚本以体验 AimRT 的功能。


## STEP1：确保本地环境符合要求

请先确保本地的 python 环境满足要求，并且已经安装有`aimrt_py`包。具体请参考[引用与安装（Python）](installation_py.md)中的步骤。

注意，示例本身是跨平台的，但本文档基于 linux 进行演示。

## STEP2: 编写业务代码

参考以下代码，编写一个 python 文件`helloworld_app.py`，在其中创建了一个 aimrt 实例，并解析传入的配置文件、打印一些简单的日志。

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


## STEP3: 编写配置文件
以下是一个简单的示例配置文件`helloworld_cfg.yaml`。这个配置文件中的其他内容将在后续章节中介绍，这里关注两个地方：
- `aimrt.log`节点：此处指定了日志的一些细节。
- `HelloWorldPyModule`节点：此处为`HelloWorldPyModule`的配置，可以在模块中读取到。


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

## STEP4: 启动并测试

将 python 代码执行文件`helloworld_app.py`和配置文件`helloworld_cfg.yaml`拷贝到一个目录下，然后执行以下命令运行脚本，观察打印出来的日志：
```shell
python3 ./helloworld_app.py --cfg_file_path=./helloworld_cfg.yaml
```

