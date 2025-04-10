
# CLI 工具


## 简介

**aimrt_cli**是一个 AimRT 官方提供的命令行工具，目前支持以下功能：

- [为新项目生成脚手架代码](./gen_prj.md)

您也可以直接执行`aimrt_cli --help`获取内置帮助说明。更多功能敬请期待。


## 安装
**aimrt_cli**工具是一个基于 Python 开发的小工具，有以下三种安装方式，请选择任意一种您喜欢的进行安装。


### 源码安装到 python 环境中
**aimrt_cli**提供了通过`setuptools`工具直接打包到 python 环境中的功能，您可下载 AimRT 源码后，在终端中执行以下命令:
```
cd <path_to_your_aimrt_src_code>/src/tools/aimrt_cli
python -m build --wheel
pip install dist/aimrt_cli-*.whl
```
以上流程会为您安装一个 aimrt_cli 可执行程序到您的 python 环境中，一般位于 `~/.local/bin/` 目录下（如果使用虚拟环境则有所不同），该目录需要添加到您的 `PATH` 环境变量中，使用 `aimrt_cli` 命令即可执行。

可使用 `pip uninstall aimrt_cli`进行卸载。


### 源码编译出可执行文件
可直接通过编译 **aimrt** 生成 `aimrt_cli` 的可执行文件，并将其加入到系统的环境变量中，即可在终端中执行 `aimrt_cli` 命令。整体流程为:
- 执行您 aimrt 源码库中的`build.sh`文件进行编译。可以在`build.sh`文件中关闭其他您不需要的 CMake 选项以加快编译。
- 在 build 文件夹下可找到编译出的**aimrt_cli**可执行文件。
- 将其添加到环境变量中。

参考以下命令：
```
cd <path_to_your_aimrt_src_code>
./build.sh

mv <path_to_your_aimrt_src_code>/build/install/bin/aimrt_cli <path_to_your_aimrt_cli>

export PATH=<path_to_your_aimrt_cli>:$PATH
```

### 直接pip安装

***TODO***

### 从Release中下载可执行文件

***TODO***

