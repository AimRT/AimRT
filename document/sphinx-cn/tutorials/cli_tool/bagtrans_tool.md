# bagtrans 工具

## 简介

**bagtrans** 是一个 AimRT 官方提供的命令行工具，目前可以将 AimRT **recordplayback** 插件记录的 bag 文件转换为 ROS2 的 bag 文件，其中ros2 原生消息会直接转换，pb 消息会转换为 aimrt 提供的 ros2_plugin_proto 中的 RosMsgWrapper 消息类型。

## 安装

**bagtrans** 工具是一个基于 Python 开发的小工具，并且依赖于 aimrt 提供的 ros2_plugin_protobuf 消息类型，需要按照以下步骤安装：

1. 需要您的 python 环境中安装了 build、wheel 和 setuptools 包，如果未安装，可以使用以下命令安装：

```bash
pip3 install build wheel setuptools --upgrade
```

2. 需要编译生成 aimrt 源码库中的 ros2_plugin_proto 消息类型，并且在 build.sh 文件中确保`AIMRT_BUILD_ROS2_PLUGIN` 、 `AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN` 和 `AIMRT_BUILD_BAGTRANS` 选项为`ON`，执行您 aimrt 源码库中的`build.sh`文件进行编译，编译完成后，在  `<path_to_your_aimrt_src_code>/src/tools/bagtrans/build/dist` 文件夹下可找到编译生成的`whl`文件。

3. 在终端中执行以下命令:
```
cd <path_to_your_aimrt_src_code>/src/tools/bagtrans/build/dist
pip3 install bagtrans-<version>-py3-none-any.whl
```
**bagtrans**工具将会自动安装到您的 python 环境中。使用 `pip list | grep bagtrans`可查看是否安装成功。可使用 `pip uninstall bagtrans`进行卸载。


## 使用说明

**bagtrans** 工具的使用方法如下：

每次使用前，需要使用命令 `source <path_to_your_aimrt_src_code>/src/tools/bagtrans/build/share/ros2_plugin_proto/local_setup.bash` 以修改环境变量。

transbag 命令的使用方法如下：

```bash
bagtrans trans -h, --help 会显示参数说明:

options:
  -h, --help            show this help message and exit
  -s SRC_DIR, --src_dir SRC_DIR
                        aimrtbag source directory.
  -o OUTPUT_DIR, --output_dir OUTPUT_DIR
                        directory you want to output your files.
```

其中 `-s` 参数为必填参数，表示 aimrtbag 的源目录，`-o` 参数为必填参数，表示转换后的bag的输出目录，如果输出目录不存在，则会自动创建；如果输出目录存在，则会覆盖。


