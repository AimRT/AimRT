# 转换 AimRT 的 bag 文件为 ROS2 的 bag 文件

## 简介

**aimrt_cli** 工具可以将 AimRT **recordplayback** 插件记录的 bag 文件转换为 ROS2 的 bag 文件，其中ros2 消息会直接转换，pb 消息则不会转换。

基本使用样例如下：
```
aimrt_cli trans -s [your_aimrtbag_source_dir] -o [your_output_dir]
```

您也可以使用 `aimrt_cli -h/--help`查看支持的命令行选项。


## 使用说明

**aimrt_cli** 工具的使用方法如下：

`trans` 命令的使用方法如下：

```bash
aimrt_cli trans -h, --help 会显示参数说明:

options:
  -h, --help            show this help message and exit
  -s SRC_DIR [SRC_DIR ...], --src_dir SRC_DIR [SRC_DIR ...]
                        aimrtbag source directories (support multiple directories)
  -o OUTPUT_DIR, --output_dir OUTPUT_DIR
                        directory you want to output your files
```

其中 `-s` 参数为必填参数，表示 aimrtbag 的源目录，支持多个目录，`-o` 参数为必填参数，表示转换后的bag的输出目录，如果输出目录不存在，则会自动创建；如果输出目录存在，则会覆盖。
