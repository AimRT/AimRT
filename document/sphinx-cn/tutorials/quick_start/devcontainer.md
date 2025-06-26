# 使用开发容器快速开始

使用开发容器（Dev Container）是开始 AimRT 开发的最快速、最简单的方式。开发容器为您提供了一个完整配置好的开发环境，无需手动安装任何依赖项。

## 什么是开发容器

开发容器（Development Container，简称 Dev Container）是一个完整的开发环境，运行在 Docker 容器中。它包含了：

- 预配置的操作系统（Ubuntu 22.04 + ROS2 Humble）
- 所有必需的编译工具和依赖项
- VS Code 扩展和配置
- AimRT 开发所需的全部环境

## 系统要求

在使用开发容器之前，您需要在本地系统上安装：

### Windows 系统
- [Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/)
- [Visual Studio Code](https://code.visualstudio.com/)
- [Dev Containers 扩展](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### macOS 系统
- [Docker Desktop for Mac](https://docs.docker.com/desktop/install/mac-install/)
- [Visual Studio Code](https://code.visualstudio.com/)
- [Dev Containers 扩展](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### Linux 系统
- [Docker Engine](https://docs.docker.com/engine/install/) 或 [Docker Desktop for Linux](https://docs.docker.com/desktop/install/linux-install/)
- [Visual Studio Code](https://code.visualstudio.com/)
- [Dev Containers 扩展](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### 代理配置

如果您在公司网络环境或需要通过代理访问互联网，在构建开发容器时可能会遇到网络连接问题。此时需要配置 Docker 代理，请参考以下官方文档：
- [Docker 构建时的代理配置](https://docs.docker.com/engine/cli/proxy/#proxy-as-environment-variable-for-builds)
- [Docker 守护进程代理配置](https://docs.docker.com/engine/daemon/proxy/)


## 快速开始步骤

### 1. 克隆仓库

```bash
git clone https://github.com/AimRT/AimRT.git
cd AimRT
```

### 2. 在 VS Code 中打开项目

```bash
code .
```

### 3. 启动开发容器

当您在 VS Code 中打开项目后，会看到右下角弹出提示：

> **Folder contains a Dev Container configuration file. Reopen in container?**

点击 **"Reopen in Container"** 按钮。

**或者**，您可以手动启动：

1. 按 `Ctrl+Shift+P`（Windows/Linux）或 `Cmd+Shift+P`（macOS）打开命令面板
2. 输入 `Dev Containers: Reopen in Container`
3. 选择该命令并回车

### 4. 等待容器构建完成

首次启动时，Docker 会：
- 下载基础镜像（约 1-2 GB）
- 构建开发容器（约 5-10 分钟，取决于网络速度）
- 安装所有依赖项和开发工具

后续启动会更快，因为镜像已被缓存。

### 5. 开始开发

容器启动完成后，您将拥有一个完整的 AimRT 开发环境：

- **编译项目**：
  ```bash
  ./build.sh
  ```


## 开发容器特性

### 预安装的工具和环境

- **操作系统**：Ubuntu 22.04
- **ROS2**：Humble 版本，已配置环境变量
- **编译工具**：
  - CMake 3.28.0
  - GCC/Clang 编译器
  - Rust 工具链（用于 Zenoh 插件）
- **Python 环境**：
  - Python 3.10
  - 预安装必要的包（pyinstaller, jinja2, pyyaml 等）
- **开发依赖**：
  - 所有 AimRT 编译所需的系统库
  - protobuf、gRPC、Zenoh 等第三方库

### VS Code 扩展

容器自动安装以下扩展：
- `llvm-vs-code-extensions.vscode-clangd` - C++ 语言支持和代码跳转
- `ms-vscode.cmake-tools` - CMake 支持
- `ms-python.python` - Python 语言支持
- `eamodio.gitlens` - Git 增强
- `mjohns.clang-format` - 代码格式化
- `redhat.vscode-yaml` - YAML 支持

### 代码跳转和智能提示

开发容器预配置了完整的 C++ 代码跳转功能：

- **clangd 语言服务器**：安装了 clangd-18，提供精确的代码跳转、补全和错误检查
- **编译数据库**：CMake 自动生成 `compile_commands.json`，确保 clangd 理解项目结构
- **智能配置**：
  - 自动检测头文件路径
  - 启用后台索引以提高性能
  - 内置 clang-tidy 静态分析
  - 支持现代 C++20 特性

**使用方法**：
- **跳转到定义**：`Ctrl+Click` 或 `F12`
- **查找引用**：`Shift+F12`
- **重命名符号**：`F2`
- **查看类型信息**：悬停鼠标
- **代码补全**：输入时自动触发

### 端口转发

容器自动转发以下端口：
- `2222` - SSH 端口（用于远程访问）

## 常见问题解答

### Q: 首次启动很慢怎么办？
A: 首次启动需要下载和构建镜像，请耐心等待。确保网络连接稳定，如果在中国大陆，容器会自动检测并使用国内镜像源加速下载。

### Q: 如何在容器外访问编译结果？
A: 编译结果会保存在项目目录的 `build` 文件夹中，该文件夹在宿主机和容器间共享。

### Q: 如何重新构建容器？
A: 在命令面板中选择 `Dev Containers: Rebuild Container` 来重新构建容器。

### Q: 容器占用空间大吗？
A: 完整的开发容器约占用 3-4 GB 磁盘空间。

### Q: 可以在没有网络的环境下使用吗？
A: 一旦容器构建完成，可以在离线环境下正常使用进行开发。

### Q: 代码跳转不工作怎么办？
A: 如果遇到代码跳转问题，请尝试以下步骤：
1. 确保项目已编译：运行 `./build.sh` 生成编译信息
2. 生成编译数据库：运行 `compdb -p build/ list > compile_commands.json` 来创建或更新编译命令数据库
3. 重启 clangd：按 `Ctrl+Shift+P`，运行 `clangd: Restart language server`
4. 检查 clangd 状态：查看 VS Code 右下角是否显示 "clangd: Ready"
5. 如果仍有问题，可以查看输出面板中的 "clangd" 日志

## 下一步

成功启动开发容器后，建议您：

1. 阅读 [Hello World 示例](helloworld_cpp.md) 开始第一个 AimRT 程序
2. 了解 [AimRT 核心概念](../concepts/concepts.md)
3. 探索更多 [示例程序](../examples/examples_cpp.md)

