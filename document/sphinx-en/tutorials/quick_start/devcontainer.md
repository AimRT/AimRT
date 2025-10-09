# Quick Start with Dev Container

Using a development container (Dev Container) is the fastest and easiest way to get started with AimRT development. It provides a fully configured development environment without the need to manually install any dependencies.

## What is a Dev Container

A development container (Dev Container) is a complete development environment that runs inside a Docker container. It includes:

- A pre-configured operating system (Ubuntu 22.04 + ROS2 Humble)
- All required build tools and dependencies
- VS Code extensions and configurations
- The complete environment needed for AimRT development

## System Requirements

Before using the development container, you need to install the following on your local system:

### Windows
- [Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/)
- [Visual Studio Code](https://code.visualstudio.com/)
- [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### macOS
- [Docker Desktop for Mac](https://docs.docker.com/desktop/install/mac-install/)
- [Visual Studio Code](https://code.visualstudio.com/)
- [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### Linux
- [Docker Engine](https://docs.docker.com/engine/install/) or [Docker Desktop for Linux](https://docs.docker.com/desktop/install/linux-install/)
- [Visual Studio Code](https://code.visualstudio.com/)
- [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### Proxy Configuration

If you are in a corporate network environment or need to access the internet through a proxy, you may encounter network connectivity issues when building the development container. In this case, you need to configure Docker proxy settings. Please refer to the following official documentation:
- [Docker build-time proxy configuration](https://docs.docker.com/engine/cli/proxy/#proxy-as-environment-variable-for-builds)
- [Docker daemon proxy configuration](https://docs.docker.com/engine/daemon/proxy/)

## Quick Start Steps

### 1. Clone the Repository


```bash
git clone https://github.com/AimRT/AimRT.git
cd AimRT
```


### 2. Open the Project in VS Code


```bash
code .
```


### 3. Start the Development Container

When you open the project in VS Code, you will see a popup in the bottom right corner:

> **Folder contains a Dev Container configuration file. Reopen in container?**

Click the **"Reopen in Container"** button.

**Alternatively**, you can start it manually:

1. Press `Ctrl+Shift+P` (Windows/Linux) or `Cmd+Shift+P` (macOS) to open the command palette
2. Type `Dev Containers: Reopen in Container`
3. Select the command and press Enter

### 4. Wait for Container Build to Complete

On first launch, Docker will:
- Download the base image (approximately 1-2 GB)
- Build the development container (about 5-10 minutes, depending on network speed)
- Install all dependencies and development tools

Subsequent launches will be faster as the image is cached.

### 5. Start Development

Once the container starts, you will have a complete AimRT development environment:

- **Build the project**:
  
```bash
  ./build.sh
  ```


## Development Container Features

### Pre-installed Tools and Environment

- **Operating System**: Ubuntu 22.04
- **ROS2**: Humble version with environment variables configured
- **Build Tools**:
  - CMake 3.28.0
  - GCC/Clang compilers
  - Rust toolchain (for Zenoh plugin)
- **Python Environment**:
  - Python 3.10
  - Pre-installed necessary packages (pyinstaller, jinja2, pyyaml, etc.)
- **Development Dependencies**:
  - All system libraries required for AimRT compilation

### VS Code Extensions

The container automatically installs the following extensions:
- `llvm-vs-code-extensions.vscode-clangd` - C++ language support and code navigation
- `ms-vscode.cmake-tools` - CMake support
- `ms-python.python` - Python language support
- `eamodio.gitlens` - Git enhancements
- `mjohns.clang-format` - Code formatting
- `redhat.vscode-yaml` - YAML support

### Code Navigation and IntelliSense

The development container comes with complete C++ code navigation functionality:

- **clangd language server**: clangd-18 is installed, providing precise code navigation, completion, and error checking
- **Compilation database**: CMake automatically generates `compile_commands.json` to ensure clangd understands the project structure
- **Smart configuration**:
  - Automatic header file path detection
  - Background indexing enabled for better performance
  - Built-in clang-tidy static analysis
  - Support for modern C++20 features

**Usage**:
- **Go to definition**: `Ctrl+Click` or `F12`
- **Find references**: `Shift+F12`
- **Rename symbol**: `F2`
- **View type information**: Hover mouse
- **Code completion**: Automatically triggered while typing

### Port Forwarding

The container automatically forwards the following ports:
- `2222` - SSH port (for remote access)

## FAQ

### Q: What if the first startup is very slow?
A: The first startup requires downloading and building the image, please be patient. Ensure a stable network connection. If in mainland China, the container will automatically detect and use domestic mirror sources to accelerate downloads.

### Q: How to access build results outside the container?
A: Build results are saved in the `build` folder in the project directory, which is shared between the host and container.

### Q: How to rebuild the container?
A: Select `Dev Containers: Rebuild Container` in the command palette to rebuild the container.

### Q: Does the container take up a lot of space?
A: The complete development container takes up about 3-4 GB of disk space.

### Q: What if code navigation doesn't work?
A: If you encounter code navigation issues, try these steps:
1. Ensure the project is built: run `./build.sh` to generate compilation information
2. Generate compilation database: run `compdb -p build/ list > compile_commands.json` to create or update the compilation command database
3. Restart clangd: press `Ctrl+Shift+P`, run `clangd: Restart language server`
4. Check clangd status: see if "clangd: Ready" is displayed in the bottom right corner of VS Code
5. If problems persist, check the "clangd" logs in the output panel

## Next Steps

After successfully starting the development container, we recommend:

1. Read the [Hello World example](helloworld_cpp.md) to start your first AimRT program
2. Learn about [AimRT core concepts](../concepts/concepts.md)
3. Explore more [example programs](../examples/examples_cpp.md)