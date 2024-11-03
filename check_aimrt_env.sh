#!/bin/bash

# Script Name: check_aimrt_env.sh
# Purpose: Check the system environment to determine if it meets the build and run requirements of AimRT,
#          generate appropriate cmake build commands, and include test steps.

# Define colors
GREEN='\033[0;32m'   # Green
YELLOW='\033[1;33m'  # Yellow
RED='\033[0;31m'     # Red
NC='\033[0m'         # No Color

echo -e "${GREEN}----------------------------------------${NC}"
echo -e "${GREEN}   AimRT Environment Check and Test Script${NC}"
echo -e "${GREEN}----------------------------------------${NC}"

# Define variables to store check results
declare -A installed_components
declare -A missing_components

# Check operating system version
echo -e "\n${GREEN}**Operating System Information:**${NC}"
os_info=$(lsb_release -a 2>/dev/null || cat /etc/os-release)
echo "$os_info"

# Check CMake version
echo -e "\n${GREEN}**CMake Version Check:**${NC}"
if command -v cmake &>/dev/null; then
    cmake_version=$(cmake --version | head -n1 | awk '{print $3}')
    required_cmake_version=3.24
    if [[ $(printf '%s\n' "$required_cmake_version" "$cmake_version" | sort -V | head -n1) == "$required_cmake_version" ]]; then
        echo -e "${GREEN}Detected CMake version: $cmake_version${NC}"
        echo -e "${GREEN}CMake version meets the requirement (>= $required_cmake_version)${NC}"
        installed_components["CMake"]="Version $cmake_version"
    else
        echo -e "${YELLOW}Detected CMake version: $cmake_version${NC}"
        echo -e "${RED}CMake version is lower than required (>= $required_cmake_version), please update CMake.${NC}"
        missing_components["CMake"]="Version $cmake_version (requires >= $required_cmake_version)"
    fi
else
    echo -e "${RED}CMake is not installed.${NC}"
    missing_components["CMake"]="Not installed"
fi

# Check gcc and g++ versions
echo -e "\n${GREEN}**GCC and G++ Version Check:**${NC}"
if command -v gcc &>/dev/null && command -v g++ &>/dev/null; then
    gcc_version=$(gcc --version | head -n1 | awk '{print $3}')
    gpp_version=$(g++ --version | head -n1 | awk '{print $3}')
    echo -e "${GREEN}Detected GCC version: $gcc_version${NC}"
    echo -e "${GREEN}Detected G++ version: $gpp_version${NC}"
    installed_components["GCC/G++"]="GCC version $gcc_version, G++ version $gpp_version"
else
    echo -e "${RED}GCC or G++ is not installed.${NC}"
    missing_components["GCC/G++"]="Not installed"
fi

# Check Make version
echo -e "\n${GREEN}**Make Version Check:**${NC}"
if command -v make &>/dev/null; then
    make_version=$(make --version | head -n1 | awk '{print $3}')
    echo -e "${GREEN}Detected Make version: $make_version${NC}"
    installed_components["Make"]="Version $make_version"
else
    echo -e "${RED}Make is not installed.${NC}"
    missing_components["Make"]="Not installed"
fi

# Check Python 3 version
echo -e "\n${GREEN}**Python 3 Version Check:**${NC}"
if command -v python3 &>/dev/null; then
    python_version=$(python3 --version 2>&1 | awk '{print $2}')
    required_python_version=3.10
    if [[ $(printf '%s\n' "$required_python_version" "$python_version" | sort -V | head -n1) == "$required_python_version" ]]; then
        echo -e "${GREEN}Detected Python 3 version: $python_version${NC}"
        echo -e "${GREEN}Python 3 version meets the requirement (>= $required_python_version)${NC}"
        installed_components["Python 3"]="Version $python_version"
    else
        echo -e "${YELLOW}Detected Python 3 version: $python_version${NC}"
        echo -e "${RED}Python 3 version is lower than required (>= $required_python_version), please update Python.${NC}"
        missing_components["Python 3"]="Version $python_version (requires >= $required_python_version)"
    fi
else
    echo -e "${RED}Python 3 is not installed.${NC}"
    missing_components["Python 3"]="Not installed"
fi

# Check if pip3 is installed
echo -e "\n${GREEN}**pip3 Check:**${NC}"
if command -v pip3 &>/dev/null; then
    echo -e "${GREEN}pip3 is installed.${NC}"
    installed_components["pip3"]="Installed"
else
    echo -e "${RED}pip3 is not installed.${NC}"
    missing_components["pip3"]="Not installed"
fi

# Check Python packages
echo -e "\n${GREEN}**Python Package Check:**${NC}"
python_packages=("pyinstaller" "jinja2" "pyyaml" "build" "setuptools" "wheel")
for pkg in "${python_packages[@]}"; do
    if pip3 show "$pkg" &>/dev/null; then
        version=$(pip3 show "$pkg" | grep Version | awk '{print $2}')
        echo -e "${GREEN}$pkg is installed, version: $version${NC}"
        installed_components["$pkg"]="Version $version"
    else
        echo -e "${YELLOW}$pkg is not installed.${NC}"
        missing_components["$pkg"]="Not installed"
    fi
done

# Check if ROS2 Humble version is installed
echo -e "\n${GREEN}**ROS2 Humble Version Check:**${NC}"
if command -v ros2 &>/dev/null; then
    source /opt/ros/humble/setup.bash &>/dev/null
    if [[ "$ROS_DISTRO" == "humble" ]]; then
        echo -e "${GREEN}Detected ROS2 version: $ROS_DISTRO${NC}"
        installed_components["ROS2"]="Version $ROS_DISTRO"
    else
        echo -e "${YELLOW}Detected ROS2 version: $ROS_DISTRO${NC}"
        echo -e "${RED}ROS2 Humble version not detected, please install the corresponding version.${NC}"
        missing_components["ROS2"]="Humble version not installed"
    fi
else
    echo -e "${RED}ROS2 is not installed.${NC}"
    missing_components["ROS2"]="Not installed"
fi

# Check if libacl1-dev is installed (Iceoryx dependency)
echo -e "\n${GREEN}**libacl1-dev Check (Iceoryx dependency):**${NC}"
if dpkg -l | grep libacl1-dev &>/dev/null; then
    echo -e "${GREEN}libacl1-dev is installed.${NC}"
    installed_components["libacl1-dev"]="Installed"
else
    echo -e "${YELLOW}libacl1-dev is not installed.${NC}"
    missing_components["libacl1-dev"]="Not installed"
fi

# Check Rust environment (Zenoh dependency)
echo -e "\n${GREEN}**Rust Environment Check (Zenoh dependency):**${NC}"
if command -v rustc &>/dev/null; then
    rust_version=$(rustc --version | awk '{print $2}')
    echo -e "${GREEN}Detected Rust version: $rust_version${NC}"
    installed_components["Rust"]="Version $rust_version"
else
    echo -e "${YELLOW}Rust is not installed.${NC}"
    missing_components["Rust"]="Not installed"
fi

# Check if libssl-dev is installed (MQTT dependency)
echo -e "\n${GREEN}**libssl-dev Check (MQTT dependency):**${NC}"
if dpkg -l | grep libssl-dev &>/dev/null; then
    echo -e "${GREEN}libssl-dev is installed.${NC}"
    installed_components["libssl-dev"]="Installed"
else
    echo -e "${YELLOW}libssl-dev is not installed.${NC}"
    missing_components["libssl-dev"]="Not installed"
fi

# Output configurations that meet the requirements
echo -e "\n${GREEN}----------------------------------------${NC}"
echo -e "${GREEN}Configurations that meet the requirements:${NC}"
for key in "${!installed_components[@]}"; do
    echo -e "${GREEN}- $key: ${installed_components[$key]}${NC}"
done

# Output configurations that do not meet the requirements or are missing
if [ ${#missing_components[@]} -gt 0 ]; then
    echo -e "\n${YELLOW}Configurations that do not meet the requirements or are missing:${NC}"
    for key in "${!missing_components[@]}"; do
        echo -e "${YELLOW}- $key: ${missing_components[$key]}${NC}"
    done
else
    echo -e "\n${GREEN}All configurations meet the requirements.${NC}"
fi

# Generate cmake build options based on the check results
echo -e "\n${GREEN}----------------------------------------${NC}"
echo -e "${GREEN}Generating appropriate cmake build commands:${NC}"

# Default cmake build options (including test options)
cmake_options="-DCMAKE_BUILD_TYPE=Release \\
-DAIMRT_INSTALL=ON \\
-DCMAKE_INSTALL_PREFIX=./build/install \\
-DAIMRT_BUILD_TESTS=ON \\
-DAIMRT_BUILD_EXAMPLES=ON \\
-DAIMRT_BUILD_DOCUMENT=ON \\
-DAIMRT_BUILD_RUNTIME=ON \\
-DAIMRT_BUILD_CLI_TOOLS=OFF \\
-DAIMRT_BUILD_PYTHON_RUNTIME=OFF \\
-DAIMRT_USE_FMT_LIB=ON \\
-DAIMRT_BUILD_WITH_PROTOBUF=ON \\
-DAIMRT_USE_LOCAL_PROTOC_COMPILER=OFF \\
-DAIMRT_USE_PROTOC_PYTHON_PLUGIN=OFF \\
-DAIMRT_BUILD_WITH_ROS2=OFF \\
-DAIMRT_BUILD_NET_PLUGIN=ON \\
-DAIMRT_BUILD_MQTT_PLUGIN=OFF \\
-DAIMRT_BUILD_ZENOH_PLUGIN=OFF \\
-DAIMRT_BUILD_ICEORYX_PLUGIN=OFF \\
-DAIMRT_BUILD_ROS2_PLUGIN=OFF \\
-DAIMRT_BUILD_RECORD_PLAYBACK_PLUGIN=ON \\
-DAIMRT_BUILD_TIME_MANIPULATOR_PLUGIN=ON \\
-DAIMRT_BUILD_PARAMETER_PLUGIN=ON \\
-DAIMRT_BUILD_LOG_CONTROL_PLUGIN=ON \\
-DAIMRT_BUILD_OPENTELEMETRY_PLUGIN=ON \\
-DAIMRT_BUILD_GRPC_PLUGIN=ON \\
-DAIMRT_BUILD_ECHO_PLUGIN=ON \\
-DAIMRT_BUILD_PYTHON_PACKAGE=OFF"

# Adjust build options based on check results
# If Python 3 version meets requirements and necessary Python packages are installed, enable Python features
if [[ -n "${installed_components["Python 3"]}" && -n "${installed_components["pip3"]}" ]]; then
    if [[ -n "${installed_components["pyinstaller"]}" && -n "${installed_components["jinja2"]}" && -n "${installed_components["pyyaml"]}" ]]; then
        cmake_options=$(echo "$cmake_options" | sed 's/-DAIMRT_BUILD_CLI_TOOLS=OFF/-DAIMRT_BUILD_CLI_TOOLS=ON/')
    fi
    if [[ -n "${installed_components["build"]}" && -n "${installed_components["setuptools"]}" && -n "${installed_components["wheel"]}" ]]; then
        cmake_options=$(echo "$cmake_options" | sed 's/-DAIMRT_BUILD_PYTHON_PACKAGE=OFF/-DAIMRT_BUILD_PYTHON_PACKAGE=ON/')
    fi
    cmake_options=$(echo "$cmake_options" | sed 's/-DAIMRT_BUILD_PYTHON_RUNTIME=OFF/-DAIMRT_BUILD_PYTHON_RUNTIME=ON/')
fi

# If ROS2 Humble is installed, enable ROS2 related features
if [[ -n "${installed_components["ROS2"]}" ]]; then
    cmake_options=$(echo "$cmake_options" | sed 's/-DAIMRT_BUILD_WITH_ROS2=OFF/-DAIMRT_BUILD_WITH_ROS2=ON/')
    cmake_options=$(echo "$cmake_options" | sed 's/-DAIMRT_BUILD_ROS2_PLUGIN=OFF/-DAIMRT_BUILD_ROS2_PLUGIN=ON/')
fi

# If libacl1-dev is installed, enable Iceoryx plugin
if [[ -n "${installed_components["libacl1-dev"]}" ]]; then
    cmake_options=$(echo "$cmake_options" | sed 's/-DAIMRT_BUILD_ICEORYX_PLUGIN=OFF/-DAIMRT_BUILD_ICEORYX_PLUGIN=ON/')
fi

# If Rust is installed, enable Zenoh plugin
if [[ -n "${installed_components["Rust"]}" ]]; then
    cmake_options=$(echo "$cmake_options" | sed 's/-DAIMRT_BUILD_ZENOH_PLUGIN=OFF/-DAIMRT_BUILD_ZENOH_PLUGIN=ON/')
fi

# If libssl-dev is installed, enable MQTT plugin
if [[ -n "${installed_components["libssl-dev"]}" ]]; then
    cmake_options=$(echo "$cmake_options" | sed 's/-DAIMRT_BUILD_MQTT_PLUGIN=OFF/-DAIMRT_BUILD_MQTT_PLUGIN=ON/')
fi

# Output the generated cmake build commands
echo -e "\n${GREEN}Please use the following commands to build and test:${NC}"
echo -e "\ncmake -B build \\"
echo -e "$cmake_options"
echo -e "\ncmake --build build --config Release --parallel \$(nproc)"
echo -e "\ncmake --build build --config Release --target test"

echo -e "\n${GREEN}----------------------------------------${NC}"
echo -e "${GREEN}                Check Completed${NC}"
echo -e "${GREEN}----------------------------------------${NC}"

