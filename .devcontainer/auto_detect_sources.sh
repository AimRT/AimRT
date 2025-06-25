#!/bin/bash

# Docker Auto Detect Best Mirror Sources Script
# This script tests multiple mirror sources and automatically selects the fastest one for Docker build

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${GREEN}[DOCKER-BUILD]${NC} $1"
}

print_testing() {
    echo -e "${BLUE}[TESTING]${NC} $1"
}

print_result() {
    echo -e "${CYAN}[RESULT]${NC} $1"
}

# Define mirror sources (including official sources for comparison)
declare -A APT_MIRRORS=(
    ["official"]="archive.ubuntu.com"
    ["security"]="security.ubuntu.com"
    ["aliyun"]="mirrors.aliyun.com"
    ["tuna"]="mirrors.tuna.tsinghua.edu.cn"
    ["ustc"]="mirrors.ustc.edu.cn"
    ["huawei"]="mirrors.huaweicloud.com"
)

# ARM64-friendly APT mirrors (including official ARM ports)
declare -A APT_MIRRORS_ARM64=(
    ["ports"]="ports.ubuntu.com"
    ["aliyun"]="mirrors.aliyun.com"
    ["tuna"]="mirrors.tuna.tsinghua.edu.cn"
    ["ustc"]="mirrors.ustc.edu.cn"
    ["tencent"]="mirrors.cloud.tencent.com"
)

declare -A PIP_MIRRORS=(
    ["official"]="pypi.org"
    ["aliyun"]="mirrors.aliyun.com"
    ["tuna"]="pypi.tuna.tsinghua.edu.cn"
    ["ustc"]="pypi.mirrors.ustc.edu.cn"
    ["douban"]="pypi.douban.com"
)

# Global variables for best mirrors
BEST_APT_MIRROR=""
BEST_APT_NAME=""
BEST_PIP_MIRROR=""
BEST_PIP_NAME=""
BEST_ROS2_MIRROR=""
BEST_ROS2_NAME=""

# Function to test mirror latency (simplified for Docker build)
test_mirror_latency() {
    local url="$1"
    local timeout="${2:-3}"

    # Check if curl is available
    if ! command -v curl > /dev/null 2>&1; then
        print_info "curl not available, using fallback method"
        # Return a default latency to indicate testing failed but not fatal
        echo "999000"
        return
    fi

    # Check if awk is available
    if ! command -v awk > /dev/null 2>&1; then
        print_info "awk not available, using fallback method"
        echo "999000"
        return
    fi

    # Test with curl and measure time
    local time_output
    time_output=$(curl -o /dev/null -s -w "%{time_total}\n" --connect-timeout "$timeout" --max-time "$timeout" "http://$url" 2>/dev/null || echo "999.999")

    # Convert to milliseconds and return as integer
    echo "$time_output" | awk '{printf "%.0f", $1 * 1000}'
}

# Function to test APT mirrors quickly
test_apt_mirrors() {
    print_info "Quick testing APT mirrors for Docker build..."

    local best_latency=999999
    local ubuntu_version
    ubuntu_version=$(lsb_release -cs)
    local arch
    arch=$(dpkg --print-architecture)

    print_info "Testing APT mirrors for architecture: $arch"

    # Choose appropriate mirror set based on architecture
    local -n mirrors_ref
    if [ "$arch" = "arm64" ] || [ "$arch" = "armhf" ]; then
        mirrors_ref=APT_MIRRORS_ARM64
        print_info "Using ARM64-optimized mirror list"
    else
        mirrors_ref=APT_MIRRORS
        print_info "Using standard mirror list"
    fi

    for name in "${!mirrors_ref[@]}"; do
        local mirror="${mirrors_ref[$name]}"
        local test_url

        # Different path structures for different mirrors
        if [ "$name" = "ports" ]; then
            test_url="$mirror/ubuntu-ports/dists/$ubuntu_version/Release"
        elif [ "$name" = "security" ]; then
            test_url="$mirror/ubuntu/dists/$ubuntu_version-security/Release"
        else
            test_url="$mirror/ubuntu/dists/$ubuntu_version/Release"
        fi

        print_testing "Testing APT mirror: $name"

        local latency
        latency=$(test_mirror_latency "$test_url" 5)

        if [ "$latency" -lt 999000 ]; then
            print_result "APT $name: ${latency}ms"
            if [ "$latency" -lt "$best_latency" ]; then
                best_latency="$latency"
                BEST_APT_MIRROR="$mirror"
                BEST_APT_NAME="$name"
            fi
        else
            print_result "APT $name: Failed"
        fi
    done

    if [ -n "$BEST_APT_MIRROR" ]; then
        print_info "Selected APT mirror: $BEST_APT_NAME ($BEST_APT_MIRROR) - ${best_latency}ms"
    else
        print_info "No fast APT mirror found, using default"
        if [ "$arch" = "arm64" ] || [ "$arch" = "armhf" ]; then
            BEST_APT_MIRROR="ports.ubuntu.com/ubuntu-ports"
        else
            BEST_APT_MIRROR="archive.ubuntu.com"
        fi
        BEST_APT_NAME="default"
    fi
}

# Function to test PIP mirrors quickly
test_pip_mirrors() {
    print_info "Quick testing PIP mirrors for Docker build..."

    local best_latency=999999

    for name in "${!PIP_MIRRORS[@]}"; do
        local mirror="${PIP_MIRRORS[$name]}"
        local test_url

        # Different path structures for different PIP mirrors
        if [ "$name" = "official" ]; then
            test_url="$mirror/simple/"
        elif [ "$name" = "douban" ]; then
            test_url="$mirror/simple/"
        else
            test_url="$mirror/pypi/simple/"
        fi

        print_testing "Testing PIP mirror: $name"

        local latency
        latency=$(test_mirror_latency "$test_url" 5)

        if [ "$latency" -lt 999000 ]; then
            print_result "PIP $name: ${latency}ms"
            if [ "$latency" -lt "$best_latency" ]; then
                best_latency="$latency"
                BEST_PIP_MIRROR="$mirror"
                BEST_PIP_NAME="$name"
            fi
        else
            print_result "PIP $name: Failed"
        fi
    done

    if [ -n "$BEST_PIP_MIRROR" ]; then
        print_info "Selected PIP mirror: $BEST_PIP_NAME ($BEST_PIP_MIRROR) - ${best_latency}ms"
    else
        print_info "No fast PIP mirror found, using default"
        BEST_PIP_MIRROR="pypi.org"
        BEST_PIP_NAME="default"
    fi
}

# Function to configure APT sources
configure_apt_sources() {
    print_info "Configuring APT sources with $BEST_APT_NAME mirror..."

    local ubuntu_version
    ubuntu_version=$(lsb_release -cs)

    # Detect architecture
    local arch
    arch=$(dpkg --print-architecture)
    print_info "Detected architecture: $arch"

    # Backup original sources.list
    if [ -f /etc/apt/sources.list ]; then
        cp /etc/apt/sources.list /etc/apt/sources.list.backup
    fi

    # Create new sources.list with architecture support
    if [ "$BEST_APT_NAME" != "default" ]; then
        # Check if mirror supports current architecture
        local mirror_supports_arch=true

        # Some mirrors may not support ARM64, fallback to default for unsupported architectures
        if [ "$arch" = "arm64" ]; then
            case "$BEST_APT_NAME" in
                "163"|"huawei")
                    print_info "Mirror $BEST_APT_NAME may not fully support ARM64, using alternative"
                    mirror_supports_arch=false
                    ;;
            esac
        fi

        if [ "$mirror_supports_arch" = "true" ]; then
            # Configure sources based on mirror type
            if [ "$BEST_APT_NAME" = "ports" ]; then
                cat > /etc/apt/sources.list <<EOF
# Auto-selected fastest mirror: $BEST_APT_NAME ($BEST_APT_MIRROR)
# Architecture: $arch
deb http://$BEST_APT_MIRROR/ubuntu-ports/ $ubuntu_version main restricted universe multiverse
deb-src http://$BEST_APT_MIRROR/ubuntu-ports/ $ubuntu_version main restricted universe multiverse

deb http://$BEST_APT_MIRROR/ubuntu-ports/ $ubuntu_version-security main restricted universe multiverse
deb-src http://$BEST_APT_MIRROR/ubuntu-ports/ $ubuntu_version-security main restricted universe multiverse

deb http://$BEST_APT_MIRROR/ubuntu-ports/ $ubuntu_version-updates main restricted universe multiverse
deb-src http://$BEST_APT_MIRROR/ubuntu-ports/ $ubuntu_version-updates main restricted universe multiverse

deb http://$BEST_APT_MIRROR/ubuntu-ports/ $ubuntu_version-backports main restricted universe multiverse
deb-src http://$BEST_APT_MIRROR/ubuntu-ports/ $ubuntu_version-backports main restricted universe multiverse
EOF
            elif [ "$BEST_APT_NAME" = "security" ]; then
                # Use security mirror for security updates, official for others
                cat > /etc/apt/sources.list <<EOF
# Auto-selected fastest mirror: $BEST_APT_NAME ($BEST_APT_MIRROR) + official
# Architecture: $arch
deb http://archive.ubuntu.com/ubuntu/ $ubuntu_version main restricted universe multiverse
deb-src http://archive.ubuntu.com/ubuntu/ $ubuntu_version main restricted universe multiverse

deb http://$BEST_APT_MIRROR/ubuntu/ $ubuntu_version-security main restricted universe multiverse
deb-src http://$BEST_APT_MIRROR/ubuntu/ $ubuntu_version-security main restricted universe multiverse

deb http://archive.ubuntu.com/ubuntu/ $ubuntu_version-updates main restricted universe multiverse
deb-src http://archive.ubuntu.com/ubuntu/ $ubuntu_version-updates main restricted universe multiverse

deb http://archive.ubuntu.com/ubuntu/ $ubuntu_version-backports main restricted universe multiverse
deb-src http://archive.ubuntu.com/ubuntu/ $ubuntu_version-backports main restricted universe multiverse
EOF
            else
                cat > /etc/apt/sources.list <<EOF
# Auto-selected fastest mirror: $BEST_APT_NAME ($BEST_APT_MIRROR)
# Architecture: $arch
deb http://$BEST_APT_MIRROR/ubuntu/ $ubuntu_version main restricted universe multiverse
deb-src http://$BEST_APT_MIRROR/ubuntu/ $ubuntu_version main restricted universe multiverse

deb http://$BEST_APT_MIRROR/ubuntu/ $ubuntu_version-security main restricted universe multiverse
deb-src http://$BEST_APT_MIRROR/ubuntu/ $ubuntu_version-security main restricted universe multiverse

deb http://$BEST_APT_MIRROR/ubuntu/ $ubuntu_version-updates main restricted universe multiverse
deb-src http://$BEST_APT_MIRROR/ubuntu/ $ubuntu_version-updates main restricted universe multiverse

deb http://$BEST_APT_MIRROR/ubuntu/ $ubuntu_version-backports main restricted universe multiverse
deb-src http://$BEST_APT_MIRROR/ubuntu/ $ubuntu_version-backports main restricted universe multiverse
EOF
            fi
        else
            print_info "Using default APT sources for better $arch compatibility"
            cat > /etc/apt/sources.list <<EOF
# Default Ubuntu sources for $arch compatibility
deb http://archive.ubuntu.com/ubuntu/ $ubuntu_version main restricted universe multiverse
deb-src http://archive.ubuntu.com/ubuntu/ $ubuntu_version main restricted universe multiverse

deb http://security.ubuntu.com/ubuntu/ $ubuntu_version-security main restricted universe multiverse
deb-src http://security.ubuntu.com/ubuntu/ $ubuntu_version-security main restricted universe multiverse

deb http://archive.ubuntu.com/ubuntu/ $ubuntu_version-updates main restricted universe multiverse
deb-src http://archive.ubuntu.com/ubuntu/ $ubuntu_version-updates main restricted universe multiverse

deb http://archive.ubuntu.com/ubuntu/ $ubuntu_version-backports main restricted universe multiverse
deb-src http://archive.ubuntu.com/ubuntu/ $ubuntu_version-backports main restricted universe multiverse
EOF
        fi
    else
        print_info "Using default APT sources"
        # For ARM64, use ports.ubuntu.com for better compatibility
        if [ "$arch" = "arm64" ] || [ "$arch" = "armhf" ]; then
            cat > /etc/apt/sources.list <<EOF
# Default Ubuntu ports sources for $arch
deb http://ports.ubuntu.com/ubuntu-ports/ $ubuntu_version main restricted universe multiverse
deb-src http://ports.ubuntu.com/ubuntu-ports/ $ubuntu_version main restricted universe multiverse

deb http://ports.ubuntu.com/ubuntu-ports/ $ubuntu_version-security main restricted universe multiverse
deb-src http://ports.ubuntu.com/ubuntu-ports/ $ubuntu_version-security main restricted universe multiverse

deb http://ports.ubuntu.com/ubuntu-ports/ $ubuntu_version-updates main restricted universe multiverse
deb-src http://ports.ubuntu.com/ubuntu-ports/ $ubuntu_version-updates main restricted universe multiverse

deb http://ports.ubuntu.com/ubuntu-ports/ $ubuntu_version-backports main restricted universe multiverse
deb-src http://ports.ubuntu.com/ubuntu-ports/ $ubuntu_version-backports main restricted universe multiverse
EOF
        fi
    fi

    print_info "APT sources configured successfully for $arch"
}

# Function to configure PIP sources
configure_pip_sources() {
    print_info "Configuring PIP sources with $BEST_PIP_NAME mirror..."

    # Create system-wide pip config
    mkdir -p /etc/pip

    if [ "$BEST_PIP_NAME" != "default" ]; then
        # Create pip config with appropriate URL structure
        local pip_url
        if [ "$BEST_PIP_NAME" = "official" ]; then
            pip_url="https://$BEST_PIP_MIRROR/simple/"
        elif [ "$BEST_PIP_NAME" = "douban" ]; then
            pip_url="http://$BEST_PIP_MIRROR/simple/"
        else
            pip_url="https://$BEST_PIP_MIRROR/pypi/simple/"
        fi

        cat > /etc/pip/pip.conf <<EOF
# Auto-selected fastest mirror: $BEST_PIP_NAME ($BEST_PIP_MIRROR)
[global]
index-url = $pip_url
trusted-host = $BEST_PIP_MIRROR

[install]
trusted-host = $BEST_PIP_MIRROR
EOF
    else
        print_info "Using default PIP sources"
        cat > /etc/pip/pip.conf <<EOF
# Using default PIP sources
[global]
index-url = https://pypi.org/simple/

[install]
EOF
    fi

    print_info "PIP sources configured successfully"
}

# Function to show summary
show_summary() {
    local arch
    arch=$(dpkg --print-architecture)

    echo ""
    echo "=========================================="
    print_info "DOCKER BUILD MIRROR SUMMARY"
    echo "=========================================="
    echo "🏗️  Architecture: $arch"
    echo "✅ APT:  $BEST_APT_NAME ($BEST_APT_MIRROR)"
    echo "✅ PIP:  $BEST_PIP_NAME ($BEST_PIP_MIRROR)"
    echo "✅ ROS2: $BEST_ROS2_NAME ($BEST_ROS2_MIRROR)"
    echo "=========================================="
}

# Function to check dependencies
check_dependencies() {
    local missing_deps=()

    # Check essential tools
    if ! command -v curl > /dev/null 2>&1; then
        missing_deps+=("curl")
    fi

    if ! command -v awk > /dev/null 2>&1 && ! command -v gawk > /dev/null 2>&1; then
        missing_deps+=("awk")
    fi

    if ! command -v lsb_release > /dev/null 2>&1; then
        missing_deps+=("lsb-release")
    fi

    if [ ${#missing_deps[@]} -gt 0 ]; then
        print_info "Missing dependencies: ${missing_deps[*]}"
        print_info "Will use fallback configuration instead of testing"
        return 1
    fi

    return 0
}

# Function to use fallback configuration (when dependencies are missing)
configure_fallback_sources() {
    print_info "Using fallback mirror configuration..."

    local arch
    arch=$(dpkg --print-architecture 2>/dev/null || echo "amd64")
    local ubuntu_version
    ubuntu_version=$(lsb_release -cs 2>/dev/null || echo "jammy")

    # Set fallback mirrors (known to be reliable)
    BEST_APT_MIRROR="mirrors.aliyun.com"
    BEST_APT_NAME="aliyun (fallback)"
    BEST_PIP_MIRROR="mirrors.aliyun.com"
    BEST_PIP_NAME="aliyun (fallback)"
    BEST_ROS2_MIRROR="mirrors.tuna.tsinghua.edu.cn"
    BEST_ROS2_NAME="tuna (fallback)"

    print_info "Fallback configuration:"
    print_info "APT: $BEST_APT_NAME"
    print_info "PIP: $BEST_PIP_NAME"
    print_info "ROS2: $BEST_ROS2_NAME"
}

# Main function
main() {
    print_info "Starting automatic mirror detection for Docker build..."

    # Check if we have required dependencies
    if check_dependencies; then
        print_info "All dependencies available, running full detection..."

        # Test mirrors quickly (parallel would be nice but keeping it simple for Docker)
        test_apt_mirrors
        test_pip_mirrors
    else
        print_info "Dependencies missing, using fallback configuration..."
        configure_fallback_sources
    fi

    # Show summary
    show_summary

    # Configure sources
    configure_apt_sources
    configure_pip_sources

    print_info "Docker build mirror detection and configuration completed!"
}

# Execute main function
main