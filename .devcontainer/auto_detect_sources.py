#!/usr/bin/env python3

import subprocess
import time
import urllib.request
import urllib.error
import socket
import shutil
from pathlib import Path

# color output
class Colors:
    GREEN = '\033[0;32m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    YELLOW = '\033[1;33m'
    RED = '\033[0;31m'
    NC = '\033[0m'  # No Color

def print_info(message: str):
    print(f"{Colors.GREEN}[AUTO-DETECT]{Colors.NC} {message}")

def print_testing(message: str):
    print(f"{Colors.BLUE}[TESTING]{Colors.NC} {message}")

def print_result(message: str):
    print(f"{Colors.CYAN}[RESULT]{Colors.NC} {message}")

def print_warning(message: str):
    print(f"{Colors.YELLOW}[WARNING]{Colors.NC} {message}")

def print_error(message: str):
    print(f"{Colors.RED}[ERROR]{Colors.NC} {message}")

# mirror sources configuration
APT_MIRRORS = {
    "official": "archive.ubuntu.com",
    "security": "security.ubuntu.com",
    "aliyun": "mirrors.aliyun.com",
    "tuna": "mirrors.tuna.tsinghua.edu.cn",
    "ustc": "mirrors.ustc.edu.cn",
    "huawei": "mirrors.huaweicloud.com",
}

APT_MIRRORS_ARM64 = {
    "official": "ports.ubuntu.com",
    "aliyun": "mirrors.aliyun.com",
    "tuna": "mirrors.tuna.tsinghua.edu.cn",
    "ustc": "mirrors.ustc.edu.cn",
    "tencent": "mirrors.cloud.tencent.com",
}

PIP_MIRRORS = {
    "official": "pypi.org",
    "aliyun": "mirrors.aliyun.com",
    "tuna": "pypi.tuna.tsinghua.edu.cn",
    "ustc": "pypi.mirrors.ustc.edu.cn",
    "douban": "pypi.douban.com",
}

class MirrorDetector:
    def __init__(self):
        self.arch = self._get_architecture()
        self.ubuntu_version = self._get_ubuntu_version()
        self.best_apt_mirror = ""
        self.best_apt_name = ""
        self.best_pip_mirror = ""
        self.best_pip_name = ""

    def _get_architecture(self) -> str:
        """get system architecture"""
        try:
            result = subprocess.run(['dpkg', '--print-architecture'],
                                  capture_output=True, text=True, check=True)
            return result.stdout.strip()
        except (subprocess.CalledProcessError, FileNotFoundError):
            return "amd64"  # default value

    def _get_ubuntu_version(self) -> str:
        """get Ubuntu version code name"""
        try:
            result = subprocess.run(['lsb_release', '-cs'],
                                  capture_output=True, text=True, check=True)
            return result.stdout.strip()
        except (subprocess.CalledProcessError, FileNotFoundError):
            return "jammy"  # default value

    def _get_ubuntu_path(self) -> str:
        return "ubuntu-ports" if self.arch in ["arm64", "armhf"] else "ubuntu"
   
    def test_mirror_latency(self, url: str, timeout: int = 5) -> int:
        """test mirror source latency (milliseconds)"""
        try:
            start_time = time.time()
            req = urllib.request.Request(f"http://{url}")
            req.add_header('User-Agent', 'AimRT-Mirror-Detector/1.0')

            with urllib.request.urlopen(req, timeout=timeout) as response:
                # read a small amount of data to ensure the connection is fully established
                response.read(1024)

            end_time = time.time()
            latency_ms = int((end_time - start_time) * 1000)
            return latency_ms

        except (urllib.error.URLError, socket.timeout, Exception):
            return 999000  # return a large value when failed

    def test_apt_mirrors(self):
        """test APT mirror sources"""
        print_info("Testing APT mirrors...")
        print_info(f"Architecture: {self.arch}")

        best_latency = 999999

        # select mirror sources based on architecture
        if self.arch in ["arm64", "armhf"]:
            mirrors = APT_MIRRORS_ARM64
            print_info("Using ARM64-optimized mirror list")
        else:
            mirrors = APT_MIRRORS
            print_info("Using standard mirror list")

        for name, mirror in mirrors.items():
            # construct test URL
            if name == "security":
                test_url = f"{mirror}/{self._get_ubuntu_path()}/dists/{self.ubuntu_version}-security/Release"
            else:
                test_url = f"{mirror}/{self._get_ubuntu_path()}/dists/{self.ubuntu_version}/Release"

            print_testing(f"Testing APT mirror: {name}")
            latency = self.test_mirror_latency(test_url, 5)

            if latency < 999000:
                print_result(f"APT {name}: {latency}ms")
                if latency < best_latency:
                    best_latency = latency
                    self.best_apt_mirror = mirror
                    self.best_apt_name = name
            else:
                print_result(f"APT {name}: Failed")

        if self.best_apt_mirror:
            print_info(f"Selected APT mirror: {self.best_apt_name} ({self.best_apt_mirror}) - {best_latency}ms")
        else:
            print_info("No fast APT mirror found, using default")
            if self.arch in ["arm64", "armhf"]:
                self.best_apt_mirror = "ports.ubuntu.com"
                self.best_apt_name = "default"  # default value
            else:
                self.best_apt_mirror = "archive.ubuntu.com"
                self.best_apt_name = "default"

    def test_pip_mirrors(self):
        """test PIP mirror sources"""
        print_info("Testing PIP mirrors...")

        best_latency = 999999

        for name, mirror in PIP_MIRRORS.items():
            # construct test URL
            if name == "official":
                test_url = f"{mirror}/simple/"
            elif name == "douban":
                test_url = f"{mirror}/simple/"
            else:
                test_url = f"{mirror}/pypi/simple/"

            print_testing(f"Testing PIP mirror: {name}")
            latency = self.test_mirror_latency(test_url, 5)

            if latency < 999000:
                print_result(f"PIP {name}: {latency}ms")
                if latency < best_latency:
                    best_latency = latency
                    self.best_pip_mirror = mirror
                    self.best_pip_name = name
            else:
                print_result(f"PIP {name}: Failed")

        if self.best_pip_mirror:
            print_info(f"Selected PIP mirror: {self.best_pip_name} ({self.best_pip_mirror}) - {best_latency}ms")
        else:
            print_info("No fast PIP mirror found, using default")
            self.best_pip_mirror = "pypi.org"
            self.best_pip_name = "default"

    def configure_fallback_sources(self):
        """configure fallback sources (when dependencies are missing)"""
        print_info("Using fallback configuration - keeping default sources...")

        # set default sources based on architecture (no modifications)
        if self.arch in ["arm64", "armhf"]:
            self.best_apt_mirror = "ports.ubuntu.com"
            self.best_apt_name = "default"
        else:
            self.best_apt_mirror = "archive.ubuntu.com"
            self.best_apt_name = "default"

        self.best_pip_mirror = "pypi.org"
        self.best_pip_name = "default"

        print_info(f"Fallback configuration for {self.arch}:")
        print_info(f"APT: {self.best_apt_name}")
        print_info(f"PIP: {self.best_pip_name}")

    def configure_apt_sources(self):
        """configure APT sources"""
        print_info(f"Configuring APT sources with {self.best_apt_name} mirror...")
        print_info(f"Detected architecture: {self.arch}")

        # backup original configuration
        sources_list = Path("/etc/apt/sources.list")
        if sources_list.exists():
            shutil.copy2(sources_list, f"{sources_list}.backup")

        # if default source, no modifications needed
        if self.best_apt_name.startswith("default"):
            print_info("Using default APT sources - no modifications needed")
            # restore backup (if exists)
            backup_file = Path(f"{sources_list}.backup")
            if backup_file.exists():
                print_info("Restoring original APT sources")
                shutil.move(backup_file, sources_list)
            else:
                print_info("Keeping existing APT sources unchanged")
            return

        # check if mirror supports current architecture
        mirror_supports_arch = True
        if self.arch == "arm64" and self.best_apt_name in ["163", "huawei"]:
            print_info(f"Mirror {self.best_apt_name} may not fully support ARM64, using alternative")
            mirror_supports_arch = False

        if not mirror_supports_arch:
            print_info(f"Using default APT sources for better {self.arch} compatibility")
            self._write_default_apt_sources()
            return

        # configure mirror sources
        self._write_mirror_apt_sources()
        print_info(f"APT sources configured successfully for {self.arch}")

    def _write_default_apt_sources(self):
        """write default APT sources"""
        
        _apt_mirror = "archive.ubuntu.com"
        if self.arch in ["arm64", "armhf"]:
            _apt_mirror = "ports.ubuntu.com"

        content = f"""# Default Ubuntu sources for {self.arch} compatibility
deb http://{_apt_mirror}/{self._get_ubuntu_path()}/ {self.ubuntu_version} main restricted universe multiverse
deb-src http://{_apt_mirror}/{self._get_ubuntu_path()}/ {self.ubuntu_version} main restricted universe multiverse

# Security updates (security.ubuntu.com supports all architectures including amd64, arm64, armhf, etc.)
deb http://security.ubuntu.com/ubuntu/ {self.ubuntu_version}-security main restricted universe multiverse
deb-src http://security.ubuntu.com/ubuntu/ {self.ubuntu_version}-security main restricted universe multiverse

deb http://{_apt_mirror}/{self._get_ubuntu_path()}/ {self.ubuntu_version}-updates main restricted universe multiverse
deb-src http://{_apt_mirror}/{self._get_ubuntu_path()}/ {self.ubuntu_version}-updates main restricted universe multiverse

deb http://{_apt_mirror}/{self._get_ubuntu_path()}/ {self.ubuntu_version}-backports main restricted universe multiverse
deb-src http://{_apt_mirror}/{self._get_ubuntu_path()}/ {self.ubuntu_version}-backports main restricted universe multiverse
"""
        with open("/etc/apt/sources.list", "w") as f:
            f.write(content)

    def _write_mirror_apt_sources(self):
        """write mirror APT sources"""
        if self.best_apt_name == "security":
            content = f"""# Auto-selected fastest mirror: {self.best_apt_name} ({self.best_apt_mirror}) + official
# Architecture: {self.arch}
deb http://archive.ubuntu.com/ubuntu/ {self.ubuntu_version} main restricted universe multiverse
deb-src http://archive.ubuntu.com/ubuntu/ {self.ubuntu_version} main restricted universe multiverse

deb http://{self.best_apt_mirror}/ubuntu/ {self.ubuntu_version}-security main restricted universe multiverse
deb-src http://{self.best_apt_mirror}/ubuntu/ {self.ubuntu_version}-security main restricted universe multiverse

deb http://archive.ubuntu.com/ubuntu/ {self.ubuntu_version}-updates main restricted universe multiverse
deb-src http://archive.ubuntu.com/ubuntu/ {self.ubuntu_version}-updates main restricted universe multiverse

deb http://archive.ubuntu.com/ubuntu/ {self.ubuntu_version}-backports main restricted universe multiverse
deb-src http://archive.ubuntu.com/ubuntu/ {self.ubuntu_version}-backports main restricted universe multiverse
"""
        else:
            content = f"""# Auto-selected fastest mirror: {self.best_apt_name} ({self.best_apt_mirror})
# Architecture: {self.arch}
deb http://{self.best_apt_mirror}/{self._get_ubuntu_path()}/ {self.ubuntu_version} main restricted universe multiverse
deb-src http://{self.best_apt_mirror}/{self._get_ubuntu_path()}/ {self.ubuntu_version} main restricted universe multiverse

deb http://{self.best_apt_mirror}/{self._get_ubuntu_path()}/ {self.ubuntu_version}-security main restricted universe multiverse
deb-src http://{self.best_apt_mirror}/{self._get_ubuntu_path()}/ {self.ubuntu_version}-security main restricted universe multiverse

deb http://{self.best_apt_mirror}/{self._get_ubuntu_path()}/ {self.ubuntu_version}-updates main restricted universe multiverse
deb-src http://{self.best_apt_mirror}/{self._get_ubuntu_path()}/ {self.ubuntu_version}-updates main restricted universe multiverse

deb http://{self.best_apt_mirror}/{self._get_ubuntu_path()}/ {self.ubuntu_version}-backports main restricted universe multiverse
deb-src http://{self.best_apt_mirror}/{self._get_ubuntu_path()}/ {self.ubuntu_version}-backports main restricted universe multiverse
"""

        with open("/etc/apt/sources.list", "w") as f:
            f.write(content)

    def configure_pip_sources(self):
        """configure PIP sources"""
        print_info(f"Configuring PIP sources with {self.best_pip_name} mirror...")

        pip_conf_dir = Path("/etc")
        pip_conf_dir.mkdir(exist_ok=True)
        pip_conf_file = pip_conf_dir / "pip.conf"

        # if default source, no modifications needed
        if self.best_pip_name.startswith("default"):
            print_info("Using default PIP sources - no modifications needed")
            if pip_conf_file.exists():
                print_info("Removing custom PIP config to use system defaults")
                pip_conf_file.unlink()
            return

        # construct PIP URL
        if self.best_pip_name == "official":
            pip_url = f"https://{self.best_pip_mirror}/simple/"
        elif self.best_pip_name == "douban":
            pip_url = f"http://{self.best_pip_mirror}/simple/"
        else:
            pip_url = f"https://{self.best_pip_mirror}/pypi/simple/"

        content = f"""# Auto-selected fastest mirror: {self.best_pip_name} ({self.best_pip_mirror})
[global]
index-url = {pip_url}
trusted-host = {self.best_pip_mirror}
"""

        with open(pip_conf_file, "w") as f:
            f.write(content)

        print_info("PIP sources configured successfully")

    def show_summary(self):
        """show configuration summary"""
        print()
        print("=" * 50)
        print_info("MIRROR DETECTION SUMMARY")
        print("=" * 50)
        print(f"🏗️  Architecture: {self.arch}")
        print(f"✅ APT:  {self.best_apt_name} ({self.best_apt_mirror})")
        print(f"✅ PIP:  {self.best_pip_name} ({self.best_pip_mirror})")
        print("=" * 50)

    def check_dependencies(self) -> bool:
        """check dependencies"""
        required_commands = ['curl', 'lsb_release', 'dpkg']
        missing_deps = []

        for cmd in required_commands:
            if not shutil.which(cmd):
                missing_deps.append(cmd)

        if missing_deps:
            print_info(f"Missing dependencies: {', '.join(missing_deps)}")
            print_info("Will use fallback configuration instead of testing")
            return False

        return True

    def run(self):
        """run main program"""
        print_info("Starting automatic mirror detection...")

        # check dependencies
        if self.check_dependencies():
            print_info("All dependencies available, running full detection...")

            # test mirror sources
            self.test_apt_mirrors()
            self.test_pip_mirrors()
        else:
            print_info("Dependencies missing, using fallback configuration...")
            self.configure_fallback_sources()

        # show summary
        self.show_summary()

        # configure sources
        try:
            self.configure_apt_sources()
            self.configure_pip_sources()
            print_info("Mirror detection and configuration completed!")
        except PermissionError:
            print_error("Permission denied. Please run with sudo privileges.")
            return 1
        except Exception as e:
            print_error(f"Configuration failed: {e}")
            return 1

        return 0


def main():
    """main entry function"""
    detector = MirrorDetector()
    return detector.run()


if __name__ == "__main__":
    exit(main())