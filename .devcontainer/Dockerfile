FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

ENV RUSTUP_DIST_SERVER="https://rsproxy.cn"
ENV RUSTUP_UPDATE_ROOT="https://rsproxy.cn/rustup"
ENV CARGO_HOME=/usr/local/cargo
ENV RUSTUP_HOME=/usr/local/rustup
ENV PATH="${CARGO_HOME}/bin:${PATH}"

# Install packages and dependencies
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        # basic tools
        bc \
        ca-certificates \
        curl \
        git \
        gnupg \
        lcov \
        lsb-release \
        vim \
        wget \
        zip \
        dnsutils \
        # additional dev container tools
        openssh-client \
        sudo \
        zsh \
        # python environment
        python3 \
        python3-pip \
        python3-venv \
        # C/C++ compile tools and libraries (for iox plugin and so on)
        build-essential \
        autoconf \
        cmake \
        doxygen \
        libacl1-dev \
        libcurl4-openssl-dev \
        libglib2.0-dev \
        libssl-dev \
        libtool \
        pkg-config \
    && \
    # install ROS 2 (Humble)
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y --no-install-recommends ros-humble-ros-base && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create vscode user
RUN groupadd --gid 1000 vscode \
    && useradd --uid 1000 --gid vscode --shell /bin/bash --create-home vscode \
    && echo "vscode ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/vscode \
    && chmod 0440 /etc/sudoers.d/vscode

# install python dependencies
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt && \
    rm requirements.txt

# install rust (system-wide)
RUN curl --proto '=https' --tlsv1.2 https://sh.rustup.rs -sSf | sh -s -- -y --default-toolchain stable && \
    rustup component add rustfmt clippy

WORKDIR /build
# install zenoh-c
RUN wget -O zenoh-c.tar.gz https://github.com/eclipse-zenoh/zenoh-c/archive/refs/tags/1.0.0.11.tar.gz && \
    tar -xvf zenoh-c.tar.gz && \
    cd zenoh-c-1.0.0.11 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build . --config Release --parallel $(nproc) && \
    cmake --build . --target install && \
    cd / && rm -rf /build

RUN chown -R vscode:vscode /usr/local/cargo /usr/local/rustup

# Create setup-init script
RUN echo '#!/bin/bash\n\
echo "source /opt/ros/humble/setup.bash" >> /home/vscode/.bashrc\n\
echo "source /opt/ros/humble/setup.zsh" >> /home/vscode/.zshrc\n\
echo "Dev container setup completed!"\n\
' > /usr/local/bin/setup-init && \
    chmod +x /usr/local/bin/setup-init

# Switch to vscode user
USER vscode

# install clang-format & cmake-format & autopep8
RUN sudo apt update && \
    sudo apt install -y clang-format-15  cmake-format python3-autopep8 && \
    sudo update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-15 100

WORKDIR /workspaces

# Keep container running
CMD ["sleep", "infinity"]
