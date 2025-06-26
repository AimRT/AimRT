FROM ros:humble-ros-base-jammy AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV RUSTUP_DIST_SERVER="https://rsproxy.cn"
ENV RUSTUP_UPDATE_ROOT="https://rsproxy.cn/rustup"
ENV CARGO_HOME=/usr/local/cargo
ENV RUSTUP_HOME=/usr/local/rustup
ENV PATH="${CARGO_HOME}/bin:${PATH}"

COPY auto_detect_sources.py /tmp/
RUN chmod +x /tmp/auto_detect_sources.py && \
    python3 /tmp/auto_detect_sources.py

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        wget git build-essential  && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN curl --proto '=https' --tlsv1.2 https://sh.rustup.rs -sSf | sh -s -- -y --default-toolchain stable && \
    rustup component add rustfmt clippy


FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV CARGO_HOME=/usr/local/cargo
ENV RUSTUP_HOME=/usr/local/rustup
ENV PATH="${CARGO_HOME}/bin:${PATH}"

COPY auto_detect_sources.py /tmp/
RUN chmod +x /tmp/auto_detect_sources.py && \
    python3 /tmp/auto_detect_sources.py

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git vim zip unzip bc sudo zsh openssh-client dnsutils \
        python3 python3-pip python3-venv python3-autopep8 \
        libcurl4 libglib2.0-0 wget \
        autoconf cmake doxygen libacl1-dev libcurl4-openssl-dev libglib2.0-dev libssl-dev libtool pkg-config software-properties-common \
        clang-format-15 cmake-format software-properties-common && \
    wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | tee /etc/apt/trusted.gpg.d/apt.llvm.org.asc && \
    add-apt-repository "deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy-18 main" && \
    apt-get update && \
    apt-get install -y --no-install-recommends clangd-18 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-15 100

RUN mkdir -p /build/zenoh-c && \
    cd /build/zenoh-c && \
    ARCH=$(uname -m) && \
    if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then \
        wget -O zenoh-c.zip https://github.com/eclipse-zenoh/zenoh-c/releases/download/1.0.0.11/zenoh-c-1.0.0.11-aarch64-unknown-linux-gnu-debian.zip && \
        unzip zenoh-c.zip && \
        dpkg -i /build/zenoh-c/libzenohc_1.0.0.11-1_arm64.deb; \
    else \
        wget -O zenoh-c.zip https://github.com/eclipse-zenoh/zenoh-c/releases/download/1.0.0.11/zenoh-c-1.0.0.11-x86_64-unknown-linux-gnu-debian.zip && \
        unzip zenoh-c.zip  && \
        dpkg -i /build/zenoh-c/libzenohc_1.0.0.11-1_amd64.deb; \
    fi && \
    rm -rf /build/zenoh-c

RUN groupadd --gid 1000 vscode \
    && useradd --uid 1000 --gid vscode --shell /bin/bash --create-home vscode \
    && echo "vscode ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/vscode \
    && chmod 0440 /etc/sudoers.d/vscode

USER vscode
WORKDIR /home/vscode
COPY requirements.txt .
RUN pip3 install --no-cache-dir --user -r requirements.txt && \
    rm requirements.txt

COPY --from=builder --chown=vscode:vscode /usr/local/ /usr/local/

ENV PATH="/home/vscode/.local/bin:${PATH}"

USER root
RUN echo '#!/bin/bash\n\
echo "source /opt/ros/humble/setup.bash" >> /home/vscode/.bashrc\n\
echo "source /opt/ros/humble/setup.zsh" >> /home/vscode/.zshrc\n\
echo "export PATH=/home/vscode/.local/bin:\$PATH" >> /home/vscode/.bashrc\n\
echo "export PATH=/home/vscode/.local/bin:\$PATH" >> /home/vscode/.zshrc\n\
echo "Dev container setup completed!"\n\
' > /usr/local/bin/setup-init && \
    chmod +x /usr/local/bin/setup-init

USER vscode
WORKDIR /workspaces

CMD ["sleep", "infinity"]