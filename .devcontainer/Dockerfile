
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

RUN mkdir -p /build/zenoh-c && \
    cd /build/zenoh-c && \
    wget -O zenoh-c.tar.gz https://github.com/eclipse-zenoh/zenoh-c/archive/refs/tags/1.0.0.11.tar.gz && \
    tar -xvf zenoh-c.tar.gz && \
    cd zenoh-c-1.0.0.11 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build . --config Release --parallel $(nproc) && \
    cmake --build . --target install


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
        git vim zip bc sudo zsh openssh-client dnsutils \
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