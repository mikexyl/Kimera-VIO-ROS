FROM ubuntu_base:noetic

ARG USERNAME=vscode
ARG USER_UID=1000
ENV ROS_DISTRO=noetic
ARG PIXI_VERSION=v0.48.0

ENV DEBIAN_FRONTEND=noninteractive

# install ros-perception dependencies
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-perception \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-rqt-graph \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-image-pipeline \
    libspdlog-dev libgraphviz-dev kitty \
    libmetis-dev libgflags-dev libgoogle-glog-dev googletest \
    python3-catkin-tools python3-vcstool ccache libblas-dev python3-empy && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Configure ccache for faster rebuilds
ENV CCACHE_DIR=/tmp/ccache
ENV CCACHE_MAXSIZE=2G
RUN mkdir -p /tmp/ccache && chmod 777 /tmp/ccache

# install taskfile
RUN sh -c "$(curl --location https://taskfile.dev/install.sh)" -- -d -b /usr/local/bin

# install ros2 develop tools
# RUN --mount=type=cache,target=/var/cache/apt \
#     --mount=type=cache,target=/var/lib/apt \
#     apt-get update && apt-get install -y \
#     python3-vcstool \
#     python3-rosdep ccache ninja-build && \
#     apt-get clean && \
#     rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y curl bzip2 ca-certificates wget \
    && rm -rf /var/lib/apt/lists/*

# create user
RUN groupadd --gid ${USER_UID} ${USERNAME} && \
    useradd --create-home --shell /bin/bash --uid ${USER_UID} --gid ${USER_UID} ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

RUN curl -L -o /usr/local/bin/pixi -fsSL --compressed "https://github.com/prefix-dev/pixi/releases/download/${PIXI_VERSION}/pixi-$(uname -m)-unknown-linux-musl" \
    && chmod +x /usr/local/bin/pixi \
    && pixi info

# Ensure empy is in PATH (do this before switching to non-root user)
RUN ln -sf /usr/bin/empy3 /usr/local/bin/empy || ln -sf /usr/bin/empy /usr/local/bin/empy || true

# change to USER and install rust
USER $USERNAME
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y && \
    echo 'source $HOME/.cargo/env' >> $HOME/.zshrc && \
    . "$HOME/.cargo/env" && \
    rustup default stable

RUN echo 'eval "$(pixi completion -s zsh)"' >> $HOME/.zshrc

