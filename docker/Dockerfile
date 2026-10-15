FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0


ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# 基础工具 + locale
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    sudo \
    git \
    vim \
    nano \
    htop \
    iputils-ping \
    net-tools \
    usbutils \
    v4l-utils \
    ffmpeg \
    netcat-openbsd \
    python3-pip \
    python3-venv \
    python3-aiohttp \
    python3-requests \
    build-essential \
    cmake \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

# 添加 ROS2 Humble 官方源
RUN apt-get update && apt-get install -y curl gnupg2 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
       > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

# 安装 ROS2 Humble 基础环境和常用包
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

COPY boat_ws/wheels/onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl /tmp/onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl
COPY boat_ws/wheels/pyrealsense2-2.58.2-cp310-cp310-linux_aarch64.whl /tmp/pyrealsense2-2.58.2-cp310-cp310-linux_aarch64.whl

RUN python3 -m pip install --upgrade pip 'setuptools<80' wheel && \
    python3 -m pip install --no-cache-dir \
    'numpy<2' \
    /tmp/onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl \
    /tmp/pyrealsense2-2.58.2-cp310-cp310-linux_aarch64.whl && \
    rm -f \
    /tmp/onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl \
    /tmp/pyrealsense2-2.58.2-cp310-cp310-linux_aarch64.whl

# rosdep 初始化
RUN rosdep init || true
RUN rosdep update || true

# SR73F SocketCAN and PX4 MAVLink runtime tools. Keep this in a late layer so
# updating hardware tooling does not invalidate the large ROS/Jetson layers.
RUN apt-get update && apt-get install -y \
    iproute2 \
    can-utils \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir 'pymavlink==2.4.49'

RUN apt-get update && apt-get install -y \
    fonts-noto-cjk \
    python3-pil \
    && rm -rf /var/lib/apt/lists/*

# 创建和宿主机同名用户，避免生成的文件全是 root 权限
ARG USERNAME=ubuntu2204
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} || true \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} || true \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}

WORKDIR /workspace/boat_ws

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=39" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc

CMD ["/bin/bash"]
