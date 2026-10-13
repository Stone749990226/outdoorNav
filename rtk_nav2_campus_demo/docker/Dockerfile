# 二维 SLAM 演示运行环境
#
# 基础镜像：ROS 2 Humble Desktop Full（Ubuntu 22.04）。
# 默认用服务器上已有的本地副本 robonix-osrf-ros:humble-desktop-full；
# 换机器时可以用 --build-arg BASE_IMAGE=... 指定该机器上实际可用的同类镜像
# （脚本 scripts/build-image.sh 会自动挑一个可用的并打印出来）。
# 该镜像已内置 RViz2、ros_gz、teleop_twist_keyboard、xacro、rosbag2。
# 本层补齐 Gazebo Classic 11、slam_toolbox、TurtleBot3 官方模型与地图加载工具。
ARG BASE_IMAGE=robonix-osrf-ros:humble-desktop-full
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GALLIUM_DRIVER=llvmpipe
ENV QT_X11_NO_MITSHM=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

RUN apt-get update && apt-get install -y --no-install-recommends \
        gazebo \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-plugins \
        ros-humble-slam-toolbox \
        ros-humble-turtlebot3 \
        ros-humble-turtlebot3-msgs \
        ros-humble-turtlebot3-gazebo \
        ros-humble-turtlebot3-teleop \
        ros-humble-nav2-map-server \
        ros-humble-teleop-twist-keyboard \
        ros-humble-tf2-tools \
        ros-humble-robot-state-publisher \
        ros-humble-xacro \
        mesa-utils \
        libgl1-mesa-dri \
        libglu1-mesa \
        x11-utils \
        x11-xserver-utils \
        xdotool \
        wmctrl \
        python3-pytest \
        python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# 容器内快速进入工作区
RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    '[ -f /work/demo_ws/install/setup.bash ] && source /work/demo_ws/install/setup.bash' \
    'cd /work/demo_ws 2>/dev/null || true' \
    > /root/.bashrc_ros

WORKDIR /work

# 构建信息（供 docs/environment.md 记录）
COPY docker/build_info.sh /usr/local/bin/build_info.sh
RUN bash /usr/local/bin/build_info.sh > /build-info.txt && cat /build-info.txt
