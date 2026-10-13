#!/usr/bin/env bash
# 输出本镜像的关键组件版本，写入 /build-info.txt，供 docs/environment-*.md 引用。
# 注意：不要把这些 dpkg-query 直接写在 Dockerfile 的 RUN 里——Dockerfile 会吃掉
# ${...}，导致版本号输出成 "$版本号" 这种残缺形式（踩过一次）。放在被 COPY 的脚本里没有这个问题。
set +u

head -2 /etc/os-release
echo "arch:               $(dpkg --print-architecture)"

dpkg-query -W \
    gazebo \
    ros-humble-desktop ros-humble-slam-toolbox ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
    ros-humble-nav2-map-server ros-humble-robot-state-publisher \
    ros-humble-rosbag2 ros-humble-xacro \
    ignition-fortress libignition-gazebo6-6 \
    python3-colcon-common-extensions \
    2>/dev/null | awk '{printf "%-34s %s\n", $1, $2}'
