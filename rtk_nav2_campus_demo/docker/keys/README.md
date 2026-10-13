# apt 签名密钥

`ros-archive-keyring.gpg` 由 `scripts/build-image.sh` 在构建前从构建机的
`/usr/share/keyrings/ros-archive-keyring.gpg` 复制到这里，再由 Dockerfile 装进镜像。

原因：`http://packages.ros.org/ros.key` 目前下发的是**已过期**的旧公钥
（指纹 C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654，过期时间 2025-06-01），
用它做 `signed-by` 会让 apt 报 `EXPKEYSIG`，装不了任何 ROS 包。
构建机上那份 keyring 是更新过的（同一指纹，有效期到 2030），因此直接复用。

`gazebo.key`（Gazebo Sim / Fortress 源）不需要预置：它的公钥仍然有效，
但它是 ASCII-armored 格式，apt 的 `signed-by` 读不了，Dockerfile 里会用
`gpg --dearmor` 转成二进制后再使用。
