# Livox ROS Driver 2（ROS 2-only）

这是针对当前工作空间精简后的 ROS 2 驱动，只保留 ament/colcon、ROS 2 节点、消息和启动文件。ROS 1、catkin、ROS 1 rosbag 与仅 ROS 1 可用的 PCL PointXYZI 输出已经移除。

完整的本机网络配置、MID-360 启动、FAST-LIO 接入与故障排查请阅读工作空间根目录的 `README.md`。

在运行 FAST-LIO 前，还需要下载其 ROS 2 源码、递归获取子模块、安装依赖并用 colcon 构建。完整命令见根目录 README 的“下载并构建 FAST-LIO2”章节；仅构建 Livox 驱动并不会自动安装 FAST-LIO。

## 构建

```bash
cd /home/pc/livox_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select livox_ros_driver2
source install/setup.bash
```


## MID-360 启动

查看 PointCloud2 点云并打开 RViz2：

```bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

为 FAST-LIO 发布带逐点时间戳的 Livox CustomMsg：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

当前支持的点云传输格式：

- `xfer_format = 0`：`sensor_msgs/msg/PointCloud2`
- `xfer_format = 1`：`livox_ros_driver2/msg/CustomMsg`

设备地址和主机网口参数位于 `config/MID360_config.json`。

上游项目：<https://github.com/Livox-SDK/livox_ros_driver2>
