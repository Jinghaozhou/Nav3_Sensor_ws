# FAST-LIO2 + Livox MID360 本机部署说明

本文只面向当前电脑和一台 Livox MID360，说明如何完成网络连接、源码准备、构建、参数配置、启动、验证和地图保存。完整算法介绍请看原始 [README.md](README.md)。

## 1. 当前环境

已经确认的本机环境：

| 项目 | 当前配置 |
|---|---|
| 系统 | Ubuntu 24.04.1 LTS，x86_64 |
| ROS 2 | Jazzy，安装于 `/opt/ros/jazzy` |
| CPU | Intel Core i9-13980HX，24 核 32 线程 |
| 内存 | 62 GiB |
| FAST-LIO 工作空间 | `/home/pc/Fast_lio_ws` |
| Livox 驱动工作空间 | `/home/pc/livox_ws` |
| 有线网卡 | `eno2` |
| 主机有线 IP | `192.168.1.5/24` |
| MID360 IP | `192.168.1.106` |

这台电脑的算力和内存足够实时运行 FAST-LIO2。Livox ROS 2 驱动已经编译，位置为：

```text
/home/pc/livox_ws/install/livox_ros_driver2
```

当前 FAST-LIO 尚未构建，而且 `include/ikd-Tree` 目录为空。首次构建前必须先完成第 3 节。

## 2. 连接并配置 MID360 网络

MID360 使用网线连接电脑的 `eno2` 接口。本机已经有名为 `livox-mid360` 的 NetworkManager 配置，其中静态地址已经设为 `192.168.1.5/24`，不设置网关和 DNS；Wi-Fi 可以继续负责上网。

接好 MID360 的电源和网线后执行：

```bash
sudo nmcli connection up livox-mid360
ip -br -4 addr show eno2
ping -c 4 192.168.1.106
```

正常时，`eno2` 应显示 `192.168.1.5/24`，并且能 ping 通 `192.168.1.106`。

驱动使用的配置文件是：

```text
/home/pc/livox_ws/src/livox_ros_driver2/config/MID360_config.json
```

其中已经配置为：

```text
host IP : 192.168.1.5
LiDAR IP: 192.168.1.106
```

如果更换了 MID360，必须把 JSON 中 `lidar_configs` 下的 `ip` 改成新雷达的真实 IP。主机 IP、JSON 中的 `host_net_info` 和 `eno2` 的静态 IP 必须保持一致。

## 3. 准备 FAST-LIO 源码

这个源码副本没有 Git 仓库元数据，不能直接执行 `git submodule update`。当前 `include/ikd-Tree` 是空目录，使用下面的命令补齐：

```bash
cd /home/pc/Fast_lio_ws/src/FAST_LIO
git clone -b fast_lio https://github.com/hku-mars/ikd-Tree.git include/ikd-Tree
```

确认核心文件存在：

```bash
test -f include/ikd-Tree/ikd_Tree.cpp && echo "ikd-Tree 已就绪"
```

如果以后重新下载整个 FAST-LIO 仓库，应使用带子模块的方式：

```bash
git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git FAST_LIO
```

## 4. 安装依赖

本机已经安装 Eigen、PCL、`pcl_ros` 和 `pcl_conversions`。仍建议在首次构建前让 rosdep 检查一次所有 ROS 依赖：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/pc/Fast_lio_ws
rosdep install --from-paths src --ignore-src -r -y \
  --rosdistro jazzy \
  --skip-keys livox_ros_driver2
```

如果系统提示 rosdep 尚未初始化，只需执行一次：

```bash
sudo rosdep init
rosdep update
```

## 5. 构建 FAST-LIO

构建 FAST-LIO 前必须先加载 ROS 2 Jazzy 和已经编译好的 Livox 驱动，否则 CMake 找不到 `livox_ros_driver2` 的消息类型。

```bash
source /opt/ros/jazzy/setup.bash
source /home/pc/livox_ws/install/setup.bash

cd /home/pc/Fast_lio_ws
colcon build \
  --symlink-install \
  --packages-select fast_lio \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

构建成功后检查：

```bash
source /home/pc/Fast_lio_ws/install/setup.bash
ros2 pkg prefix fast_lio
ros2 pkg executables fast_lio
```

预期能够看到 `/home/pc/Fast_lio_ws/install/fast_lio` 和可执行程序 `fastlio_mapping`。

重新构建时仍然按以下顺序加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source /home/pc/livox_ws/install/setup.bash
source /home/pc/Fast_lio_ws/install/setup.bash
```

不要把工作空间加载顺序颠倒：基础 ROS 在前，Livox 驱动其次，FAST-LIO 最后。

## 6. MID360 参数配置

FAST-LIO 使用：

```text
/home/pc/Fast_lio_ws/src/FAST_LIO/config/mid360.yaml
```

关键配置如下：

```yaml
common:
  lid_topic: "/livox/lidar"
  imu_topic: "/livox/imu"
  time_sync_en: false
  time_offset_lidar_to_imu: 0.0

preprocess:
  lidar_type: 1
  scan_line: 4
  blind: 0.5
  timestamp_unit: 3
  scan_rate: 10
```

现有 Livox 启动文件使用 `xfer_format = 1`，会发布带逐点时间戳的 `livox_ros_driver2/msg/CustomMsg`。这与 `lidar_type: 1` 以及 `/livox/lidar` 完全匹配，不要改成驱动的 `rviz_MID360_launch.py`，否则会变成普通 `PointCloud2`。

### 雷达与 IMU 外参

`mapping.extrinsic_T` 和 `mapping.extrinsic_R` 表示“雷达坐标系在 IMU 坐标系中的位姿”，即以 IMU 为基准的 LiDAR 外参。当前配置是：

```yaml
mapping:
  extrinsic_est_en: true
  extrinsic_T: [-0.011, -0.02329, 0.04412]
  extrinsic_R: [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
```

首次验证可以保留现有值。如果已经通过标定获得可靠外参，建议填入标定值并将 `extrinsic_est_en` 改为 `false`，避免在线估计引起漂移。不要随意复制其他安装位置的外参。

### 地图保存

建议先创建地图目录：

```bash
mkdir -p /home/pc/Fast_lio_ws/maps
```

然后把 `mid360.yaml` 中的保存配置改成绝对路径：

```yaml
map_file_path: "/home/pc/Fast_lio_ws/maps/mid360_map.pcd"

pcd_save:
  pcd_save_en: true
  interval: -1
```

`interval: -1` 会把全部地图保存在一个文件中。长时间建图可能占用较多内存；长距离任务应设置分段保存帧数。

## 7. 部署启动

建议使用三个终端，按下面的顺序启动。

### 终端 1：启动 MID360 驱动

```bash
source /opt/ros/jazzy/setup.bash
source /home/pc/livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 终端 2：确认雷达和 IMU 数据

```bash
source /opt/ros/jazzy/setup.bash
source /home/pc/livox_ws/install/setup.bash

ros2 topic list -t | grep '^/livox/'
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

应至少看到：

```text
/livox/lidar [livox_ros_driver2/msg/CustomMsg]
/livox/imu   [sensor_msgs/msg/Imu]
```

只有这两个话题都在持续更新时，才继续启动 FAST-LIO。

### 终端 3：启动 FAST-LIO 和 RViz2

```bash
source /opt/ros/jazzy/setup.bash
source /home/pc/livox_ws/install/setup.bash
source /home/pc/Fast_lio_ws/install/setup.bash

ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

不需要图形界面时：

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false
```

启动后的最初几秒应让 MID360 保持静止，便于 IMU 初始化。随后缓慢移动设备，先检查姿态、里程计和点云方向是否正确，再进行正式建图。

## 8. 运行检查

FAST-LIO 正常运行后执行：

```bash
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic echo /Odometry --once
```

主要输出话题：

| 话题 | 内容 |
|---|---|
| `/Odometry` | 融合后的实时位姿 |
| `/path` | 运动轨迹 |
| `/cloud_registered` | 世界坐标系中的配准点云 |
| `/cloud_registered_body` | 机体坐标系中的点云 |
| `/Laser_map` | 累积地图 |

如果 RViz 中没有点云，先检查 Fixed Frame 是否与节点发布的坐标系一致，再检查 `/livox/lidar`、`/livox/imu` 和 `/Odometry` 是否有数据。

## 9. 保存地图

保持 FAST-LIO 节点运行，在另一个终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/pc/livox_ws/install/setup.bash
source /home/pc/Fast_lio_ws/install/setup.bash

ros2 service call /map_save std_srvs/srv/Trigger "{}"
ls -lh /home/pc/Fast_lio_ws/maps/mid360_map.pcd
```

服务返回 `success: true` 才表示保存成功。可使用下面的命令查看地图：

```bash
pcl_viewer /home/pc/Fast_lio_ws/maps/mid360_map.pcd
```

## 10. 常见问题

### 找不到 `ikd_Tree.cpp`

原因是 ikd-Tree 子模块缺失。重新执行第 3 节的克隆命令，并确认下面的文件存在：

```text
/home/pc/Fast_lio_ws/src/FAST_LIO/include/ikd-Tree/ikd_Tree.cpp
```

### 找不到 `livox_ros_driver2`

构建或启动前遗漏了驱动环境：

```bash
source /opt/ros/jazzy/setup.bash
source /home/pc/livox_ws/install/setup.bash
```

### 驱动启动但没有 `/livox/lidar`

依次检查：

```bash
ip -br -4 addr show eno2
ping -c 4 192.168.1.106
grep -n '192.168.1' /home/pc/livox_ws/src/livox_ros_driver2/config/MID360_config.json
```

主机必须是 `192.168.1.5/24`，当前雷达必须是 `192.168.1.106`，网线和雷达供电必须正常。

### 点云跳动、重影或轨迹漂移

优先检查：

1. 启动时设备是否保持静止完成 IMU 初始化；
2. 雷达和 IMU 时间戳是否正常；
3. 是否错误开启 `time_sync_en`；
4. `extrinsic_T` 和 `extrinsic_R` 是否符合实际安装关系；
5. 设备安装是否松动、振动是否过大。

MID360 使用内置 IMU 时保持 `time_sync_en: false`。只有外部时间同步确实无法实现时，才考虑使用软件时间同步。

## 11. 最短启动清单

首次使用：

```bash
sudo nmcli connection up livox-mid360
ping -c 4 192.168.1.106

cd /home/pc/Fast_lio_ws/src/FAST_LIO
git clone -b fast_lio https://github.com/hku-mars/ikd-Tree.git include/ikd-Tree

source /opt/ros/jazzy/setup.bash
source /home/pc/livox_ws/install/setup.bash
cd /home/pc/Fast_lio_ws
colcon build --symlink-install --packages-select fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

日常启动：

```bash
# 终端 1
source /opt/ros/jazzy/setup.bash
source /home/pc/livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 终端 2
source /opt/ros/jazzy/setup.bash
source /home/pc/livox_ws/install/setup.bash
source /home/pc/Fast_lio_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```
