# FAST-LIO2 + Livox MID360 工作空间使用说明

## 13区候选与区域维护（2026-11-24）

当前实验扫描工具位于 `tools/mid360_lidar`。显式 `bbs`、`virtual` 模式支持 `space_1～space_13`；启动仍指定实际区域，成功后全图先验跟踪。space_2 默认双路与冻结40帧不变，无KISS回退。组合launch自动启动只读区域维护，输出当前区域、RViz文字和跨区事件，采用三维标签投票及连续确认；边界/过期报告不确定，不改定位TF，不自动切换恢复候选区域。

模型与索引二进制不入Git。`prepare_all_regions.py` 可从已核对地图缓存生成区域模型，`prepare_region_tracking.py` 从生产区域缓存生成索引。已保留旧space_2/10模型；新自动站位未认证地面层、Go2通行性或任意起点定位准确率。源码改动无需重编译现有定位C++，需下一轮整轮重启。只修改本模块，不提交主仓库指针。

本工作空间把 Livox MID360 驱动和 FAST-LIO2 放在一起构建、部署：

```text
/home/jinghao/Nav3_Sensor_ws
├── src
│   ├── livox_ros_driver2   # MID360 驱动、CustomMsg 和网络配置
│   ├── FAST_LIO           # 激光惯性里程计与建图
│   └── mid360_bringup     # 驱动、建图、监控和 RViz 组合启动
├── build                  # colcon 构建目录
├── install                # 构建后的统一环境
└── log                    # 构建日志
```

本文针对当前电脑：Ubuntu 22.04、ROS 2 Humble、Livox MID360。

## 1. 当前配置

| 项目 | 配置 |
|---|---|
| 工作空间 | `/home/jinghao/Nav3_Sensor_ws` |
| ROS 2 | Humble |
| 有线网卡 | `eno2` |
| 主机有线 IP | `192.168.1.5/24` |
| MID360 IP | `192.168.1.134` |
| 雷达点云话题 | `/livox/lidar` |
| IMU 话题 | `/livox/imu` |

相关配置文件：

- MID360 网络：[MID360_config.json](src/livox_ros_driver2/config/MID360_config.json)
- FAST-LIO 参数：[mid360.yaml](src/FAST_LIO/config/mid360.yaml)
- FAST-LIO 中文详细说明：[README_CN.md](src/FAST_LIO/README_CN.md)

## 2. 首次构建前检查

### 2.1 检查 ROS 2

```bash
source /opt/ros/humble/setup.bash
echo "$ROS_DISTRO"
```

应输出：

```text
humble
```

### 2.2 检查 Livox SDK2

驱动优先使用本工作空间 `external/lib/liblivox_lidar_sdk_shared.so`，也支持系统安装的 SDK2。
当前 SDK2 已安装到 `external`。检查：

```bash
cd /home/jinghao/Nav3_Sensor_ws
test -f external/lib/liblivox_lidar_sdk_shared.so && echo "SDK2 已就绪"
```

需要重新构建 SDK2 时（源码位于本工作空间的 `third_party/Livox-SDK2`，复制自 [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) 的 `c0796f0` 提交，许可证见源码目录的 `LICENSE.txt`）：

```bash
cmake -S third_party/Livox-SDK2 -B build/livox_sdk2 \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/external"
cmake --build build/livox_sdk2 -j 4
cmake --install build/livox_sdk2
```

### 2.3 检查 ikd-Tree

```bash
test -f /home/jinghao/Nav3_Sensor_ws/src/FAST_LIO/include/ikd-Tree/ikd_Tree.cpp \
  && echo "ikd-Tree 已就绪"
```

当前工作空间已包含该文件。如果以后重新克隆 FAST-LIO，应同步拉取它的 Git 子模块。

## 3. 安装依赖

初始化并更新 rosdep（已初始化的电脑跳过 `init`）：

```bash
sudo rosdep init
```

如果网络需要代理，请按本机实际代理地址配置，不要照搬其他电脑的端口。

`rosdep init` 每台电脑只需执行一次。如果提示已经初始化，可直接继续。只更新本机需要的 Humble 数据：

```bash
rosdep update --rosdistro humble
```

安装工作空间内各源码包需要的系统依赖：

```bash
source /opt/ros/humble/setup.bash
cd /home/jinghao/Nav3_Sensor_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

`pkg_resources is deprecated` 是 Python 弃用警告，不影响 rosdep 使用。

## 4. 构建整个工作空间

驱动和 FAST-LIO 现在位于同一个工作空间，构建前只加载 ROS 2，不再加载旧的 `/home/jinghao/livox_ws`：

```bash
source /opt/ros/humble/setup.bash
cd /home/jinghao/Nav3_Sensor_ws

CMAKE_BUILD_PARALLEL_LEVEL=4 colcon build \
  --symlink-install \
  --packages-select livox_ros_driver2 fast_lio mid360_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`colcon` 会按照依赖顺序构建 Livox 驱动、FAST-LIO 和组合启动包。

构建成功后加载统一环境：

```bash
source /opt/ros/humble/setup.bash
source /home/jinghao/Nav3_Sensor_ws/install/setup.bash
export LD_LIBRARY_PATH="/home/jinghao/Nav3_Sensor_ws/external/lib:${LD_LIBRARY_PATH:-}"
```

检查三个包是否都来自当前工作空间：

```bash
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix fast_lio
ros2 pkg prefix mid360_bringup
ros2 pkg executables livox_ros_driver2
ros2 pkg executables fast_lio
ros2 pkg executables mid360_bringup
```

三个包的 prefix 都应位于：

```text
/home/jinghao/Nav3_Sensor_ws/install/
```

如果驱动仍然显示为旧的 `/home/jinghao/livox_ws/install/livox_ros_driver2`，请重新打开终端，只加载 `/opt/ros/humble/setup.bash`，然后重新执行本节构建命令。

## 5. 配置 MID360 网络

本机有线接口为 `eno2`，已创建 `livox-mid360` 静态地址连接，主机地址为 `192.168.1.5/24`。当前配置的雷达地址为 `192.168.1.134`，启动前请用下方命令确认可达。直接连接雷达时应使用静态地址；自动获取 IP 会因没有 DHCP 服务而连接失败。

更换网卡后，请通过 `ip -br -4 addr` 确认接口名并调整下方命令。

仅在连接尚不存在时创建：

```bash
sudo nmcli connection add type ethernet ifname eno2 con-name livox-mid360 \
  ipv4.method manual ipv4.addresses 192.168.1.5/24 ipv4.never-default yes \
  ipv6.method disabled
```

已有该连接时直接启用：

```bash
sudo nmcli connection up livox-mid360
ip -br -4 addr show eno2
ping -c 4 192.168.1.134
```

正常状态：

- `eno2` 地址为 `192.168.1.5/24`；
- 能 ping 通 MID360 的 `192.168.1.134`；
- Wi-Fi 可继续用于上网，有线连接不需要网关和 DNS。

驱动配置中的地址必须与实际设备一致：

```json
"host_net_info": {
  "cmd_data_ip": "192.168.1.5",
  "point_data_ip": "192.168.1.5",
  "imu_data_ip": "192.168.1.5"
},
"lidar_configs": [
  { "ip": "192.168.1.134" }
]
```

如果更换了雷达，需要修改 [MID360_config.json](src/livox_ros_driver2/config/MID360_config.json) 中的雷达 IP。

## 6. 运行

建议使用三个终端。

### 终端1：启动 MID360 驱动

```bash
source /opt/ros/humble/setup.bash
source /home/jinghao/Nav3_Sensor_ws/install/setup.bash
export LD_LIBRARY_PATH="/home/jinghao/Nav3_Sensor_ws/external/lib:${LD_LIBRARY_PATH:-}"

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

该启动文件使用 `xfer_format = 1`，输出 FAST-LIO 需要的 `livox_ros_driver2/msg/CustomMsg`。

### 终端2：检查雷达和 IMU

```bash
source /opt/ros/humble/setup.bash
source /home/jinghao/Nav3_Sensor_ws/install/setup.bash
export LD_LIBRARY_PATH="/home/jinghao/Nav3_Sensor_ws/external/lib:${LD_LIBRARY_PATH:-}"

ros2 topic list -t | grep '^/livox/'
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

至少应看到：

```text
/livox/lidar [livox_ros_driver2/msg/CustomMsg]
/livox/imu   [sensor_msgs/msg/Imu]
```

确认两个话题持续更新后再启动 FAST-LIO。

### 终端3：启动 FAST-LIO 和 RViz2

```bash
source /opt/ros/humble/setup.bash
source /home/jinghao/Nav3_Sensor_ws/install/setup.bash
export LD_LIBRARY_PATH="/home/jinghao/Nav3_Sensor_ws/external/lib:${LD_LIBRARY_PATH:-}"

ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

无图形界面运行：

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false
```

启动后的最初几秒让 MID360 保持静止，以便 IMU 初始化；随后缓慢移动并检查点云和里程计是否稳定。

## 7. 检查 FAST-LIO 输出

```bash
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic hz /cloud_registered_body
ros2 topic hz /Laser_map
ros2 topic echo /fast_lio/gravity --once
ros2 topic echo /Odometry --once
```

独立启动时，`src/FAST_LIO/config/mid360.yaml` 的 `publish.map_en` 默认是 `true`。若只需里程计和当前帧点云，可改为 `false`，同时停止历史点云的累计与 `/Laser_map` 发布；此时无需执行该话题的 `ros2 topic hz`，也无法通过 `/map_save` 保存完整累计地图。需要存图时，`publish.map_en` 和 `pcd_save.pcd_save_en` 都必须为 `true`。

主要输出：

| 话题 | 含义 |
|---|---|
| `/Odometry` | FAST-LIO 实时位姿 |
| `/path` | 历史轨迹 |
| `/cloud_registered` | 世界坐标系下的当前配准点云 |
| `/cloud_registered_body` | IMU/机体坐标系下的点云 |
| `/Laser_map` | 累积点云地图 |
| `/fast_lio/gravity` | FAST-LIO 估计的世界系重力向量，`geometry_msgs/msg/Vector3Stamped` |
| `/tf` | `camera_init -> body` 动态变换 |

`/livox/lidar` 和 `/livox/imu` 是驱动提供给 FAST-LIO 的输入，不是 FAST-LIO 输出。

## 8. 保存地图

当前 `mid360.yaml` 已启用地图保存。建议先把 `map_file_path` 改成绝对路径，例如：

```yaml
map_file_path: "/home/jinghao/Nav3_Sensor_ws/maps/mid360_map.pcd"
```

创建目录：

```bash
mkdir -p /home/jinghao/Nav3_Sensor_ws/maps
```

运行 FAST-LIO 后调用：

```bash
source /opt/ros/humble/setup.bash
source /home/jinghao/Nav3_Sensor_ws/install/setup.bash
export LD_LIBRARY_PATH="/home/jinghao/Nav3_Sensor_ws/external/lib:${LD_LIBRARY_PATH:-}"

ros2 service call /map_save std_srvs/srv/Trigger "{}"
ls -lh /home/jinghao/Nav3_Sensor_ws/maps/mid360_map.pcd
```

服务返回 `success: true` 表示保存成功。

## 9. 日常最短启动流程

```bash
# 接好雷达后启用有线连接
sudo nmcli connection up livox-mid360
ping -c 4 192.168.1.134
```

终端1：

```bash
source /opt/ros/humble/setup.bash
source /home/jinghao/Nav3_Sensor_ws/install/setup.bash
export LD_LIBRARY_PATH="/home/jinghao/Nav3_Sensor_ws/external/lib:${LD_LIBRARY_PATH:-}"
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

终端2：

```bash
source /opt/ros/humble/setup.bash
source /home/jinghao/Nav3_Sensor_ws/install/setup.bash
export LD_LIBRARY_PATH="/home/jinghao/Nav3_Sensor_ws/external/lib:${LD_LIBRARY_PATH:-}"
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

## 10. 常见问题

### 构建时找不到 `livox_ros_driver2`

确认三个包都在当前工作空间：

```bash
cd /home/jinghao/Nav3_Sensor_ws
colcon list
```

应同时列出 `livox_ros_driver2`、`fast_lio` 和 `mid360_bringup`。构建前只加载 `/opt/ros/humble/setup.bash`。

### 驱动启动但没有点云

```bash
ip -br -4 addr show eno2
ping -c 4 192.168.1.134
ros2 topic list -t | grep livox
```

重点检查雷达供电、网线、主机 IP、MID360 IP 和 JSON 配置是否一致。

### FAST-LIO 启动但没有里程计

确认以下两个输入都在持续更新：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

同时检查驱动是否使用 `msg_MID360_launch.py`，不要使用输出普通 `PointCloud2` 的 RViz 驱动启动文件。

### 点云重影或轨迹漂移

检查：

1. 启动时是否保持静止完成 IMU 初始化；
2. `mid360.yaml` 中时间同步参数是否正确；
3. LiDAR—IMU 外参是否符合实际安装；
4. 雷达是否松动或振动过大。

## 11. 统一组合启动包（mid360_bringup）

组合启动包同时运行驱动、FAST-LIO、静态 TF、健康监控、局部地图和 RViz。它向独立的 KISS 定位工作空间提供里程计、重力和长/短窗口点云；需要时可启用关键帧候选发布者。首次使用前先按第 4 节构建三个包。KISS 环境加载顺序、候选库和数据接口见 [mid360_bringup 使用说明](src/mid360_bringup/README.md)。

### 启动建图（终端 1）

接好雷达并在终端 1 执行：

```bash
cd /home/jinghao/Nav3_Sensor_ws
sudo nmcli connection up livox-mid360
ping -c 4 192.168.1.134
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
ros2 launch mid360_bringup mid360_mapping.launch.py
```

启动后保持雷达静止 5～10 秒，待 IMU 初始化完成再移动。

### 保存地图（终端 2）

保持终端 1 运行，在新的终端 2 执行：

```bash
cd /home/jinghao/Nav3_Sensor_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
ros2 run mid360_bringup save_map
```

保存成功后会打印 PCD 文件的绝对路径，再停止终端 1。

## 2026-11-23 双路初始化实验工具

本分支增加[独立实验入口](tools/mid360_lidar/README.md)，保留彦男的一键录包、BBS/实采库双路初始化、先验跟踪、可回滚自动补库及实验出图工具。原 `src/mid360_bringup` 和标准联合导航入口保持原有行为；每轮只选一种启动入口。此工具须配套 Localization 的同名同步分支及另行传输的地图/库/BBS数据。隔离录制窗口测试不能替代 Go2 实机验收。
