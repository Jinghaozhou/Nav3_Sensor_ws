# mid360_bringup：MID360 统一建图入口

`mid360_bringup` 提供 Livox MID360 的建图入口：一次启动雷达驱动、FAST-LIO、静态 TF、健康监控、滑动窗口局部地图和 RViz，并通过命令保存 PCD 地图。建图使用雷达点云与 IMU 数据，输出里程计、轨迹和点云地图。本包不提供回环优化、全局重定位或导航控制。

适用环境：Ubuntu 22.04、ROS 2 Humble、Livox MID360。所有终端使用系统 Python，先退出 conda 环境。运行前需要同一工作空间中的 `livox_ros_driver2`、`fast_lio` 和 Livox SDK2。

## 1. 代码结构

```text
mid360_bringup/
├── package.xml                  ROS 依赖声明
├── setup.py / setup.cfg         Python 包与可执行程序安装
├── launch/
│   └── mid360_mapping.launch.py 组合启动文件
├── config/
│   ├── MID360_config.json       雷达网络与设备配置
│   └── mid360_robot.yaml        建图参数与组合启动话题
├── rviz/fastlio_map.rviz        RViz 显示配置
└── mid360_bringup/
    ├── livox_monitor.py         频率、CPU 和内存监控
    ├── local_map_window.py      滑动窗口局部地图
    └── save_map.py              PCD 存图客户端
```

`save_map` 将当前地图保存为 PCD 文件。PLY 转换、地图对齐和离线分析不属于本包的运行流程。

## 2. 构建方法

首次安装依赖：

```bash
source /opt/ros/humble/setup.bash
cd /home/jinghao/Fast_lio_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

若 rosdep 尚未初始化，先执行 `sudo rosdep init` 和 `rosdep update --rosdistro humble`。SDK2 必须提前安装到 `external`，方法见工作空间根 README。

打开新终端，加载 ROS 2 Humble 并在工作空间根目录构建：

```bash
source /opt/ros/humble/setup.bash
cd /home/jinghao/Fast_lio_ws
CMAKE_BUILD_PARALLEL_LEVEL=4 colcon build \
  --symlink-install --packages-select livox_ros_driver2 fast_lio mid360_bringup \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
export LD_LIBRARY_PATH="/home/jinghao/Fast_lio_ws/external/lib:${LD_LIBRARY_PATH:-}"
ros2 pkg prefix mid360_bringup
ros2 pkg executables mid360_bringup
```

Python 运行依赖包括 NumPy、SciPy 和 PyYAML，均已在 `package.xml` 声明。修改 C++ 后需重新构建对应包；新增 Python 文件、入口或资源后需重新构建本包。修改配置后建议重新构建本包，或通过启动参数直接指定源码配置文件，避免读取旧安装副本。

## 3. 使用方法

默认启动驱动、FAST-LIO、静态 TF、健康监控、局部地图窗口和 RViz：

```bash
cd /home/jinghao/Fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
ros2 launch mid360_bringup mid360_mapping.launch.py
```

无图形界面：

```bash
cd /home/jinghao/Fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
ros2 launch mid360_bringup mid360_mapping.launch.py rviz:=false
```

默认网络配置位于 `config/MID360_config.json`：主机地址为 `192.168.1.5`，雷达地址为 `192.168.1.106`。不同设备可通过 `driver_config` 指定另一份 JSON。主机有线接口须配置为 `192.168.1.5/24`，网线、供电和雷达 IP 须匹配。

启动后保持静止 5～10 秒完成 IMU 初始化，再缓慢移动。停止前在另一个终端保存地图：

```bash
cd /home/jinghao/Fast_lio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
ros2 run mid360_bringup save_map
```

服务使用 `/map_save`，地图文件为 `map_output_dir/mid360_map_启动时间.pcd`。一次启动对应一个文件，多次保存同一会话会覆盖该文件；重新启动会生成新名称。文件路径由 launch 设置为绝对路径，运行目录不影响输出。目录在启动时自动创建。

## 4. 数据接口

以下是 `mid360_robot.yaml` 的默认话题名及组合启动使用的坐标系。话题名可在该 YAML 的 `mid360_bringup.ros__parameters.topics` 中统一修改。

| 接口 | 类型 | 预期频率 | frame / 内容 |
|---|---|---|---|
| `/lidar_points` | livox_ros_driver2/msg/CustomMsg | 10 Hz | `lidar`，含逐点时间信息的原始点云 |
| `/imu` | sensor_msgs/msg/Imu | 约 200 Hz | `imu_link` |
| `/fast_lio/odom` | nav_msgs/msg/Odometry | 约 10 Hz | header=`odom`，child=`base_link` |
| `/fast_lio/local_cloud` | sensor_msgs/msg/PointCloud2 | 约 10 Hz | `odom`，配准后的当前帧 |
| `/fast_lio/body_cloud` | sensor_msgs/msg/PointCloud2 | 约 10 Hz | `base_link`，机体系当前帧 |
| `/fast_lio/effect_cloud` | sensor_msgs/msg/PointCloud2 | 按配置 | `odom`，默认不发布有效特征点 |
| `/Laser_map` | sensor_msgs/msg/PointCloud2 | 默认约 1 Hz；可关闭 | `odom`，从本次运行开始按周期累积的点云 |
| `/path` | nav_msgs/msg/Path | 算法内部抽样 | `odom`，历史轨迹 |
| `/fast_lio/local_map` | sensor_msgs/msg/PointCloud2 | 默认 2 Hz | `odom`，滑动窗口累计点云 |
| `/fast_lio/health` | std_msgs/msg/Float32MultiArray | 1 Hz（墙钟） | 见下面字段定义 |
| `/map_save` | std_srvs/srv/Trigger | 按需 | 保存当前累计地图；返回成功与实际路径 |

健康数组：`[lidar_hz, imu_hz, odom_hz, healthy, rss_mb, cpu_pct]`。默认健康阈值为 8/150/8 Hz 且 `fastlio_mapping` 进程存在。健康状态衡量链路频率和进程存活，不判断位姿漂移或滤波器发散。监控使用墙钟，bag 暂停时会显示频率下降；CPU 百分比可因多核超过 100%。默认假定只运行一个 Fast-LIO 实例。

算法点云和里程计采用默认可靠 QoS；监控输入采用可靠、深度 1000，以免浅队列漏计 IMU；局部地图输入采用 BEST_EFFORT，可订阅可靠发布者；局部地图输出为可靠、深度 10。局部地图是时间窗累计点云，不执行动态物体剔除；输入停止后仍保留并发布最后窗口，其时间戳保持最后输入时间，下游应检查数据新鲜度。`/Laser_map` 每次发送的都是截至当前的累计点云，运行越久消息越大；关闭累计与发布后，仍有 FAST-LIO 用于位姿估计的内部地图，但不再保留供 `/map_save` 使用的历史累计点云。

### TF 和 frame 定义

```text
odom
└── base_link       FAST-LIO 发布动态 TF
    ├── imu_link    本包发布静态恒等 TF
    └── lidar       本包根据同一 YAML 的 extrinsic_T/R 发布静态 TF
```

- `odom`：算法启动时建立的局部世界系，无绝对位置和航向参考。
- `base_link`：本包当前定义为 IMU 本体系，不能直接解释为任意机器人底盘原点。
- `imu_link`：与 `base_link` 同原点同轴的测量 frame。
- `lidar`：雷达测量 frame；`p_base = extrinsic_R * p_lidar + extrinsic_T`。
- 点云、轨迹、里程计及 TF 使用同一组 `odom/base_link` 参数。
- 本包不发布 `map→odom`。该变换留给全局定位模块。

如机器人底盘原点与 IMU 不重合，需根据安装外参换算底盘位姿和 TF，不能仅将 IMU 位姿改名为底盘位姿。

## 5. 配置文件与启动参数

`config/` 中的两份文件分别交给不同组件使用：`mid360_robot.yaml` 提供 FAST-LIO 建图参数和组合启动的话题名；`MID360_config.json` 提供 Livox 驱动连接雷达所需的网络与设备参数。组合启动默认读取本包安装目录中的这两份文件，可分别用 `config_file` 和 `driver_config` 指向自定义副本。`src/FAST_LIO/config/mid360.yaml` 是单独启动 FAST-LIO 时的默认配置，修改它不会改变组合启动读取的 YAML。

### 启动参数

查看全部参数：

```bash
ros2 launch mid360_bringup mid360_mapping.launch.py --show-args
```

| 参数 | 默认值 | 用途 |
|---|---|---|
| `driver` | true | false 时不启动硬件驱动 |
| `use_sim_time` | false | 回放时 true；此时必须 driver=false |
| `rviz` / `monitor` / `local_map` | true | 分别控制可视化、监控、局部地图 |
| `config_file` | 包内 mid360_robot.yaml | 建图参数和话题接口配置路径 |
| `driver_config` | 包内 MID360_config.json | 网络配置路径 |
| `rviz_config` | 包内 fastlio_map.rviz | RViz 配置路径 |
| `map_output_dir` | ~/Fast_lio_ws/maps | 可写地图输出目录 |
| `window_time` | 5.0 | 局部地图时间窗，秒 |
| `window_frames` | 100 | 局部地图最多帧数 |
| `local_map_voxel` | 0.15 | 局部地图抽稀体素，米 |
| `local_map_rate` | 2.0 | 局部地图发布 Hz |

路径支持绝对路径或相对当前终端工作目录的路径。需要修改配置时，推荐复制到用户自己的配置目录，再通过参数指定，避免修改安装目录。

### 话题接口 YAML

`config/mid360_robot.yaml` 包含两段：`/**.ros__parameters` 是 FAST-LIO 参数；`mid360_bringup.ros__parameters.topics` 由组合启动文件读取，用于统一设置话题名。后者的默认值是：

```yaml
mid360_bringup:
  ros__parameters:
    topics:
      lidar: "/lidar_points"
      imu: "/imu"
      odom: "/fast_lio/odom"
      local_cloud: "/fast_lio/local_cloud"
      body_cloud: "/fast_lio/body_cloud"
      effect_cloud: "/fast_lio/effect_cloud"
      laser_map: "/Laser_map"
      path: "/path"
      local_map: "/fast_lio/local_map"
      health: "/fast_lio/health"
```

修改右侧话题名后，启动文件会同步配置驱动输出、FAST-LIO 输入和输出、监控订阅、局部地图以及 RViz 的订阅重映射。名称须是互不重复的绝对 ROS 话题名，例如 `/robot/odom`。`/map_save` 是固定的存图服务名；`odom`、`base_link`、`imu_link` 和 `lidar` 是 TF frame 名，不在此处修改。使用 `config_file:=/绝对路径/自定义.yaml` 时，自定义文件也须包含完整的 `topics` 段。这里的话题配置只对组合启动生效。

### Fast-LIO YAML

`mid360_robot.yaml` 中 `/**: ros__parameters:` 下可调整的建图设置如下。修改浮点数组时保持浮点类型。

| 参数 | 包内默认值 | 作用 |
|---|---|---|
| `feature_extract_enable` | false | 是否启用特征提取 |
| `point_filter_num` / `max_iteration` | 2 / 3 | 原始点抽样间隔 / 每帧状态估计迭代上限 |
| `filter_size_surf` / `filter_size_map` / `cube_side_length` | 0.2 / 0.1 / 1000.0 | 点云与内部地图的体素大小、局部地图立方体边长，单位米 |
| `common.time_sync_en` / `common.time_offset_lidar_to_imu` | false / 0.0 | 点云与 IMU 的时间同步及时间偏移；无准确标定值时保持默认 |
| `preprocess.lidar_type` / `scan_line` / `timestamp_unit` / `scan_rate` | 1 / 4 / 3 / 10 | 雷达数据格式、扫描线、逐点时间单位与扫描频率；当前值对应 MID360 数据 |
| `preprocess.blind` | 0.5 | 剔除雷达近处点云的距离，米 |
| `mapping.acc_cov` / `gyr_cov` / `b_acc_cov` / `b_gyr_cov` | 0.1 / 0.1 / 0.0001 / 0.0001 | IMU 测量和偏置的噪声参数 |
| `mapping.fov_degree` / `det_range` | 360.0 / 50.0 | 视场角与建图处理距离 |
| `mapping.extrinsic_est_en` | false | 统一入口要求固定外参 |
| `mapping.extrinsic_T/R` | 文件内的平移和单位旋转矩阵 | 雷达到 IMU 的外参；算法与静态 TF 共用 |
| `runtime_pos_log_enable` | false | 运行日志开关 |
| `publish.path_en` / `effect_map_en` | true / false | 历史轨迹 / 有效特征点云的发布开关 |
| `publish.map_en` | true | 是否累计点云并向 `/Laser_map` 发布；关闭后无法保存累计地图 |
| `publish.scan_publish_en` / `dense_publish_en` / `scan_bodyframe_pub_en` | true / false / true | 当前帧点云发布、点云密度、机体系当前帧发布设置 |
| `pcd_save.pcd_save_en` | true | 是否允许通过 `/map_save` 存图；还需 `publish.map_en=true` |

只需里程计和当前帧配准点云、无需完整累计地图时，可将 `publish.map_en` 设为 `false` 并重启组合启动；这会同时停止历史点云的累计与发布，`/map_save` 将没有可保存的数据。需要存图时保持 `publish.map_en` 和 `pcd_save.pcd_save_en` 都为 `true`。`pcd_save.interval` 是原代码保留的分段存图字段，当前 `/map_save` 服务不使用它；存图文件名及目录由启动参数 `map_output_dir` 决定。

组合启动会根据 `topics.lidar/imu` 覆盖 YAML 中的 `common.lid_topic/imu_topic`，并设置 `world_frame=odom`、`body_frame=base_link` 和带启动时间的 `map_file_path`，因此组合启动时不要通过修改这几个 YAML 字段调整输入话题、frame 或存图路径。`extrinsic_R` 必须是合法的 3×3 旋转矩阵，启动时校验并转换为四元数；启动文件要求 `extrinsic_est_en=false`，确保建图计算和静态 TF 使用同一组固定外参。

### 网络 JSON

`config/MID360_config.json` 由 Livox 驱动读取，控制网络连接和设备设置，不控制 ROS 话题名或 FAST-LIO 建图参数。本包的默认主机 IP 是 `192.168.1.5`，雷达 IP 是 `192.168.1.106`。

| JSON 字段 | 当前值 | 作用 |
|---|---|---|
| `lidar_summary_info.lidar_type` | 8 | 驱动配置使用的雷达类别标识；本 MID360 配置保持该值 |
| `MID360.lidar_net_info` | 56100～56500 | 雷达端的命令、推送、点云、IMU 和日志端口 |
| `MID360.host_net_info.*_ip` | cmd/push/point/imu 均为 `192.168.1.5`；日志 IP 为空 | 主机接收雷达数据的网卡地址 |
| `MID360.host_net_info.*_port` | 56101～56501 | 主机端对应的 UDP 端口 |
| `lidar_configs[].ip` | `192.168.1.106` | 实际雷达的 IP 地址 |
| `lidar_configs[].pcl_data_type` / `pattern_mode` | 1 / 0 | 雷达点云数据格式和扫描模式；ROS 输出话题名仍由 YAML 配置 |
| `lidar_configs[].extrinsic_parameter` | 全部为 0 | 驱动侧外参；本包保持零值，雷达到 IMU 的变换在 YAML 中配置 |

更换主机网卡 IP 时，需同时修改 `host_net_info` 中 cmd/push/point/imu 的 `*_ip`，并把实际网卡配置到该地址；更换雷达时修改 `lidar_configs[].ip`。端口及雷达类型只有在设备网络配置也相应改变时才需要调整。两份配置可分别通过以下启动参数指定，修改后重新启动生效：

```bash
ros2 launch mid360_bringup mid360_mapping.launch.py \
  config_file:=/绝对路径/mid360_robot.yaml \
  driver_config:=/绝对路径/MID360_config.json
```

## 6. 标准运行示例

以下示例使用本机已有的 `livox-mid360` 网口连接，主机 `192.168.1.5/24`、雷达 `192.168.1.106`。

**终端 1：检查网络并启动建图**

```bash
sudo nmcli connection up livox-mid360
ping -c 4 192.168.1.106
source /opt/ros/humble/setup.bash
source /home/jinghao/Fast_lio_ws/install/setup.bash
export LD_LIBRARY_PATH="/home/jinghao/Fast_lio_ws/external/lib:${LD_LIBRARY_PATH:-}"
ros2 launch mid360_bringup mid360_mapping.launch.py \
  map_output_dir:=/home/jinghao/Fast_lio_ws/maps
```

保持静止完成初始化，随后缓慢移动，RViz 中观察地图和轨迹。

**终端 2：检查接口并可选录制**

```bash
source /opt/ros/humble/setup.bash
source /home/jinghao/Fast_lio_ws/install/setup.bash
ros2 topic list -t
ros2 topic echo /fast_lio/odom --once
ros2 run tf2_ros tf2_echo odom base_link
# 上一命令检查后 Ctrl-C，再录制两个输入话题
ros2 bag record /lidar_points /imu -o /tmp/mid360_scan
```

预期里程计 header=`odom`、child=`base_link`，TF 连通，点云和 IMU 都持续更新。停止录制后用 `ros2 bag info /tmp/mid360_scan` 确认两种数据均有消息。

**终端 3：运行期间存图**

```bash
source /opt/ros/humble/setup.bash
source /home/jinghao/Fast_lio_ws/install/setup.bash
ros2 run mid360_bringup save_map
```

成功时打印 PCD 绝对路径，再停止终端 1。存图工具服务等待 5 秒、响应默认等待 60 秒，可通过 `--timeout 120` 增加响应等待时间。

### 离线回放

先停止实时建图和驱动。加载上述环境后：

```bash
# 终端 1
ros2 launch mid360_bringup mid360_mapping.launch.py \
  driver:=false use_sim_time:=true
# 终端 2
ros2 bag play /tmp/mid360_scan --clock 100
```

录制文件的话题名为 `/livox/lidar` 和 `/livox/imu` 时，可在回放时重映射：

```bash
ros2 bag play /path/to/old_bag --clock 100 \
  --remap /livox/lidar:=/lidar_points /livox/imu:=/imu
```

回放结束后保持建图节点运行并调用存图。话题重映射不会修改消息中的 `header.frame_id`；如其他节点要求 `lidar` 和 `imu_link`，请检查录制数据的实际 frame。录制文件须同时包含点云和 IMU，才能用于建图。

## 7. 排查

- 没有点云：检查供电、网口、ping、包内 JSON 或 `driver_config` 指向的文件。
- RViz 没图：检查 `/fast_lio/odom`、`/fast_lio/local_cloud` 和 `odom→base_link` TF。
- frame 不是 `odom/base_link`：确认 `ros2 pkg prefix fast_lio` 指向当前工作空间，并重新构建相应包。
- 监控异常：检查三种输入频率；bag 慢放、暂停可能触发阈值。
- 存图失败：确认 `publish.map_en` 和 `pcd_save.pcd_save_en` 都已启用、累计点云已有数据、输出目录可写。服务返回成功后工具还会检查文件存在及非空。
- 只启动一套雷达驱动与 FAST-LIO；重复实例会争用 UDP 端口和产生重复 TF。
