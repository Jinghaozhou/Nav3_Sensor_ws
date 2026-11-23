#!/usr/bin/env bash
# Skylandx-C01 → Fast-LIO2 依赖构建脚本（在任意 Ubuntu 22.04 + ROS 2 Humble 电脑上运行）
# 用法: bash setup_fastlio.sh [工作区根目录，默认 ~/]
# 产物: ~/livox_ws（Livox SDK + 驱动，仅编译依赖）、~/lio_ws（FAST_LIO_ROS2）
# 需要 sudo 的部分只有: apt 依赖 + Livox-SDK2 make install（脚本会暂停提示）
set -e

WS_ROOT="${1:-$HOME}"
LIVOX_WS="$WS_ROOT/livox_ws"
LIO_WS="$WS_ROOT/lio_ws"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PATH=/usr/bin:/bin:/usr/local/bin   # 避开 conda，用系统 python3.10

echo "=== [1/6] 系统依赖 ==="
sudo apt install -y libpcl-dev libeigen3-dev python3-colcon-common-extensions \
    ros-humble-pcl-ros ros-humble-sensor-msgs-py
# rosdep 初始化（新机器可能未做过）
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
    rosdep update || true
fi

echo "=== [2/6] 克隆源码（gitcode 镜像，GitHub 直连不通时可用） ==="
mkdir -p "$LIVOX_WS/src" "$LIO_WS/src"
cd "$LIVOX_WS/src"
[ -d Livox-SDK2 ] || git clone --depth 1 https://gitcode.com/gh_mirrors/li/Livox-SDK2.git
[ -d livox_ros_driver2 ] || git clone --depth 1 https://gitcode.com/gh_mirrors/li/livox_ros_driver2.git
# 本工程补丁:IMU frame_id 参数化(设计文档要求 /imu frame = imu_link);已应用则跳过
cd "$LIVOX_WS/src/livox_ros_driver2"
if git apply --check "$SCRIPT_DIR/scripts/livox_driver_imu_frame.patch" 2>/dev/null; then
    git apply "$SCRIPT_DIR/scripts/livox_driver_imu_frame.patch"
    echo ">>> livox_ros_driver2 已打 imu_frame_id 补丁"
else
    echo ">>> livox_ros_driver2 imu_frame_id 补丁已应用(跳过)"
fi
cd "$LIO_WS/src"
if [ ! -d FAST_LIO_ROS2 ]; then
    git clone -b ros2 --depth 1 https://gitcode.com/gh_mirrors/fa/FAST_LIO_ROS2.git
    cd FAST_LIO_ROS2
    git submodule init
    git config submodule.include/ikd-Tree.url https://gitcode.com/gh_mirrors/ik/ikd-Tree.git
    git submodule update --init --depth 1 || \
        git submodule update --init
fi
# 本工程补丁:det_range 远距过滤(恢复原版语义,该移植版丢失);已应用则跳过
cd "$LIO_WS/src/FAST_LIO_ROS2"
if git apply --check "$SCRIPT_DIR/scripts/fastlio_maxrange.patch" 2>/dev/null; then
    git apply "$SCRIPT_DIR/scripts/fastlio_maxrange.patch"
    echo ">>> FAST_LIO_ROS2 已打 max_range 补丁"
else
    echo ">>> FAST_LIO_ROS2 max_range 补丁已应用(跳过)"
fi

echo "=== [3/6] 编译 Livox-SDK2 ==="
cd "$LIVOX_WS/src/Livox-SDK2"
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"
echo ">>> 即将安装 Livox-SDK2 到 /usr/local（需要密码）"
sudo make install

echo "=== [4/6] 编译 livox_ros_driver2（仅作为 fast_lio 编译依赖，运行时不启动） ==="
source /opt/ros/humble/setup.bash
cd "$LIVOX_WS/src/livox_ros_driver2"
./build.sh humble

echo "=== [5/6] 编译 FAST_LIO_ROS2 ==="
cd "$LIO_WS"
source "$LIVOX_WS/install/setup.bash"
rosdep install --from-paths src --ignore-src -r -y || true
MAKEFLAGS="-j4" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "=== [6/6] 复制雷达配置到 install 空间 ==="
mkdir -p "$LIO_WS/install/fast_lio/share/fast_lio/config"
cp "$SCRIPT_DIR/mid360_robot.yaml" "$LIO_WS/install/fast_lio/share/fast_lio/config/"
cp "$SCRIPT_DIR/mid360_robot.yaml" "$LIO_WS/src/FAST_LIO_ROS2/config/"

echo ""
echo "=== 完成 ==="
echo "启动 Fast-LIO2:"
echo "  source $LIO_WS/install/setup.bash"
echo "  ros2 launch fast_lio mapping.launch.py config_file:=mid360_robot.yaml"
