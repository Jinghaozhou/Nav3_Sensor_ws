#!/bin/bash
# Docker SLAM 启动助手（详见 README 第 9 节）
#
# 用法:
#   scripts/run_slam_docker.sh fast_lio2    [数据目录]   # 纯激光惯性建图
#   scripts/run_slam_docker.sh funny_lidar  [数据目录]   # 建图 + 回环 + 先验地图重定位
#
# 默认把 <数据目录>(默认 ./slam_data)挂载到容器 /root/data、
# <数据目录>/../slam_output 挂载到 /root/out(PCD 输出目录)。
#
# Mid-360 数据(ROS2 CustomMsg)先用 ros2bag_to_ros1bag.py 转成 ROS1 bag
# (CustomMsg 类型已支持),本脚本主要跑 MetaCam 教程的标准 ROS1 bag。
set -euo pipefail

IMAGE_KEY=${1:-fast_lio2}
DATA_DIR=${2:-./slam_data}

case "$IMAGE_KEY" in
    fast_lio2)
        TAG="registry.cn-shanghai.aliyuncs.com/slamibot/fast_lio2:3.1"
        RUN_CMD="bash /run.sh"
        ;;
    funny_lidar)
        TAG="registry.cn-shanghai.aliyuncs.com/slamibot/funny_lidar_slam:3.1"
        RUN_CMD="bash -c 'cd /root/funny_lidar_slam_ws && ./mapping.sh'"
        ;;
    *) echo "用法: $0 {fast_lio2|funny_lidar} [数据目录]"; exit 1 ;;
esac

mkdir -p "$DATA_DIR"
OUT_DIR="$(dirname "$DATA_DIR")/slam_output"
mkdir -p "$OUT_DIR"

docker pull "$TAG" > /dev/null 2>&1 || true

cat <<EOF

容器启动后，按 README 第 9 节操作：
  容器内终端 1：$RUN_CMD
  容器内终端 2：docker exec -it <容器名> bash
                rosbag play /root/data/你的数据_ros1.bag [话题重映射]
  结果：PCD 输出到宿主机的 $OUT_DIR
EOF

exec docker run --rm --net=host \
    -v "$(realpath "$DATA_DIR")":/root/data \
    -v "$(realpath "$OUT_DIR")":/root/out \
    -it "$TAG" /bin/bash
