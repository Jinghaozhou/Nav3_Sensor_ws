#!/bin/bash
# 调 /map_save 服务保存 Fast-LIO2 地图,并自动生成同名 .ply(适配 apt 版 CloudCompare)。
#
# 用法(任意目录,fast_lio 还在运行时):
#     scripts/save_map.sh
#
# 输出: slam_output/mid360_map_YYYYMMDD_HHMMSS.pcd + 同名 .ply(原始),
#       以及 _aligned.pcd/.ply(地面找平 + 墙向对齐后的横平竖直版)
# 注意: 不用 set -u —— ROS 的 setup.bash 系列会引用未声明的 AMENT_* 变量
set -eo pipefail

SKYLAND_DIR="$(cd "$(dirname "$0")/.." && pwd)"
# ROS setup.bash 用到了未声明的 AMENT_TRACE_SETUP_FILES,先显式置空
AMENT_TRACE_SETUP_FILES="" source /opt/ros/humble/setup.bash

echo ">>> 调 /map_save 服务..."
RESP="$(ros2 service call /map_save std_srvs/srv/Trigger 2>/dev/null)" || true
echo "$RESP"

if ! echo "$RESP" | grep -q "success=True"; then
    echo ">>> 保存失败(地图为空或 fast_lio 未启动)"; exit 1
fi

FNAME="$(echo "$RESP" | grep -oE "Map saved to [^']+" | sed 's/Map saved to //' | tr -d '"')"
PCD="$SKYLAND_DIR/${FNAME#./}"
[ -f "$PCD" ] || { echo ">>> 找不到 $PCD"; exit 1; }

echo ">>> 生成 PLY..."
/home/pc/anaconda3/bin/python3 "$SKYLAND_DIR/scripts/pcd2ply.py" "$PCD" -o "${PCD%.pcd}.ply"

echo ">>> 对齐(地面找平 + 墙向对齐,输出 _aligned 横平竖直版)..."
/home/pc/anaconda3/bin/python3 "$SKYLAND_DIR/scripts/align_map.py" "$PCD"

echo ">>> 完成:"
echo "    PCD: $PCD"
echo "    PLY: ${PCD%.pcd}.ply(原始版,拖进 CloudCompare)"
echo "    PCD: ${PCD%.pcd}_aligned.pcd(横平竖直版)"
echo "    PLY: ${PCD%.pcd}_aligned.ply"
