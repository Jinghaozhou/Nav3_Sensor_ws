"""ROS2 bag → ROS1 bag 转换（供 MetaCam 教程的 Docker SLAM 镜像离线处理）。

PDF 教程第 3 章所有 SLAM 镜像的输入都是 ROS1 rosbag 格式；本脚本把
mid360_lidar 录制的 ROS2 bag 转成 ROS1 bag。

用法（conda 环境，已装 rosbags）:
    python3 ros2bag_to_ros1bag.py <ros2_bag目录> [-o 输出.bag]
    python3 ros2bag_to_ros1bag.py ~/人形/mid360_lidar/bags/场景名 \
        -o /tmp/场景名_ros1.bag

需要重命名的话题在下方 TOPIC_MAP 里配置（默认与 ROS1 侧同名，无需映射）。

livox CustomMsg 支持: 该类型不在 rosbags 标准 typestore 里，脚本启动时从
~/livox_ws/src/livox_ros_driver2/msg 读取消息定义(不存在则用内置副本)，
分别注册到 ROS2/ROS1 typestore；ROS1 侧注册为 livox_ros_driver 包名
(Docker 镜像里是 ROS1 版驱动，类型名/md5 与 ROS1 版一致)。

实现说明: 逐条消息用 ROS2 typestore 反序列化 → 给 header 补 ROS1 才有的
seq=0 → 用 ROS1 typestore 序列化写入。
"""

import argparse
import sys
from pathlib import Path

from rosbags.rosbag1 import Writer
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

# 需要重命名的话题写这里。
# 一键 launch 录的 bag 用设计文档话题名,而 Docker ROS1 侧 livox 驱动
# 订阅 /livox/*,转换时改回(单独跑驱动录的 bag 本就是 /livox/*,不受影响)
TOPIC_MAP = {
    '/lidar_points': '/livox/lidar',
    '/imu': '/livox/imu',
}

# ROS2 bag 消息类型 → ROS1 bag 消息类型(Docker 镜像用 ROS1 版 livox_ros_driver)
TYPE_MAP = {
    'livox_ros_driver2/msg/CustomMsg': 'livox_ros_driver/msg/CustomMsg',
}

# 内置消息定义副本(~/livox_ws 不存在时兜底,与 livox_ros_driver2 源码一致)
CUSTOMMSG_TEXT = """\
# Livox publish pointcloud msg format.

std_msgs/Header header    # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data
"""

CUSTOMPOINT_TEXT = """\
# Livox costom pointcloud format.

uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar
"""


def _load_livox_msg_texts():
    """优先读 ~/livox_ws 驱动源码里的定义,不存在则用内置副本。"""
    msg_dir = Path.home() / 'livox_ws/src/livox_ros_driver2/msg'
    cm = msg_dir / 'CustomMsg.msg'
    cp = msg_dir / 'CustomPoint.msg'
    if cm.is_file() and cp.is_file():
        return cm.read_text(), cp.read_text()
    return CUSTOMMSG_TEXT, CUSTOMPOINT_TEXT


def register_livox_types(ts, pkg):
    """把 CustomMsg/CustomPoint 注册进 typestore(按给定包名)。"""
    cm_text, cp_text = _load_livox_msg_texts()
    types = get_types_from_msg(cp_text, f'{pkg}/msg/CustomPoint')
    types.update(get_types_from_msg(cm_text, f'{pkg}/msg/CustomMsg'))
    ts.register(types)


def ros2_to_ros1_message(msg):
    """补 header.seq（ROS1 专有字段），返回同一对象。"""
    header = getattr(msg, 'header', None)
    if header is not None and not hasattr(header, 'seq'):
        header.seq = 0
    return msg


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('ros2_bag', help='ROS2 bag 目录（含 metadata.yaml）')
    ap.add_argument('-o', '--output', default=None,
                    help='输出 ROS1 bag 路径（默认 <ros2_bag>.bag）')
    args = ap.parse_args()

    out = args.output or args.ros2_bag.rstrip('/') + '.bag'
    ts2 = get_typestore(Stores.ROS2_HUMBLE)
    ts1 = get_typestore(Stores.ROS1_NOETIC)
    # livox CustomMsg 不在标准 typestore,两侧都注册(ROS2 侧用于反序列化,
    # ROS1 侧用于写入;ROS1 侧的 Header 自带 seq,md5 与 ROS1 版驱动一致)
    register_livox_types(ts2, 'livox_ros_driver2')
    register_livox_types(ts1, 'livox_ros_driver')

    with Reader(args.ros2_bag) as reader, Writer(out) as writer:
        conns = {}
        for conn in reader.connections:
            topic = TOPIC_MAP.get(conn.topic, conn.topic)
            ros1_type = TYPE_MAP.get(conn.msgtype, conn.msgtype)
            new_conn = writer.add_connection(
                topic, ros1_type, typestore=ts1)
            conns[conn.id] = new_conn
            print(f'  {conn.topic} [{conn.msgtype}] → {topic} '
                  f'[{ros1_type}] ({conn.msgcount} 条)')
        total = 0
        for conn, timestamp, rawdata in reader.messages():
            ros1_type = TYPE_MAP.get(conn.msgtype, conn.msgtype)
            msg = ts2.deserialize_cdr(rawdata, conn.msgtype)
            raw1 = ts1.serialize_ros1(
                ros2_to_ros1_message(msg), ros1_type)
            writer.write(conns[conn.id], timestamp, raw1)
            total += 1
    print(f'完成: {total} 条消息 → {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
