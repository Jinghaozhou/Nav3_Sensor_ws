#!/usr/bin/env python3
"""FIFO local point-cloud window preserving input frame and intensity."""

import argparse
import sys
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField


def parse_cloud(msg):
    """PointCloud2(x y z intensity) → (xyz[N,3], intensity[N] 或 None)。"""
    endian = '>' if msg.is_bigendian else '<'
    fields = {f.name: f for f in msg.fields}
    def field(name):
        f = fields[name]
        if f.datatype != PointField.FLOAT32 or f.count != 1:
            raise ValueError('Expected scalar FLOAT32 field: ' + name)
        return np.ndarray((msg.height, msg.width), dtype=endian+'f4',
                          buffer=msg.data, offset=f.offset,
                          strides=(msg.row_step, msg.point_step)).reshape(-1).copy()
    xyz = np.column_stack([field(n) for n in ('x','y','z')])
    intensity = field('intensity') if 'intensity' in fields else None
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid], intensity[valid] if intensity is not None else None


def build_cloud(stamp, frame_id, xyz, intensity=None):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(xyz)
    msg.is_dense = False
    msg.point_step = 16
    msg.row_step = msg.width * 16
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    arr = np.zeros((len(xyz), 4), dtype=np.float32)
    arr[:, :3] = xyz
    if intensity is not None and len(intensity) == len(xyz):
        arr[:, 3] = intensity
    msg.data = arr.astype(np.float32).tobytes()
    return msg


def voxel_indices(xyz, voxel):
    if len(xyz) == 0:
        return np.empty(0, dtype=np.int64)
    keys = np.floor(xyz / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return np.sort(idx)


class LocalMapWindow(Node):
    def __init__(self, window_time, window_frames, voxel, rate,
                 in_topic, out_topic):
        super().__init__('local_map_window')
        self.frame_id = None
        self.window = deque()          # (Time, xyz, intensity)
        self.window_time = window_time
        self.window_frames = window_frames
        self.voxel = voxel
        qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, in_topic, self.on_cloud, qos)
        self.pub = self.create_publisher(PointCloud2, out_topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_window)
        self.get_logger().info(f'窗口 {window_time}s / {window_frames} 帧, '
                               f'体素 {voxel}m, {in_topic} → {out_topic}')

    def on_cloud(self, msg):
        if self.frame_id is not None and self.frame_id != msg.header.frame_id:
            self.get_logger().error('Input frame changed; clearing window')
            self.window.clear()
        self.frame_id = msg.header.frame_id
        xyz, intensity = parse_cloud(msg)
        stamp = Time.from_msg(msg.header.stamp)
        if self.window and stamp < self.window[-1][0]:
            # Bag seek/restart: discard frames from the previous timeline.
            self.window.clear()
        self.window.append((stamp, xyz, intensity))
        # FIFO:过期帧(超时间窗)从队头弹出
        while (self.window and
               (stamp - self.window[0][0]).nanoseconds / 1e9 > self.window_time):
            self.window.popleft()
        # 帧数上限:超了弹队头
        while len(self.window) > self.window_frames:
            self.window.popleft()

    def publish_window(self):
        if not self.window:
            return
        xyz = np.vstack([f[1] for f in self.window])
        it = np.concatenate([f[2] for f in self.window]) if all(f[2] is not None for f in self.window) else None
        idx = voxel_indices(xyz, self.voxel)
        msg = build_cloud(self.window[-1][0].to_msg(), self.frame_id, xyz[idx],
                          it[idx] if it is not None else None)
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--window-time', type=float, default=5.0,
                    help='时间窗长度秒(默认 5)')
    ap.add_argument('--window-frames', type=int, default=100,
                    help='帧数上限(默认 100)')
    ap.add_argument('--voxel', type=float, default=0.15,
                    help='发布前体素抽稀(m,默认 0.15)')
    ap.add_argument('--rate', type=float, default=2.0, help='发布频率(默认 2Hz)')
    ap.add_argument('--in-topic', default='/fast_lio/local_cloud')
    ap.add_argument('--out-topic', default='/fast_lio/local_map')
    args, ros_args = ap.parse_known_args()
    if min(args.window_time, args.window_frames, args.voxel, args.rate) <= 0:
        ap.error('Window, voxel and rate must be positive')

    rclpy.init(args=ros_args)
    node = LocalMapWindow(args.window_time, args.window_frames, args.voxel,
                          args.rate, args.in_topic, args.out_topic)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
