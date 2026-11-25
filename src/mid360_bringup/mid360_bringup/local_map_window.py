#!/usr/bin/env python3
"""Publish long and adaptive short point-cloud windows in the odom frame."""

import argparse
import json
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


def parse_cloud(msg):
    """Read XYZ and intensity using PointCloud2 offsets, padding and byte order."""
    endian = '>' if msg.is_bigendian else '<'
    formats = {PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8',
               PointField.UINT8: 'u1', PointField.UINT16: 'u2'}
    fields = {f.name: f for f in msg.fields}

    if msg.width * msg.height == 0:
        return np.empty((0, 3), np.float32), np.empty(0, np.float32)

    def channel(name):
        f = fields[name]
        if f.count != 1 or f.datatype not in formats:
            raise ValueError('Unsupported scalar PointCloud2 field: ' + name)
        return np.ndarray((msg.height, msg.width), dtype=endian + formats[f.datatype],
                          buffer=msg.data, offset=f.offset,
                          strides=(msg.row_step, msg.point_step)).reshape(-1)

    xyz = np.column_stack([channel(n) for n in ('x', 'y', 'z')]).astype(np.float32)
    intensity = (channel('intensity').astype(np.float32)
                 if 'intensity' in fields else np.zeros(len(xyz), np.float32))
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid], np.nan_to_num(intensity[valid], nan=0.0, posinf=0.0, neginf=0.0)


def build_cloud(stamp, frame_id, xyz, intensity=None):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(xyz)
    msg.is_dense = True
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.width * 16
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    arr = np.zeros((len(xyz), 4), dtype='<f4')
    arr[:, :3] = xyz
    if intensity is not None and len(intensity) == len(xyz):
        arr[:, 3] = intensity
    msg.data = arr.tobytes()
    return msg


def voxel_indices(xyz, voxel):
    if len(xyz) == 0:
        return np.empty(0, dtype=np.int64)
    keys = np.floor(xyz / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return np.sort(idx)


class LocalMapWindow(Node):
    def __init__(self, args):
        super().__init__('local_map_window')
        self.args = args
        self.window = deque()  # (sensor stamp, XYZ, intensity)
        self.retained_points = 0
        self.tracking = False
        self.failures = 0
        self.adaptive_enabled = False
        self.timing_csv = None
        if args.timing_csv:
            path = Path(args.timing_csv).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.timing_csv = path.open('x', encoding='utf-8', buffering=1)
            self.timing_csv.write('event,steady_ns,ros_ns,cloud_stamp_ns,points,reason\n')
            self.get_logger().info(f'Local-map timing CSV: {path}')
        input_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, args.in_topic, self.on_cloud, input_qos)
        self.local_pub = self.create_publisher(PointCloud2, args.out_topic, 1)
        self.global_pub = self.create_publisher(PointCloud2, args.global_topic, 1)
        self.status_pub = self.create_publisher(String, args.window_status_topic, 1)
        if args.adaptive:
            try:
                from kiss_matcher_localization.msg import LocalizationResult
            except ImportError:
                self.get_logger().warn('KISS LocalizationResult unavailable; using the long window only')
            else:
                status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.create_subscription(LocalizationResult, args.localization_status_topic,
                                         self.on_localization_status, status_qos)
                self.adaptive_enabled = True
        self.timer = self.create_timer(1.0 / args.rate, self.publish_window)
        self.get_logger().info(
            f'window={args.window_time}s tracking={args.tracking_window_time}s '
            f'frames={args.window_frames} points={args.max_buffer_points} '
            f'adaptive={self.adaptive_enabled}')

    def on_localization_status(self, result):
        if result.applied:
            self.tracking, self.failures = True, 0
        elif self.tracking:
            self.failures += 1
            if self.failures >= self.args.recovery_failures:
                self.tracking = False

    def on_cloud(self, msg):
        if msg.header.frame_id != 'odom':
            self.get_logger().error(f'Input frame {msg.header.frame_id!r}, expected odom')
            return
        xyz, intensity = parse_cloud(msg)
        if not len(xyz):
            return
        stamp = Time.from_msg(msg.header.stamp)
        if self.window and stamp.nanoseconds <= self.window[-1][0].nanoseconds:
            self.window.clear()
            self.retained_points = 0
            self.tracking, self.failures = False, 0
            self.get_logger().warn('Sensor time reset; cleared local-map window')
        idx = voxel_indices(xyz, self.args.voxel)
        xyz, intensity = xyz[idx], intensity[idx]
        if len(xyz) > self.args.max_buffer_points:
            step = int(np.ceil(len(xyz) / self.args.max_buffer_points))
            xyz, intensity = xyz[::step], intensity[::step]
        self.window.append((stamp, xyz, intensity))
        self.retained_points += len(xyz)
        while (len(self.window) > 1 and
               ((stamp - self.window[0][0]).nanoseconds / 1e9 > self.args.window_time or
                len(self.window) > self.args.window_frames or
                self.retained_points > self.args.max_buffer_points)):
            self.retained_points -= len(self.window.popleft()[1])

    def make_window(self, seconds):
        latest = self.window[-1][0]
        frames = [frame for frame in self.window
                  if (latest - frame[0]).nanoseconds / 1e9 <= seconds]
        xyz = np.vstack([frame[1] for frame in frames])
        intensity = np.concatenate([frame[2] for frame in frames])
        idx = voxel_indices(xyz, self.args.voxel)
        cloud = build_cloud(latest.to_msg(), 'odom', xyz[idx], intensity[idx])
        span = (latest - frames[0][0]).nanoseconds / 1e9
        return cloud, len(frames), span

    def publish_window(self):
        if not self.window:
            self.log_timing('skip', 0, 0, 'no_input')
            return
        latest = self.window[-1][0]
        age = (self.get_clock().now() - latest).nanoseconds / 1e9
        if age > 2.0 or age < -1.0:
            self.log_timing('skip', latest.nanoseconds, 0, 'input_stamp_out_of_range')
            return
        long_cloud, long_frames, long_span = self.make_window(self.args.window_time)
        self.global_pub.publish(long_cloud)
        seconds = (self.args.tracking_window_time if self.tracking
                   else self.args.window_time)
        if self.tracking:
            local_cloud, local_frames, local_span = self.make_window(seconds)
        else:
            local_cloud, local_frames, local_span = long_cloud, long_frames, long_span
        self.local_pub.publish(local_cloud)
        self.log_timing('publish', latest.nanoseconds, local_cloud.width, '')
        status = dict(schema_version=2, cloud_stamp_ns=latest.nanoseconds,
                      window_start_ns=self.window[-local_frames][0].nanoseconds,
                      long_window_start_ns=self.window[-long_frames][0].nanoseconds,
                      long_requested_seconds=self.args.window_time,
                      local_topic=self.args.out_topic, global_topic=self.args.global_topic,
                      mode='tracking' if self.tracking else 'initial_or_recovery',
                      requested_seconds=seconds, actual_seconds=round(local_span, 2),
                      frames=local_frames, points=local_cloud.width,
                      retained_frames=long_frames, retained_points=self.retained_points,
                      long_actual_seconds=round(long_span, 2), long_points=long_cloud.width)
        self.status_pub.publish(String(data=json.dumps(status)))

    def log_timing(self, event, cloud_stamp_ns, points, reason):
        if self.timing_csv is not None:
            self.timing_csv.write(
                f'{event},{time.monotonic_ns()},{self.get_clock().now().nanoseconds},'
                f'{cloud_stamp_ns},{points},{reason}\n')

    def destroy_node(self):
        if self.timing_csv is not None:
            self.timing_csv.close()
        return super().destroy_node()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--window-time', type=float, default=120.0)
    ap.add_argument('--tracking-window-time', type=float, default=30.0)
    ap.add_argument('--window-frames', type=int, default=1500)
    ap.add_argument('--max-buffer-points', type=int, default=3000000)
    ap.add_argument('--voxel', type=float, default=0.15)
    ap.add_argument('--rate', type=float, default=1.0)
    ap.add_argument('--timing-csv', default='', help='Optional local-map publication timing CSV')
    ap.add_argument('--in-topic', default='/fast_lio/local_cloud')
    ap.add_argument('--out-topic', default='/fast_lio/local_map')
    ap.add_argument('--global-topic', default='/fast_lio/local_map_global')
    ap.add_argument('--window-status-topic', default='/fast_lio/window_status')
    ap.add_argument('--localization-status-topic', default='/localization/status')
    ap.add_argument('--adaptive', action='store_true')
    ap.add_argument('--recovery-failures', type=int, default=3)
    args, ros_args = ap.parse_known_args()
    if min(args.window_time, args.tracking_window_time, args.window_frames,
           args.max_buffer_points, args.voxel, args.rate, args.recovery_failures) <= 0:
        ap.error('Window, voxel, rate and buffer limits must be positive')
    if args.tracking_window_time > args.window_time:
        ap.error('Tracking window cannot exceed the long window')

    rclpy.init(args=ros_args)
    node = LocalMapWindow(args)
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
