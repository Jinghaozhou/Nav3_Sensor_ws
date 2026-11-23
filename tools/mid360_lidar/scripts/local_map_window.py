#!/usr/bin/env python3
"""FIFO map windows in odom: long initialization/recovery, short prior tracking.

Window age is relative to sensor timestamps; frame and point caps can shorten it.
Status logs report actual coverage. No transform estimation is performed here.
"""
import argparse
import signal
from collections import deque
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


def parse_cloud(msg):
    """Honor field offsets, row padding and byte order (including PCL XYZINormal)."""
    endian = '>' if msg.is_bigendian else '<'
    formats = {PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8',
               PointField.UINT8: 'u1', PointField.UINT16: 'u2'}
    fields = {f.name: f for f in msg.fields}
    def channel(name):
        f = fields[name]
        if f.count != 1 or f.datatype not in formats:
            raise ValueError(f'unsupported field {name}')
        return np.ndarray((msg.height, msg.width), dtype=endian+formats[f.datatype],
                          buffer=msg.data, offset=f.offset,
                          strides=(msg.row_step, msg.point_step)).reshape(-1)
    xyz = np.column_stack([channel(n) for n in ('x', 'y', 'z')]).astype(np.float32)
    intensity = channel('intensity').astype(np.float32) if 'intensity' in fields else np.zeros(len(xyz), np.float32)
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid], np.nan_to_num(intensity[valid])


def build_cloud(stamp, frame_id, xyz, intensity=None):
    msg = PointCloud2()
    msg.header.stamp, msg.header.frame_id = stamp, frame_id
    msg.height, msg.width, msg.is_dense = 1, len(xyz), True
    msg.point_step, msg.row_step = 16, len(xyz)*16
    msg.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
    arr = np.zeros((len(xyz), 4), dtype='<f4')
    arr[:, :3] = xyz
    if intensity is not None:
        arr[:, 3] = intensity
    msg.data = arr.tobytes()
    return msg


def voxel_indices(xyz, voxel):
    if not len(xyz): return np.array([], dtype=np.int64)
    # Compare complete XYZ keys; scalar hash collisions must not discard valid voxels.
    _, idx = np.unique(np.floor(xyz / voxel).astype(np.int64), axis=0, return_index=True)
    return np.sort(idx)


def voxel_downsample(xyz, voxel):
    return xyz[voxel_indices(xyz, voxel)]


class LocalMapWindow(Node):
    def __init__(self, args):
        super().__init__('local_map_window')
        self.args = args
        self.window, self.points = deque(), 0
        self.tracking, self.failures = False, 0
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, args.in_topic, self.on_cloud, qos)
        self.pub = self.create_publisher(PointCloud2, args.out_topic, 1)
        self.global_pub = self.create_publisher(PointCloud2, args.global_topic, 1)
        self.stats_pub = self.create_publisher(String, '/fast_lio/window_status', 1)
        if args.adaptive:
            try:
                from kiss_matcher_localization.msg import LocalizationResult
            except ImportError:
                self.get_logger().warn('KISS environment not sourced: fixed long window only; source kiss install and restart for adaptive windows')
                return self.finish_setup(args)

            status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(LocalizationResult, '/localization/status', self.on_status, status_qos)
        self.finish_setup(args)

    def finish_setup(self, args):
        self.create_timer(1.0 / args.rate, self.publish_window)
        self.get_logger().info(f'initial={args.window_time}s tracking={args.tracking_window_time}s '
                               f'cap={args.window_frames} frames/{args.max_buffer_points} points adaptive={args.adaptive}')

    def on_status(self, result):
        if result.applied:
            self.tracking, self.failures = True, 0
        elif self.tracking:
            self.failures += 1
            if self.failures >= self.args.recovery_failures:
                self.tracking = False

    def on_cloud(self, msg):
        if msg.header.frame_id != 'odom':
            self.get_logger().error(f'reject frame={msg.header.frame_id}, expected odom')
            return
        xyz, intensity = parse_cloud(msg)
        if not len(xyz): return
        stamp = Time.from_msg(msg.header.stamp)
        if self.window and stamp.nanoseconds <= self.window[-1][0].nanoseconds:
            self.window.clear(); self.points = 0; self.tracking = False
            self.get_logger().warn('sensor time reset: cleared window; restart localization too if Fast-LIO restarted')
        idx = voxel_indices(xyz, self.args.voxel)
        xyz, intensity = xyz[idx], intensity[idx]
        self.window.append((stamp, xyz, intensity)); self.points += len(xyz)
        while self.window and (
            (stamp - self.window[0][0]).nanoseconds / 1e9 > self.args.window_time or
            len(self.window) > self.args.window_frames or self.points > self.args.max_buffer_points):
            self.points -= len(self.window.popleft()[1])

    def make_window(self, seconds):
        last = self.window[-1][0]
        frames = [f for f in self.window if (last-f[0]).nanoseconds/1e9 <= seconds]
        xyz = np.vstack([f[1] for f in frames])
        intensity = np.concatenate([f[2] for f in frames])
        idx = voxel_indices(xyz, self.args.voxel)
        return build_cloud(last.to_msg(), 'odom', xyz[idx], intensity[idx]), len(frames), (last-frames[0][0]).nanoseconds/1e9

    def publish_window(self):
        if not self.window: return
        age = (self.get_clock().now()-self.window[-1][0]).nanoseconds/1e9
        if age > 2.0 or age < -1.0: return  # Never keep republishing a stopped sensor as fresh data.
        long_cloud, count, coverage = self.make_window(self.args.window_time)
        self.global_pub.publish(long_cloud)
        seconds = self.args.tracking_window_time if self.tracking else self.args.window_time
        cloud, n, span = self.make_window(seconds) if self.tracking else (long_cloud, count, coverage)
        self.pub.publish(cloud)
        self.stats_pub.publish(String(data=json.dumps(dict(
            schema_version=2,
            cloud_stamp_ns=self.window[-1][0].nanoseconds,
            window_start_ns=self.window[-n][0].nanoseconds,
            long_window_start_ns=self.window[-count][0].nanoseconds,
            long_requested_seconds=self.args.window_time,
            local_topic=self.args.out_topic, global_topic=self.args.global_topic,
            mode='tracking' if self.tracking else 'initial_or_recovery',
            requested_seconds=seconds, actual_seconds=round(span, 2), frames=n,
            points=cloud.width, retained_frames=count, retained_points=self.points,
            long_actual_seconds=round(coverage, 2), long_points=long_cloud.width))))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--window-time', type=float, default=120.0)
    ap.add_argument('--tracking-window-time', type=float, default=30.0)
    ap.add_argument('--window-frames', type=int, default=1500)
    ap.add_argument('--max-buffer-points', type=int, default=3000000)
    ap.add_argument('--voxel', type=float, default=0.15)
    ap.add_argument('--rate', type=float, default=1.0)
    ap.add_argument('--in-topic', default='/fast_lio/local_cloud')
    ap.add_argument('--out-topic', default='/fast_lio/local_map')
    ap.add_argument('--global-topic', default='/fast_lio/local_map_global')
    ap.add_argument('--adaptive', action='store_true')
    ap.add_argument('--recovery-failures', type=int, default=3)
    args, ros_args = ap.parse_known_args()
    if min(args.window_time, args.tracking_window_time, args.voxel, args.rate,
           args.window_frames, args.max_buffer_points, args.recovery_failures) <= 0:
        ap.error('all numeric limits must be positive')
    if args.tracking_window_time > args.window_time:
        ap.error('tracking window cannot exceed initial window')
    rclpy.init(args=ros_args)
    node = LocalMapWindow(args)
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
