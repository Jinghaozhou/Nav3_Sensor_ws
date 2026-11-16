#!/usr/bin/env python3
"""Monitor unified MID360 inputs, odometry frequency and Fast-LIO process resources."""

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError:
    CustomMsg = None

REPORT_HZ = 1.0


def _proc_stat(pid):
    """返回进程的 (utime+stime 时钟滴答, rss 字节),不存在返回 None。"""
    try:
        with open(f'/proc/{pid}/stat', 'rb') as f:
            parts = f.read().split()
        # comm 可能带空格,按右括号定位:fields[13/14]=utime/stime, fields[23]=rss
        return int(parts[13]) + int(parts[14]), int(parts[23]) * os.sysconf('SC_PAGE_SIZE')
    except (OSError, IndexError):
        return None


def find_fastlio_pid():
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/comm') as f:
                if f.read().strip() == 'fastlio_mapping':
                    return int(pid)
        except OSError:
            pass
    return None


class LivoxMonitor(Node):
    def __init__(self, min_lidar_hz, min_imu_hz, min_odom_hz,
                 lidar_topic, imu_topic, odom_topic, health_topic):
        super().__init__('livox_monitor')
        self.lidar_topic = lidar_topic
        self.imu_topic = imu_topic
        self.odom_topic = odom_topic
        self.min_hz = {lidar_topic: min_lidar_hz,
                       imu_topic: min_imu_hz,
                       odom_topic: min_odom_hz}
        self.counts = {t: 0 for t in self.min_hz}
        self.last_report = time.monotonic()
        self.last_cpu = None

        qos = QoSProfile(depth=1000)  # Reliable input publishers; absorb IMU bursts.
        if CustomMsg is not None:
            self.create_subscription(CustomMsg, lidar_topic,
                                     lambda m: self._count(lidar_topic), qos)
        else:
            self.get_logger().error('未找到 livox_ros_driver2，请加载当前工作空间的 install/setup.bash')
        self.create_subscription(Odometry, odom_topic,
                                 lambda m: self._count(odom_topic), qos)
        self.create_subscription(Imu, imu_topic, lambda m: self._count(imu_topic), qos)

        self.health_pub = self.create_publisher(Float32MultiArray, health_topic, 10)
        self.timer = self.create_timer(1.0 / REPORT_HZ, self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _count(self, topic):
        self.counts[topic] += 1

    def report(self):
        now = time.monotonic()
        dt = now - self.last_report
        self.last_report = now
        rates = {t: c / dt for t, c in self.counts.items()}
        self.counts = {t: 0 for t in self.counts}

        pid = find_fastlio_pid()
        rss_mb, cpu_pct = 0.0, 0.0
        if pid is not None:
            st = _proc_stat(pid)
            if st:
                ticks, rss = st
                rss_mb = rss / 1e6
                if self.last_cpu:
                    prev_ticks, prev_t = self.last_cpu
                    cpu_pct = (ticks - prev_ticks) * 100.0 / (
                        (now - prev_t) * os.sysconf('SC_CLK_TCK'))
                self.last_cpu = (ticks, now)
        alive = pid is not None

        ok = [rates[t] >= self.min_hz[t] for t in self.min_hz] + [alive]
        healthy = 1.0 if all(ok) else 0.0

        msg = Float32MultiArray()
        msg.data = [float(rates[self.lidar_topic]), float(rates[self.imu_topic]),
                    float(rates[self.odom_topic]), healthy, float(rss_mb),
                    float(cpu_pct)]
        self.health_pub.publish(msg)

        parts = (f"lidar {rates[self.lidar_topic]:5.1f}Hz(≥{self.min_hz[self.lidar_topic]:.0f}) "
                 f"imu {rates[self.imu_topic]:6.1f}Hz(≥{self.min_hz[self.imu_topic]:.0f}) "
                 f"odom {rates[self.odom_topic]:5.1f}Hz(≥{self.min_hz[self.odom_topic]:.0f}) "
                 f"rss {rss_mb:7.0f}MB cpu {cpu_pct:5.1f}%")
        if healthy:
            self.get_logger().info('OK   ' + parts)
        else:
            self.get_logger().warn('异常 ' + parts +
                                   (' | fastlio_mapping 进程不存在' if not alive else ''))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--lidar-hz', type=float, default=8.0, help='点云最低频率(默认 8)')
    ap.add_argument('--imu-hz', type=float, default=150.0, help='IMU 最低频率(默认 150)')
    ap.add_argument('--odom-hz', type=float, default=8.0, help='里程计最低频率(默认 8)')
    ap.add_argument('--lidar-topic', default='/lidar_points')
    ap.add_argument('--imu-topic', default='/imu')
    ap.add_argument('--odom-topic', default='/fast_lio/odom')
    ap.add_argument('--health-topic', default='/fast_lio/health')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = LivoxMonitor(args.lidar_hz, args.imu_hz, args.odom_hz,
                        args.lidar_topic, args.imu_topic, args.odom_topic,
                        args.health_topic)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except RuntimeError:
        # A queued subscription can race with rclpy shutdown after SIGINT.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
