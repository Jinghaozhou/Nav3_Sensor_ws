#!/usr/bin/env python3
"""Fast-LIO2 链路健康监控(任务 FL-08:节点状态与频率监控)。

订阅 /lidar_points、/imu、/fast_lio/odom 统计实际频率,同时读取
fastlio_mapping 进程的 CPU/内存占用,1Hz 汇总:

  - 控制台打印正常/异常日志
  - 发布 /fast_lio/health (Float32MultiArray),供其他模块独立判断
    Fast-LIO2 是否可用,字段顺序:
        [lidar_hz, imu_hz, odom_hz, healthy, rss_mb, cpu_pct]
        healthy = 1/0(三个频率达标且 fastlio_mapping 进程存活)

用法(需 source livox_ws,因为要 import livox_interfaces2):
    python3 scripts/livox_monitor.py
    python3 scripts/livox_monitor.py --lidar-hz 8 --imu-hz 150 --odom-hz 8
一键 launch 已默认启动本节点。
"""

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

try:
    # 注意:Python 包名是 livox_ros_driver2(C++ 命名空间),
    # 不是 rosidl 子包名 livox_interfaces2
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
    def __init__(self, min_lidar_hz, min_imu_hz, min_odom_hz):
        super().__init__('livox_monitor')
        self.min_hz = {'/lidar_points': min_lidar_hz,
                       '/imu': min_imu_hz,
                       '/fast_lio/odom': min_odom_hz}
        self.counts = {t: 0 for t in self.min_hz}
        self.last_report = time.monotonic()
        self.last_cpu = None

        qos = qos_profile_sensor_data
        if CustomMsg is not None:
            self.create_subscription(CustomMsg, '/lidar_points',
                                     lambda m: self._count('/lidar_points'), qos)
        else:
            self.get_logger().error('未找到 livox_interfaces2,请先加载当前Nav3的scripts/source_scan_env.bash')
        self.create_subscription(Odometry, '/fast_lio/odom',
                                 lambda m: self._count('/fast_lio/odom'), qos)
        self.create_subscription(Imu, '/imu', lambda m: self._count('/imu'), qos)

        self.health_pub = self.create_publisher(Float32MultiArray, '/fast_lio/health', 10)
        self.timer = self.create_timer(1.0 / REPORT_HZ, self.report)

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
        msg.data = [float(rates['/lidar_points']), float(rates['/imu']),
                    float(rates['/fast_lio/odom']), healthy, float(rss_mb),
                    float(cpu_pct)]
        self.health_pub.publish(msg)

        parts = (f"lidar {rates['/lidar_points']:5.1f}Hz(≥{self.min_hz['/lidar_points']:.0f}) "
                 f"imu {rates['/imu']:6.1f}Hz(≥{self.min_hz['/imu']:.0f}) "
                 f"odom {rates['/fast_lio/odom']:5.1f}Hz(≥{self.min_hz['/fast_lio/odom']:.0f}) "
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
    args = ap.parse_args()

    rclpy.init()
    node = LivoxMonitor(args.lidar_hz, args.imu_hz, args.odom_hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
