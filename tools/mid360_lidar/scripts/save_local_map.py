"""保存滑动窗口局部地图快照:订阅 /fast_lio/local_map 存为 PCD + PLY。

滑动窗口(/fast_lio/local_map)是滚动数据不落盘,本脚本抓取当前窗口
(或连续 N 条消息合并)保存,供离线测试使用(如 KISS-Matcher 局部地图输入)。

用法(fast_lio 运行时,系统 python3):
    python3 scripts/save_local_map.py                       # 抓一帧,存 slam_output/
    python3 scripts/save_local_map.py -n 3 -o /tmp/local_map.pcd   # 合并 3 条消息

输出:<输出名>.pcd(x y z intensity 浮点)+ 同名 .ply(强度灰度,CloudCompare 直接拖入)。
"""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from local_map_window import parse_cloud  # noqa: E402  复用 PointCloud2 解析


class Saver(Node):
    def __init__(self, n_msgs, timeout):
        super().__init__('save_local_map')
        self.n_msgs = n_msgs
        self.timeout = timeout
        self.chunks = []
        self.stamps = []
        qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(PointCloud2, '/fast_lio/local_map',
                                            self.on_msg, qos)

    def on_msg(self, msg):
        xyz, it = parse_cloud(msg)
        self.chunks.append((xyz, it))
        self.stamps.append(msg.header.stamp)
        self.get_logger().info(f'收到一帧窗口: {len(xyz)} 点')
        if len(self.chunks) >= self.n_msgs:
            raise SystemExit  # 收够了退出 spin

    def merged(self):
        xyz = np.vstack([c[0] for c in self.chunks])
        it = np.concatenate([c[1] for c in self.chunks]) \
            if self.chunks[0][1] is not None else None
        return xyz, it


def write_pcd(path, xyz, intensity):
    n = len(xyz)
    header = ['# .PCD v0.7 - Point Cloud Data file format', 'VERSION 0.7',
              'FIELDS x y z intensity', 'SIZE 4 4 4 4', 'TYPE F F F F',
              'COUNT 1 1 1 1', f'WIDTH {n}', 'HEIGHT 1',
              'VIEWPOINT 0 0 0 1 0 0 0', f'POINTS {n}', 'DATA binary', '']
    arr = np.zeros((n, 4), dtype=np.float32)
    arr[:, :3] = xyz
    if intensity is not None:
        arr[:, 3] = intensity
    with open(path, 'wb') as f:
        f.write('\n'.join(header).encode())
        f.write(np.ascontiguousarray(arr, dtype=np.float32).tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-n', type=int, default=1,
                    help='合并的窗口消息条数(默认 1 = 一个快照)')
    ap.add_argument('-o', '--output', default=None,
                    help='输出 PCD(默认 slam_output/local_map_时间戳.pcd)')
    ap.add_argument('--timeout', type=float, default=10.0,
                    help='等不到消息的超时秒数(默认 10)')
    args = ap.parse_args()

    rclpy.init()
    saver = Saver(args.n, args.timeout)
    t0 = time.time()
    try:
        while time.time() - t0 < args.timeout:
            rclpy.spin_once(saver, timeout_sec=0.1)
            if len(saver.chunks) >= args.n:
                break
    except SystemExit:
        pass
    if not saver.chunks:
        print('超时未收到 /fast_lio/local_map(确认一键 launch 在跑)')
        sys.exit(1)

    xyz, it = saver.merged()
    out = args.output or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'slam_output', time.strftime('local_map_%Y%m%d_%H%M%S.pcd'))
    write_pcd(out, xyz, it)
    print(f'PCD → {out}({len(xyz):,} 点)')

    # 同名 PLY(强度灰度;纯 python 写,不依赖 open3d,系统 python 可用)
    ply = os.path.splitext(out)[0] + '.ply'
    n = len(xyz)
    gray = ((it / float(it.max()) * 255) if it is not None and it.max() > 0
            else np.full(n, 128)).astype(np.uint8)
    with open(ply, 'wb') as f:
        f.write(f'ply\nformat binary_little_endian 1.0\n'
                f'element vertex {n}\nproperty float x\nproperty float y\n'
                f'property float z\nproperty uchar red\nproperty uchar green\n'
                f'property uchar blue\nend_header\n'.encode())
        rec = np.zeros(n, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
                                 ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
        rec['x'], rec['y'], rec['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        rec['r'] = rec['g'] = rec['b'] = gray
        f.write(rec.tobytes())
    print(f'PLY → {ply}(拖进 CloudCompare)')
    saver.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
