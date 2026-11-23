"""PCD 点云地图分析:统计 + 三连图(俯视/侧视/高度分布)。

用法:
    python3 analyze_map.py <map.pcd> [-o 输出图.png]
    python3 analyze_map.py mid360_map_20260922_165743.pcd -o slam_output/analysis.png

用途: 快速判断一张地图的规模、稀疏程度、扫描结构(如双线雷达的两条带)。
"""

import argparse
import sys

import numpy as np


def read_xyz(path):
    """读取 PCD 二进制文件的 x y z(float32,米)。兼容多字段(只取前三列)。

    按头部声明的点数截断 —— 部分 Docker 版 SLAM 写出的 PCD 尾部有多余字节。
    """
    with open(path, 'rb') as f:
        data = f.read()
    idx = data.index(b'DATA binary\n') + len(b'DATA binary\n')
    header = data[:idx].decode('latin-1')
    fields_line = [l for l in header.split('\n') if l.startswith('FIELDS')][0]
    n_fields = len(fields_line.split()[1:])
    width = int([l for l in header.split('\n') if l.startswith('WIDTH')][0].split()[1])
    expected = width * n_fields * 4
    if len(data) - idx < expected:
        raise ValueError(f'{path}: 数据段不足(需 {expected} 字节,实际 {len(data)-idx})')
    arr = np.frombuffer(data[idx:idx + expected], dtype=np.float32)
    return arr.reshape(width, n_fields)[:, :3]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('pcd', help='PCD 文件路径')
    ap.add_argument('-o', '--output', default=None, help='分析图输出路径(默认不画图)')
    args = ap.parse_args()

    xyz = read_xyz(args.pcd)
    print(f'点数: {len(xyz):,}')
    print(f'X: [{xyz[:,0].min():.2f}, {xyz[:,0].max():.2f}] m  跨度 {xyz[:,0].ptp():.1f} m')
    print(f'Y: [{xyz[:,1].min():.2f}, {xyz[:,1].max():.2f}] m  跨度 {xyz[:,1].ptp():.1f} m')
    print(f'Z: [{xyz[:,2].min():.2f}, {xyz[:,2].max():.2f}] m  跨度 {xyz[:,2].ptp():.1f} m')

    area = max(xyz[:,0].ptp() * xyz[:,1].ptp(), 1e-6)
    print(f'面密度: {len(xyz)/area:.2f} 点/㎡')

    d = np.linalg.norm(xyz, axis=1)
    print(f'距离分布: <1m {np.sum(d<1):,} | 1-10m {np.sum((d>=1)&(d<10)):,} '
          f'| >10m {np.sum(d>=10):,}')

    hist, edges = np.histogram(xyz[:,2], bins=40)
    print('高度分布峰值区间:')
    for i in np.argsort(hist)[-6:][::-1]:
        print(f'  z ∈ [{edges[i]:6.2f}, {edges[i+1]:6.2f}] m : {hist[i]:>8,} 点')

    if args.output:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        rng = np.random.default_rng(42)
        sel = rng.choice(len(xyz), min(50000, len(xyz)), replace=False)
        p = xyz[sel]
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        axes[0].scatter(p[:,0], p[:,1], s=1, c=p[:,2], cmap='viridis', linewidths=0)
        axes[0].set_aspect('equal')
        axes[0].set_xlabel('X (m)'); axes[0].set_ylabel('Y (m)')
        axes[0].set_title('Top view (colored by height)')
        axes[1].scatter(p[:,0], p[:,2], s=1, c='steelblue', linewidths=0)
        axes[1].set_aspect('equal')
        axes[1].set_xlabel('X (m)'); axes[1].set_ylabel('Z (m)')
        axes[1].set_title('Side view')
        axes[2].hist(xyz[:,2], bins=60, orientation='horizontal', color='steelblue')
        axes[2].set_xlabel('Points'); axes[2].set_ylabel('Z (m)')
        axes[2].set_title('Height histogram')
        axes[2].grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(args.output, dpi=110)
        print(f'分析图已保存: {args.output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
