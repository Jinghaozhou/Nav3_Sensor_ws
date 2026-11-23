"""PCD → PLY 转换(适配 apt 版 CloudCompare 无法打开 .pcd 的问题)。

用法:
    python3 pcd2ply.py <输入.pcd> [-o 输出.ply]
    python3 pcd2ply.py *.pcd            # 批量转换(输出到同目录同名 .ply)

CloudCompare 打开 PLY 与 PCD 效果一致(都只显示 xyz,PLY 可带强度)。
"""

import argparse
import glob
import os
import sys

import numpy as np
import open3d as o3d


def read_pcd(path):
    """读取 PCD(兼容多字段,取 x y z + intensity 若有)。

    按头部声明的点数截断 —— 部分 Docker 版 SLAM 写出的 PCD 尾部有多余字节。
    """
    with open(path, 'rb') as f:
        data = f.read()
    idx = data.index(b'DATA binary\n') + len(b'DATA binary\n')
    header = data[:idx].decode('latin-1')
    fields = [l for l in header.split('\n') if l.startswith('FIELDS')][0].split()[1:]
    n_fields = len(fields)
    width = int([l for l in header.split('\n') if l.startswith('WIDTH')][0].split()[1])
    expected = width * n_fields * 4
    if len(data) - idx < expected:
        raise ValueError(f'{path}: 数据段不足(需 {expected} 字节,实际 {len(data)-idx})')
    arr = np.frombuffer(data[idx:idx + expected], dtype=np.float32)
    return arr.reshape(width, n_fields), fields


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('inputs', nargs='+', help='PCD 文件(支持通配符)')
    ap.add_argument('-o', '--output', default=None,
                    help='输出 PLY(仅单个输入时有效,默认同目录同名)')
    args = ap.parse_args()

    paths = []
    for pat in args.inputs:
        paths.extend(glob.glob(pat))
    if not paths:
        print('没找到输入文件'); return 1
    if args.output and len(paths) > 1:
        print('-o 只在单个输入时有效'); return 1

    for path in paths:
        xyz_or_more, fields = read_pcd(path)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz_or_more[:, :3])
        if 'intensity' in fields:
            i = fields.index('intensity')
            it = xyz_or_more[:, i]
            vmax = float(it.max())
            if vmax > 0:   # 按实际最大值归一化(兼容双线 0-63 / Mid-360 0-255)
                it = it / vmax
            pcd.colors = o3d.utility.Vector3dVector(np.tile(it[:, None], (1, 3)))
        out = args.output or os.path.splitext(path)[0] + '.ply'
        o3d.io.write_point_cloud(out, pcd)
        print(f'{path} → {out}({len(pcd.points):,} 点)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
