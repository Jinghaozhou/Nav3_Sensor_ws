"""点云抽稀:体素降采样减小点云密度(可视化/对比用,不动原图)。

用法:
    python3 thin_cloud.py <输入.pcd|.ply> [-v 体素边长m] [-o 输出.ply]
    python3 thin_cloud.py map.ply -v 0.2    # 每 0.2m 网格保留一个点

默认体素 0.2m、输出 <原名>_thinned.ply(带强度/颜色,CloudCompare 直接拖入)。
体素越大点越少:0.2m 约为 0.1m 地图的 1/3~1/5 点数,0.3m 再减半左右。

注意:抽稀只用于看图和对比;Fast-LIO2 实时建图分辨率由 mid360_robot.yaml
的 filter_size_map 控制,下游避障用 /fast_lio/local_cloud,不受此影响。
"""

import argparse
import os
import sys

import numpy as np
import open3d as o3d

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pcd2ply  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input', help='输入点云(PCD 或 PLY)')
    ap.add_argument('-v', '--voxel', type=float, default=0.2,
                    help='体素边长(m),默认 0.2')
    ap.add_argument('-o', '--output', default=None,
                    help='输出 PLY(默认 <原名>_thinned.ply)')
    args = ap.parse_args()

    cloud = o3d.geometry.PointCloud()
    ext = os.path.splitext(args.input)[1].lower()
    if ext == '.pcd':
        xyz_or_more, fields = pcd2ply.read_pcd(args.input)
        cloud.points = o3d.utility.Vector3dVector(xyz_or_more[:, :3])
        if 'intensity' in fields:
            it = xyz_or_more[:, fields.index('intensity')]
            vmax = float(it.max())
            if vmax > 0:
                it = it / vmax
            cloud.colors = o3d.utility.Vector3dVector(np.tile(it[:, None], (1, 3)))
    else:
        cloud = o3d.io.read_point_cloud(args.input)

    n0 = len(cloud.points)
    if n0 == 0:
        print(f'{args.input} 读不到点'); return 1
    thin = cloud.voxel_down_sample(args.voxel)
    out = args.output or os.path.splitext(args.input)[0] + '_thinned.ply'
    o3d.io.write_point_cloud(out, thin)
    print(f'{args.input}: {n0:,} 点 → {len(thin.points):,} 点'
          f'(体素 {args.voxel}m,保留 {len(thin.points) / n0 * 100:.1f}%)')
    print(f'输出: {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
