"""点云 → 实体网格(Poisson 表面重建)。

用法:
    python3 pointcloud_to_mesh.py <map.pcd> -o 输出.ply [--voxel 0.08] [--depth 9]

流程: 去噪 → 体素降采样 → 估法线 → 泊松重建 → 三角网格(.ply)。
生成的网格用 CloudCompare / MeshLab 打开(⚠️ pcl_viewer 不支持 .ply)。

注意: 重建只是"把采样点连成面",不能凭空创造没有采样的区域 ——
双线雷达的地图重建出来是两条带状曲面;Mid-360 类全覆盖扫描才能出完整实体。
"""

import argparse
import sys

import numpy as np
import open3d as o3d


def read_xyz(path):
    """读取 PCD 二进制文件的 x y z(float32,米)。"""
    with open(path, 'rb') as f:
        data = f.read()
    idx = data.index(b'DATA binary\n') + len(b'DATA binary\n')
    header = data[:idx].decode('latin-1')
    fields_line = [l for l in header.split('\n') if l.startswith('FIELDS')][0]
    n_fields = len(fields_line.split()[1:])
    width = int([l for l in header.split('\n') if l.startswith('WIDTH')][0].split()[1])
    arr = np.frombuffer(data[idx:], dtype=np.float32)
    n = width if len(arr) == width * n_fields else len(arr) // n_fields
    return arr.reshape(n, n_fields)[:, :3]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('pcd', help='输入 PCD 点云')
    ap.add_argument('-o', '--output', required=True, help='输出网格路径(.ply)')
    ap.add_argument('--voxel', type=float, default=0.08, help='体素降采样尺寸 m(默认 0.08)')
    ap.add_argument('--depth', type=int, default=9, help='泊松深度(默认 9,越大越细)')
    args = ap.parse_args()

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(read_xyz(args.pcd))
    print(f'输入: {len(pcd.points):,} 点')

    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd = pcd.voxel_down_sample(args.voxel)
    print(f'降采样后: {len(pcd.points):,} 点')

    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(30)

    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=args.depth)
    print(f'网格: {len(mesh.vertices):,} 顶点, {len(mesh.triangles):,} 三角面')

    o3d.io.write_triangle_mesh(args.output, mesh)
    print(f'已保存: {args.output}(用 CloudCompare 打开,不要用 pcl_viewer)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
