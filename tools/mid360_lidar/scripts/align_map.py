"""点云地图对齐:地面找平(roll/pitch)+ 墙向对齐(yaw),输出横平竖直的地图。

背景:Fast-LIO2 是纯里程计,yaw 没有绝对参考 —— 地图的整体朝向 = 传感器
启动时的朝向;启动时没放平/没静置够,地图还会整体倾斜(本次实测 pitch
偏 14.7°、yaw 偏 39°)。本脚本对已保存的地图做后处理,保证输出地图:
地面与 X-Y 平面平行、墙与 X/Y 轴平行(墙法向沿 X 或 Y)。

用法:
    python3 align_map.py <map.pcd> [-o 输出.pcd]
    python3 align_map.py <map.pcd>   # 默认输出 <原名>_aligned.pcd + 同名 .ply

输出 PCD 保持原字段(含 intensity 与法向,法向随旋转同步更新),另生成
同名 .ply 可直接拖进 CloudCompare。
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pcd2ply  # noqa: E402  复用 read_pcd / 强度转色


def rot_align(n, target):
    """最小旋转矩阵,把单位方向 n 转到 target(Rodrigues)。"""
    n = n / np.linalg.norm(n)
    t = target / np.linalg.norm(target)
    v = np.cross(n, t)
    s = np.linalg.norm(v)
    c = np.dot(n, t)
    if s < 1e-9:
        return np.eye(3) if c > 0 else -np.eye(3)
    V = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]) / s
    return np.eye(3) + s * V + (1 - c) * V @ V


def fit_floor_rotation(pts):
    """RANSAC 多轮找地面(法向最接近竖直的最大平面),返回找平旋转矩阵。"""
    import open3d as o3d
    rest = o3d.geometry.PointCloud()
    rest.points = o3d.utility.Vector3dVector(pts)
    best = None  # (|nz|, n, inliers)
    for _ in range(5):
        if len(rest.points) < 3000:
            break
        plane, inliers = rest.segment_plane(0.05, 3, 1000)
        n = np.asarray(plane)[:3]
        n = n / np.linalg.norm(n)
        score = abs(n[2])
        if score > 0.5 and (best is None or score > best[0]):
            best = (score, n, inliers)
        rest = rest.select_by_index(inliers, invert=True)
    if best is None:
        return None, None
    n = best[1]
    if n[2] < 0:
        n = -n
    R = rot_align(n, np.array([0.0, 0.0, 1.0]))
    roll = np.degrees(np.arctan2(n[1], n[2]))
    pitch = np.degrees(np.arctan2(-n[0], n[2]))
    return R, (roll, pitch)


def fit_yaw_rotation(pts):
    """水平法向的方位角直方图找主墙向,返回绕 Z 的旋转矩阵(墙 ∥ X/Y)。"""
    import open3d as o3d
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(pts)
    sub = cloud.voxel_down_sample(0.25)
    if len(sub.points) < 5000:
        return None, None
    sub.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.8, max_nn=30))
    ns = np.asarray(sub.normals)
    mask = np.abs(ns[:, 2]) < 0.25
    if mask.sum() < 2000:
        return None, None
    az = np.arctan2(ns[mask, 1], ns[mask, 0])
    # 双倍角求和:两套正交墙同时投票,得到主轴方向(模 90°)
    dominant = 0.5 * np.angle(np.sum(np.exp(2j * az)))
    theta = -dominant
    R = np.array([[np.cos(theta), -np.sin(theta), 0],
                  [np.sin(theta), np.cos(theta), 0],
                  [0, 0, 1]])
    return R, np.degrees(theta)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('input', help='输入 PCD 地图')
    ap.add_argument('-o', '--output', default=None,
                    help='输出 PCD(默认 <原名>_aligned.pcd)')
    args = ap.parse_args()

    arr, fields = pcd2ply.read_pcd(args.input)
    n = len(arr)
    print(f'加载 {args.input}: {n:,} 点, 字段 {fields}')

    R_floor, tilt = fit_floor_rotation(arr[:, :3])
    if R_floor is None:
        print('⚠️  未找到地面平面,跳过找平')
        R_floor = np.eye(3)
    else:
        print(f'地面找平: 修正 roll={tilt[0]:.2f}° pitch={tilt[1]:.2f}°')

    leveled = arr[:, :3] @ R_floor.T
    R_yaw, yaw = fit_yaw_rotation(leveled)
    if R_yaw is None:
        print('⚠️  墙面特征不足,跳过 yaw 对齐(保持原朝向)')
        R_yaw = np.eye(3)
    else:
        print(f'墙向对齐: 修正 yaw={yaw:.2f}°(墙与 X/Y 轴平行)')

    R = R_yaw @ R_floor
    out_arr = arr.copy()
    out_arr[:, :3] = arr[:, :3] @ R.T
    if 'normal_x' in fields:
        i = fields.index('normal_x')
        out_arr[:, i:i + 3] = arr[:, i:i + 3] @ R.T

    out = args.output or os.path.splitext(args.input)[0] + '_aligned.pcd'
    header = ['# .PCD v0.7 - Point Cloud Data file format', 'VERSION 0.7',
              'FIELDS ' + ' '.join(fields),
              'SIZE ' + ' '.join(['4'] * len(fields)),
              'TYPE ' + ' '.join(['F'] * len(fields)),
              'COUNT ' + ' '.join(['1'] * len(fields)),
              f'WIDTH {n}', 'HEIGHT 1', 'VIEWPOINT 0 0 0 1 0 0 0',
              f'POINTS {n}', 'DATA binary', '']
    with open(out, 'wb') as f:
        f.write('\n'.join(header).encode())
        f.write(np.ascontiguousarray(out_arr, dtype=np.float32).tobytes())
    print(f'PCD → {out}')

    # 同名 PLY(强度转灰度色,CloudCompare 直接拖入)
    import open3d as o3d
    xyz_or_more, _ = pcd2ply.read_pcd(out)
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz_or_more[:, :3])
    if 'intensity' in fields:
        it = xyz_or_more[:, fields.index('intensity')]
        vmax = float(it.max())
        if vmax > 0:
            it = it / vmax
        cloud.colors = o3d.utility.Vector3dVector(np.tile(it[:, None], (1, 3)))
    ply = os.path.splitext(out)[0] + '.ply'
    o3d.io.write_point_cloud(ply, cloud)
    print(f'PLY → {ply}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
