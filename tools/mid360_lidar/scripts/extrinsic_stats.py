"""外参固化统计分析(任务 FL-12):从 Fast-LIO2 位姿日志提取外参估计值。

背景:fast_lio 的 fout_out 日志(Log/mat_out.txt)每帧记录在线估计的外参
offset_T_L_I 与外参欧拉角(ZYX,度),但只在 runtime_pos_log_enable: true
时写入。多次扫描后,用本脚本统计收敛段的均值/标准差,给出可写入
mid360_robot.yaml 的固化值。

mat_out.txt 每行 24 列:
    t, euler_cur×3, pos×3, ext_euler×3(度), offset_T×3, vel×3, bg×3, ba×3, grav, npts

用法:
    python3 extrinsic_stats.py <Log/mat_out.txt> [--warmup 10]
输出:外参均值和标准差、收敛段对比、可写入 yaml 的 extrinsic_T/extrinsic_R。
"""

import argparse
import sys

import numpy as np

# 各列的列名(按 fout_out 写入顺序)
COLS = ['t', 'euler_x', 'euler_y', 'euler_z',
        'pos_x', 'pos_y', 'pos_z',
        'ext_roll', 'ext_pitch', 'ext_yaw',
        'T_x', 'T_y', 'T_z',
        'vel_x', 'vel_y', 'vel_z',
        'bg_x', 'bg_y', 'bg_z',
        'ba_x', 'ba_y', 'ba_z',
        'grav', 'npts']


def euler_zyx_to_R(deg):
    """ZYX 欧拉角(度)→ 旋转矩阵(与 use-ikfom.hpp 的 SO3ToEuler 互逆)。"""
    r, p, y = np.radians(deg)
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log', help='Log/mat_out.txt 路径')
    ap.add_argument('--warmup', type=float, default=10.0,
                    help='剔除前 N 秒未收敛段(默认 10)')
    ap.add_argument('--tail', type=float, default=0.0,
                    help='剔除末尾 N 秒(收尾晃动,默认 0)')
    args = ap.parse_args()

    try:
        data = np.loadtxt(args.log)
    except OSError:
        print(f'找不到 {args.log} —— 先确认 mid360_robot.yaml 里 '
              'runtime_pos_log_enable: true,再跑一次扫描'); return 1
    if data.ndim == 1:
        data = data[None, :]
    if data.shape[1] != len(COLS):
        print(f'列数不符:预期 {len(COLS)},实际 {data.shape[1]}(确认是 fast_lio 的 '
              'mat_out.txt,不是 mat_pre.txt)'); return 1
    if len(data) < 100:
        print(f'数据太少({len(data)} 帧),外参未充分收敛,需要更长的扫描'); return 1

    t = data[:, 0] - data[0, 0]
    mask = (t >= args.warmup) & (t <= t[-1] - args.tail)
    seg = data[mask]
    ext_euler = seg[:, 8:11]
    ext_T = seg[:, 11:14]
    print(f'总帧数 {len(data)},用时 {t[-1]:.0f}s,统计段 {mask.sum()} 帧'
          f'({args.warmup:.0f}s 之后)')

    def stats(x, names):
        for name, col in zip(names, x.T):
            print(f'  {name:10s} 均值 {col.mean():8.4f}  标准差 {col.std():8.4f}')

    print('外参欧拉角(度,ZYX):')
    stats(ext_euler, ['roll', 'pitch', 'yaw'])
    print('外参平移(m):')
    stats(ext_T, ['x', 'y', 'z'])

    # 收敛性:统计段再分三段,比较均值漂移
    thirds = np.array_split(ext_T, 3)
    drift = np.abs(thirds[0].mean(0) - thirds[-1].mean(0))
    print(f'首尾 1/3 段平移均值漂移: {drift} m '
          f'({"已收敛" if drift.max() < 0.005 else "未收敛,需更长扫描"})')

    # 给出可写入 yaml 的固化值
    R = euler_zyx_to_R(ext_euler.mean(0))
    T = ext_T.mean(0)
    print('\n建议写入 mid360_robot.yaml(均值):')
    print(f'  extrinsic_T: [ {T[0]:.4f}, {T[1]:.4f}, {T[2]:.4f} ]')
    print(f'  extrinsic_R: [ {R[0,0]:.6f}, {R[0,1]:.6f}, {R[0,2]:.6f},')
    print(f'                 {R[1,0]:.6f}, {R[1,1]:.6f}, {R[1,2]:.6f},')
    print(f'                 {R[2,0]:.6f}, {R[2,1]:.6f}, {R[2,2]:.6f} ]')
    print('  extrinsic_est_en:  false')
    print('\n注意:固化后关闭在线估计,需多次扫描对比确认地图无系统性变形,'
          '且同步更新 launch/mid360_mapping.launch.py 里的 base_link→lidar 静态 TF。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
