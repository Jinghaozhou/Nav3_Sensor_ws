#!/usr/bin/env python3
"""Render one saved experiment; never treat a reference map or estimated path as ground truth."""
import argparse
import csv
from datetime import datetime, timedelta, timezone
import json
import math
import sqlite3
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT = Path(__file__).resolve().parents[1]
TZ = timezone(timedelta(hours=8))


def read_csv(path):
    if not path.is_file():
        return []
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def truth(value):
    return str(value).lower() in ('true', '1')


def cutoff_ns(text):
    if not text:
        return None
    value = datetime.fromisoformat(text)
    if value.tzinfo is None:
        raise ValueError('截止时间必须包含时区，例如 2026-10-26T17:18:00+08:00')
    return int(value.timestamp() * 1e9)


def transform(row):
    try:
        value = np.array([[float(row[f'T{i}{j}']) for j in range(4)] for i in range(4)])
        if np.isfinite(value).all() and np.allclose(value[3], [0, 0, 0, 1]):
            return value
    except (KeyError, ValueError):
        pass
    return None


def history_path(session, accepted, stamps, xyz, until_ns):
    """Prefer the actual TF-recorded path; never substitute the reprojected map_path.csv."""
    databases = sorted((session / 'bag').glob('*.db3'))
    error = None
    if databases:
        try:
            from rclpy.serialization import deserialize_message
            from nav_msgs.msg import Path as PathMsg
            latest = None
            for database in databases:
                with sqlite3.connect(database.resolve().as_uri() + '?mode=ro', uri=True) as connection:
                    topic = connection.execute(
                        'SELECT id FROM topics WHERE name=?', ('/localization/map_path',)).fetchone()
                    if topic is None:
                        continue
                    condition = ' AND timestamp<=?' if until_ns is not None else ''
                    args = (topic[0], until_ns) if until_ns is not None else (topic[0],)
                    row = connection.execute(
                        'SELECT timestamp,data FROM messages WHERE topic_id=?' + condition +
                        ' ORDER BY timestamp DESC LIMIT 1', args).fetchone()
                    if row is not None and (latest is None or row[0] > latest[0]):
                        latest = row
            if latest is not None:
                message = deserialize_message(latest[1], PathMsg)
                if message.header.frame_id != 'map':
                    raise ValueError('recorded map_path frame is not map')
                positions, times = [], []
                for pose in message.poses:
                    ns = pose.header.stamp.sec * 10**9 + pose.header.stamp.nanosec
                    point = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                    if (until_ns is None or ns <= until_ns) and np.isfinite(point).all():
                        positions.append(point)
                        times.append(ns)
                return np.array(positions).reshape(-1, 3), times, 'bag:/localization/map_path', None
        except (ImportError, sqlite3.Error, ValueError, RuntimeError) as exc:
            error = str(exc)
    # For legacy sessions: use each correction only AFTER its completion time.
    updates = sorted(((int(row['completed_ns']), transform(row)) for row in accepted
                      if transform(row) is not None), key=lambda update: update[0])
    positions, times = [], []
    current, index = None, 0
    for ns, point in zip(stamps, xyz):
        while index < len(updates) and updates[index][0] <= ns:
            current = updates[index][1]
            index += 1
        if current is not None:
            positions.append(current[:3, :3] @ point + current[:3, 3])
            times.append(int(ns))
    note = '黄色线由CSV按定位完成时间重建，非原始TF采样轨迹。'
    if error:
        note += ' 原始轨迹读取失败：' + error
    return np.array(positions).reshape(-1, 3), times, 'csv_completion_time_reconstruction', note


def load_data(session, until_ns=None):
    events = read_csv(session / 'attempts.csv')
    if until_ns is not None:
        events = [r for r in events if int(r['completed_ns']) <= until_ns
                  and int(r['cloud_ns']) <= until_ns]
    accepted = [r for r in events if truth(r['applied'])]
    poses = read_csv(session / 'odom_path.csv')
    if until_ns is not None:
        poses = [r for r in poses if int(r['stamp_ns']) <= until_ns]
    xyz = np.array([[float(r[k]) for k in ('x', 'y', 'z')] for r in poses]).reshape(-1, 3)
    stamps = np.array([int(r['stamp_ns']) for r in poses], dtype=np.int64)
    valid = np.isfinite(xyz).all(axis=1)
    xyz, stamps = xyz[valid], stamps[valid]
    if len(stamps):
        stamps, indices = np.unique(stamps, return_index=True)
        xyz = xyz[indices]
    last = next((r for r in reversed(accepted) if transform(r) is not None), None)
    T = transform(last) if last is not None else None
    frame = 'map' if T is not None else 'odom'
    path = xyz @ T[:3, :3].T + T[:3, 3] if T is not None else xyz.copy()
    markers = []
    missing = []
    for row in accepted:
        stamp = int(row['cloud_ns'])
        i = int(np.searchsorted(stamps, stamp))
        pos = None
        if i < len(stamps) and stamps[i] == stamp:
            pos = xyz[i]
        elif 0 < i < len(stamps) and stamps[i] - stamps[i-1] <= 500000000:
            ratio = (stamp-stamps[i-1]) / (stamps[i]-stamps[i-1])
            pos = xyz[i-1] * (1-ratio) + xyz[i] * ratio
        candidate = transform(row)
        if pos is None or candidate is None:
            missing.append(row['attempt'])
            continue
        point = candidate[:3, :3] @ pos + candidate[:3, 3]
        markers.append({'row': row, 'point': point})
    historical, historical_stamps, historical_source, historical_note = history_path(
        session, accepted, stamps, xyz, until_ns)
    return dict(events=events, accepted=accepted, last=last, frame=frame, path=path,
                historical_path=historical, historical_stamps=historical_stamps,
                historical_source=historical_source, historical_note=historical_note,
                stamps=stamps, markers=markers, missing_markers=missing)


def read_reference(path, max_points=450000):
    """Bounded preview of binary/ascii XYZ PCD, honoring declared field sizes and offsets."""
    with path.open('rb') as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PCD缺少DATA头')
            text = line.decode('ascii').strip()
            if text and not text.startswith('#'):
                words = text.split()
                header[words[0]] = words[1:]
                if words[0] == 'DATA':
                    offset = stream.tell()
                    break
        names = header['FIELDS']
        sizes = list(map(int, header['SIZE']))
        types = header['TYPE']
        counts = list(map(int, header.get('COUNT', ['1']*len(names))))
        points = int(header.get('POINTS', [str(int(header['WIDTH'][0]) * int(header['HEIGHT'][0]))])[0])
        stride = max(1, math.ceil(points / max_points))
        if header['DATA'][0] == 'binary':
            formats, unique_names = [], []
            for i, (name, size, kind, count) in enumerate(zip(names, sizes, types, counts)):
                unique_names.append(name if name not in unique_names else f'padding_{i}')
                dtype = np.dtype('<' + {'F': 'f', 'I': 'i', 'U': 'u'}[kind] + str(size))
                formats.append(dtype if count == 1 else (dtype, (count,)))
            dtype = np.dtype({'names': unique_names, 'formats': formats})
            raw = np.memmap(path, dtype=dtype, mode='r', offset=offset, shape=(points,))
            xyz = np.column_stack([raw[name][::stride] for name in ('x', 'y', 'z')])
        elif header['DATA'][0] == 'ascii':
            import itertools
            columns = [sum(counts[:names.index(name)]) for name in ('x', 'y', 'z')]
            xyz = np.loadtxt(itertools.islice(stream, 0, None, stride), usecols=columns, ndmin=2)
        else:
            raise ValueError('暂不支持binary_compressed参考PCD，请转为binary/ascii或指定其他参考图')
    return xyz[np.isfinite(xyz).all(axis=1)]


def select_map(session, explicit):
    metadata_path = session / 'run_metadata.json'
    metadata = json.loads(metadata_path.read_text()) if metadata_path.is_file() else {}
    if explicit:
        return Path(explicit).expanduser().resolve(), 'explicit', metadata
    if metadata.get('map_pcd'):
        return Path(metadata['map_pcd']).expanduser(), 'run_metadata', metadata
    return ROOT.parent / 'kiss_localization_ws1.1/src/localization/maps/3_part.pcd', 'legacy_default_unverified', metadata


def prepare_font():
    for candidate in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
                      '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc']:
        if Path(candidate).is_file():
            font_manager.fontManager.addfont(candidate)
            plt.rcParams['font.family'] = font_manager.FontProperties(fname=candidate).get_name()
            break
    plt.rcParams['axes.unicode_minus'] = False


def draw_path(ax, data, three_d=False):
    path, markers = data['path'], data['markers']
    historical = data['historical_path']
    blue_label = '最新校正重投影轨迹' if data['frame'] == 'map' else '原始里程计轨迹'
    if not len(path):
        if three_d:
            ax.text2D(.1, .5, '本轮未保存里程计路径，无法重建', transform=ax.transAxes)
        else:
            ax.text(.5, .5, '本轮未保存里程计路径，无法重建', ha='center', transform=ax.transAxes)
        return
    if three_d:
        ax.plot(*path.T, color='#008fb5', lw=1.4, label=blue_label)
        if len(historical):
            ax.plot(*historical.T, color='#e0b000', lw=2, label='历史定位输出轨迹（黄色）')
        ax.scatter(*path[0], color='#2667cc', s=45, label='起点')
        ax.scatter(*path[-1], color='#d64d3c', s=55, marker='X', label='结束位置')
        if markers:
            ax.scatter(*np.array([m['point'] for m in markers]).T, color='#209e46', s=30,
                       label='已接受匹配')
        ax.set_box_aspect(np.maximum(np.ptp(np.vstack([path, historical]), axis=0), 1))
        ax.view_init(elev=27, azim=-62)
        ax.set_zlabel(data['frame'] + ' Z / m', labelpad=2)
    else:
        ax.plot(path[:, 0], path[:, 1], color='#00a8ce', lw=1.7, label=blue_label)
        if len(historical):
            ax.plot(historical[:, 0], historical[:, 1], color='#e0b000', lw=2,
                    label='历史定位输出轨迹（黄色）', zorder=3)
        ax.scatter(*path[0, :2], color='#2667cc', s=45, label='起点', zorder=4)
        ax.scatter(*path[-1, :2], color='#d64d3c', marker='X', s=60, label='结束位置', zorder=4)
        if markers:
            points = np.array([m['point'] for m in markers])
            ax.scatter(points[:, 0], points[:, 1], color='#209e46', s=32,
                       edgecolor='white', linewidth=.5, label='已接受匹配', zorder=5)
        ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel(data['frame'] + ' X / m')
    ax.set_ylabel(data['frame'] + ' Y / m')
    ax.grid(alpha=.2)


def render_session(session, map_pcd=None, until_ns=None):
    session = Path(session).expanduser().resolve()
    prepare_font()
    data = load_data(session, until_ns)
    folder = session / 'visualizations' / datetime.now(TZ).strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True, exist_ok=False)
    reference, map_source, run_metadata = select_map(session, map_pcd)
    notes = []
    if data['historical_note'] and len(data['historical_path']):
        notes.append(data['historical_note'])
    for required in ['attempts.csv', 'odom_path.csv']:
        if not (session / required).is_file():
            notes.append('缺少' + required + '，相应路径/接受结果不能重建。')
    if map_source == 'legacy_default_unverified':
        notes.append('旧记录未保存地图元数据，背景使用默认五号楼参考图；需核对是否为当时用图。')
    if map_source == 'run_metadata' and reference.is_file():
        if (run_metadata.get('map_size_bytes') != reference.stat().st_size or
                run_metadata.get('map_mtime_ns') != reference.stat().st_mtime_ns):
            notes.append('当前地图大小/时间与运行时记录不一致，背景需核对。')
    points = None
    try:
        points = read_reference(reference)
    except Exception as error:
        notes.append('参考地图未绘制：' + str(error))
    title = ('截至 ' + datetime.fromtimestamp(until_ns / 1e9, TZ).strftime('%Y-%m-%d %H:%M:%S')
             if until_ns is not None else '本轮全程（以已保存记录为准）')
    frame_note = ('青线：最新校正重投影；黄线：历史定位输出；绿点：各次实际应用位置。'
                  if data['frame'] == 'map' else '本轮没有可用的已接受全局变换，路径仅在odom坐标系显示。')
    fig = plt.figure(figsize=(13, 6.6), constrained_layout=True)
    left = fig.add_subplot(121, projection='3d')
    right = fig.add_subplot(122)
    draw_path(left, data, True)
    draw_path(right, data)
    left.set_title('三维估计路径')
    right.set_title('俯视路径')
    if len(data['path']):
        left.legend(fontsize=8, loc='upper left')
        right.legend(fontsize=8)
    fig.suptitle(title, fontsize=15)
    fig.supxlabel(frame_note + '\n路径和接受标记均非真值；路径延伸不等于持续匹配成功。', fontsize=9)
    fig.savefig(folder / '路径图.png', dpi=150)
    plt.close(fig)

    fig, (ax, panel) = plt.subplots(1, 2, figsize=(13, 6.8), constrained_layout=True,
                                    gridspec_kw={'width_ratios': [1.3, 1]})
    if points is not None and len(points):
        if data['frame'] == 'map' and len(data['path']):
            region = np.vstack([data['path'], data['historical_path'], *[m['point'][None, :] for m in data['markers']]])
            low, high = region[:, :2].min(axis=0)-8, region[:, :2].max(axis=0)+8
            nearby = points[((points[:, :2] >= low) & (points[:, :2] <= high)).all(axis=1)]
            if len(nearby):
                points = nearby
            else:
                notes.append('参考图附近未找到估计路径对应区域，需核对地图覆盖或定位结果。')
        if len(points) > 140000:
            points = points[np.linspace(0, len(points)-1, 140000).astype(int)]
        ax.scatter(points[:, 0], points[:, 1], s=.6, color='#788897', alpha=.27,
                   rasterized=True, label='已有全局参考地图（抽样）')
    if data['frame'] == 'map':
        draw_path(ax, data)
        ax.set_title('全局参考地图上的估计路径')
    else:
        ax.set_title('全局参考地图｜尚无可叠加的全局路径')
        ax.set(xlabel='map X / m', ylabel='map Y / m')
        ax.set_aspect('equal', adjustable='datalim')
        ax.text(.04, .04, '未接受全局定位，odom路径见另一张图', transform=ax.transAxes,
                bbox=dict(facecolor='white', alpha=.9), fontsize=9)
    if points is None:
        ax.text(.04, .93, '参考地图不可用，未伪造背景', transform=ax.transAxes, fontsize=10)
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(fontsize=8)
    panel.axis('off')
    panel.text(0, .98, f"状态事件 {len(data['events'])} 条，接受 {len(data['accepted'])} 次\n"
               f"可重建接受标记 {len(data['markers'])} 个", va='top', fontsize=12)
    shown = data['accepted'][-8:]
    if shown:
        table_rows = []
        for row in shown:
            mode = ('手动确认' if row['reason'] == 'candidate_accepted' else
                    '全局' if float(row['coarse_inliers']) > 0 else '先验')
            table_rows.append([row['attempt'], datetime.fromtimestamp(int(row['completed_ns'])/1e9, TZ).strftime('%H:%M:%S'),
                               mode, f"{float(row['overlap_percent']):.2f}%",
                               f"{float(row['registration_ms'])/1000:.3f}s"])
        table = panel.table(cellText=table_rows, colLabels=['事件', '完成时间', '模式', '重叠率', '耗时'],
                            cellLoc='center', bbox=[0, .39, 1, .47], colWidths=[.12, .25, .15, .23, .25])
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        for (i, j), cell in table.get_celld().items():
            cell.set_edgecolor('#d0d9dd')
            if i == 0:
                cell.set_facecolor('#e9f1f5')
        if len(data['accepted']) > 8:
            panel.text(0, .365, '表中显示最近8次；图上保留全部可重建标记。', fontsize=9)
    else:
        panel.text(0, .66, '本轮没有已接受的定位结果。', fontsize=12)
    last_time = (datetime.fromtimestamp(int(data['last']['completed_ns'])/1e9, TZ).strftime('%H:%M:%S')
                 if data['last'] else '无')
    note = f'最后可用接受时间：{last_time}\n背景：{reference.name}\n参考图不是本轮新扫描点云。\n定位地点仍需与实际路线核对。'
    note += '\n青线：最新校正重投影\n黄线：历史定位输出\n绿点：实际应用标记'
    if data['missing_markers']:
        note += f"\n{len(data['missing_markers'])}次缺少对应时刻里程计/变换，未虚构标记。"
    if notes:
        note += '\n' + '\n'.join(notes)
    panel.text(0, .31, note, fontsize=9, va='top', linespacing=1.5, wrap=True)
    fig.suptitle(title + '｜参考地图与估计路径', fontsize=15)
    fig.savefig(folder / '全局参考地图与路径.png', dpi=150)
    plt.close(fig)
    with (folder / '估计路径.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['stamp_ns', 'x', 'y', 'z', 'frame'])
        writer.writerows([[stamp, *point, data['frame']] for stamp, point in zip(data['stamps'], data['path'])])
    with (folder / '历史定位输出路径.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['stamp_ns', 'x', 'y', 'z', 'frame'])
        writer.writerows([[stamp, *point, 'map'] for stamp, point in
                          zip(data['historical_stamps'], data['historical_path'])])
    metadata = dict(session=str(session), until_ns=until_ns, path_frame=data['frame'],
                    poses=len(data['path']), state_events=len(data['events']),
                    accepted=len(data['accepted']), markers=len(data['markers']),
                    missing_marker_events=data['missing_markers'], map_pcd=str(reference),
                    map_source=map_source, notes=notes,
                    historical_path_source=data['historical_source'],
                    historical_path_poses=len(data['historical_path']))
    (folder / 'visualization_metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
    (session / 'latest_visualization.txt').write_text(str(folder)+'\n')
    print(f'可视化结果：{folder}')
    return folder


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--map-pcd', type=Path)
    parser.add_argument('--until', help='ISO时间，必须包含时区')
    args = parser.parse_args()
    render_session(args.session, args.map_pcd, cutoff_ns(args.until))


if __name__ == '__main__':
    main()
