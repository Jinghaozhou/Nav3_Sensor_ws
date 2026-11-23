#!/usr/bin/env python3
"""Offline short-cloud reference export. No ROS publishers or online activation.

Accepted localization poses are provisional labels, never surveyed ground truth.
Point clouds are expressed in timestamp-matched base_link, NOT gravity aligned.
This is a reference-data library; ScanContext descriptors/retrieval are not built.
"""
import argparse
from bisect import bisect_left, bisect_right
from collections import Counter, deque
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024**2), b''):
            h.update(block)
    return h.hexdigest()


def pose(position, quaternion):
    q = np.asarray(quaternion, float)
    if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > .01:
        raise ValueError('Invalid pose quaternion')
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(q).as_matrix()
    t[:3, 3] = position
    if not np.isfinite(t).all():
        raise ValueError('Invalid pose position')
    return t


def message_pose(p):
    return pose([p.position.x, p.position.y, p.position.z],
                [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])


def interpolate(stamps, poses, stamp, max_gap_ns):
    i = bisect_left(stamps, stamp)
    if i < len(stamps) and stamps[i] == stamp:
        return poses[i]
    if i == 0 or i == len(stamps) or stamps[i] - stamps[i-1] > max_gap_ns:
        return None
    a, b = poses[i-1], poses[i]
    f = (stamp - stamps[i-1]) / (stamps[i] - stamps[i-1])
    r = Rotation.from_matrix(a[:3, :3].T @ b[:3, :3])
    out = np.eye(4)
    out[:3, :3] = a[:3, :3] @ Rotation.from_rotvec(r.as_rotvec() * f).as_matrix()
    out[:3, 3] = (1-f)*a[:3, 3] + f*b[:3, 3]
    return out


def transformed(points, transform):
    return points @ transform[:3, :3].T + transform[:3, 3]


def reader(bag, topics):
    import rosbag2_py
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


def write_pcd(path, xyz):
    header = ('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
              f'COUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(xyz)}\nDATA binary\n')
    with path.open('wb') as f:
        f.write(header.encode())
        f.write(np.asarray(xyz, dtype='<f4').tobytes())


def build(args):
    import yaml
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from kiss_matcher_localization.msg import LocalizationResult
    from local_map_window import parse_cloud, voxel_indices

    session = args.session.expanduser().resolve()
    metadata = json.loads((session/'run_metadata.json').read_text())
    map_hash = sha256(args.map)
    if metadata.get('map_sha256') != map_hash:
        raise ValueError('Session/map SHA mismatch: historical poses cannot label this map. '
                         'Record a new session with the current map; no automatic migration.')
    cfg = yaml.safe_load((session/'source_configs/mid360_robot.yaml').read_text())
    mapping = cfg['/**']['ros__parameters']['mapping']
    if mapping.get('extrinsic_est_en', True):
        raise ValueError('Online estimated extrinsics require per-frame extrinsic evidence')
    offset = np.asarray(mapping['extrinsic_T'], float)
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError('Missing valid LiDAR origin in base frame')
    r = reader(session/'bag', ['/fast_lio/odom', '/localization/status'])
    odom, accepted = {}, {}
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == '/fast_lio/odom':
            m = deserialize_message(data, Odometry)
            if (m.header.frame_id, m.child_frame_id) != ('odom', 'base_link'):
                raise ValueError('Unexpected odometry frame convention')
            odom[ns(m.header.stamp)] = message_pose(m.pose.pose)
        else:
            m = deserialize_message(data, LocalizationResult)
            if not (m.applied and m.matched):
                continue
            t = m.candidate_map_to_odom
            if (t.header.frame_id, t.child_frame_id) != ('map', 'odom'):
                raise ValueError('Unexpected map transform frames')
            v, q = t.transform.translation, t.transform.rotation
            accepted[ns(m.completed_stamp)] = (pose([v.x,v.y,v.z], [q.x,q.y,q.z,q.w]),
                                               float(m.overlap_percent), ns(m.local_map_stamp))
    if not odom or not accepted:
        raise ValueError('Need odometry AND actually applied localization in bag')
    stamps, completed = sorted(odom), sorted(accepted)
    poses = [odom[t] for t in stamps]
    out = args.output or ROOT/'reference_maps/keyframe_libraries'/args.region/session.name
    out = out.expanduser().absolute()
    out.mkdir(parents=True, exist_ok=False)
    (out/'clouds').mkdir()
    info = dict(schema_version=1, state='building', region_claim=args.region,
                location_note=args.location_note, device=args.device, map_sha256=map_hash,
                map_path=str(args.map.resolve()), source_session=str(session),
                source_metadata_sha256=sha256(session/'run_metadata.json'),
                source_config_sha256=sha256(session/'source_configs/mid360_robot.yaml'),
                created=datetime.now().astimezone().isoformat(),
                pose_status='accepted_localization_unreviewed_not_ground_truth',
                cloud_frame='base_link_at_end_stamp', gravity_aligned=False,
                descriptor_built=False, online_enabled=False,
                parameters={k:v for k,v in vars(args).items() if not isinstance(v, Path)})
    def save_info():
        (out/'manifest.json').write_text(json.dumps(info, ensure_ascii=False, indent=2)+'\n')
    save_info()
    r = reader(session/'bag', ['/fast_lio/local_cloud'])
    window, rows, skipped = deque(), [], Counter()
    origin, previous_stamp, last_saved = None, None, None
    try:
        while r.has_next():
            _, data, _ = r.read_next()
            m = deserialize_message(data, PointCloud2)
            stamp = ns(m.header.stamp)
            if previous_stamp is not None and stamp <= previous_stamp:
                raise ValueError('Cloud timestamps reset/repeated; split recording before export')
            if previous_stamp is not None and stamp-previous_stamp > args.max_cloud_gap*1e9:
                window.clear()
                skipped['cloud_gap_reset'] += 1
            previous_stamp = stamp
            if origin is None:
                origin = stamp
            elapsed = (stamp-origin)/1e9
            if elapsed > args.end_offset:
                break
            if elapsed < args.start_offset:
                continue
            if m.header.frame_id != 'odom':
                raise ValueError('Expected odom-frame processed cloud')
            ob = interpolate(stamps, poses, stamp, int(args.max_odom_gap*1e9))
            if ob is None:
                window.clear(); skipped['odom_unavailable'] += 1; continue
            xyz, _ = parse_cloud(m)
            window.append((stamp, xyz))
            while window and stamp-window[0][0] > args.window_seconds*1e9:
                window.popleft()
            j = bisect_right(completed, stamp)-1
            if j < 0 or (stamp-completed[j])/1e9 > args.max_pose_age:
                skipped['no_recent_applied_pose'] += 1; continue
            if stamp-window[0][0] < .8*args.window_seconds*1e9:
                skipped['window_warmup'] += 1; continue
            mt, overlap, pose_cloud_ns = accepted[completed[j]]
            if j+1<getattr(args,'minimum_applied_count',1) or overlap<getattr(args,'minimum_applied_overlap',0.):
                skipped['applied_quality_warmup'] += 1;continue
            mb = mt @ ob
            if last_saved:
                old_stamp, old_pose = last_saved
                distance = np.linalg.norm(mb[:3,3]-old_pose[:3,3])
                angle = np.rad2deg(Rotation.from_matrix(old_pose[:3,:3].T@mb[:3,:3]).magnitude())
                if (stamp-old_stamp)/1e9 < args.min_interval or (distance < args.distance and angle < args.angle):
                    skipped['similar_pose'] += 1; continue
            # All stored frames are already deskewed/transformed into the same odom.
            points = transformed(np.concatenate([x for _,x in window]), np.linalg.inv(ob))
            radius = np.linalg.norm(points-offset, axis=1)
            points = points[(radius >= args.blind) & (radius <= args.radius)]
            points = points[voxel_indices(points, args.voxel)]
            if len(points) < args.min_points:
                skipped['too_few_points'] += 1; continue
            if __import__('shutil').disk_usage(out).free < points.nbytes + 1024**3:
                raise RuntimeError('Less than 1GiB reserve')
            ident = f'kf_{len(rows):05d}'
            relative = f'clouds/{ident}.pcd'
            write_pcd(out/relative, points)
            row = dict(id=ident, stamp_ns=stamp, start_ns=window[0][0],
                       actual_span_seconds=(stamp-window[0][0])/1e9, frames=len(window),
                       point_count=len(points), pcd=relative, pcd_sha256=sha256(out/relative),
                       map_T_base=mb.tolist(), map_T_odom=mt.tolist(), odom_T_base=ob.tolist(),
                       lidar_origin_base=offset.tolist(), applied_completed_ns=completed[j],
                       applied_input_stamp_ns=pose_cloud_ns, applied_overlap_percent=overlap,
                       applied_age_seconds=(stamp-completed[j])/1e9,
                       region_claim=args.region, location_note=args.location_note,
                       verified=False)
            rows.append(row)
            with (out/'keyframes.jsonl').open('a') as f:
                f.write(json.dumps(row, ensure_ascii=False)+'\n')
            last_saved = stamp, mb
        with (out/'review.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['id','stamp_ns','x','y','z','region_claim','verified_region','verified_floor',
                        'pose_checked','use_for_reference','reviewer','notes'])
            for row in rows:
                w.writerow([row['id'],row['stamp_ns'],*[row['map_T_base'][i][3] for i in range(3)],
                            args.region,'','','false','false','',''])
        info.update(state='exported_unreviewed' if rows else 'empty', keyframes=len(rows),
                    skipped=dict(skipped), cloud_time_origin_ns=origin,
                    index_sha256=sha256(out/'keyframes.jsonl') if rows else None)
        save_info()
        print(json.dumps(dict(output=str(out),state=info['state'],keyframes=len(rows),
                              skipped=dict(skipped)), ensure_ascii=False, indent=2))
        if not rows:
            raise ValueError('No usable keyframes; inspect manifest, interval and applied poses')
        return out
    except BaseException as exc:
        if info['state']=='building':
            info.update(state='incomplete', error=str(exc),keyframes=len(rows)); save_info()
        raise


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('session', type=Path)
    p.add_argument('--map',type=Path,default=__import__('workspace_paths').map_file())
    p.add_argument('--region',choices=[f'space_{i}' for i in range(1,14)],required=True)
    p.add_argument('--device',required=True,help='Device identifier; mark unconfirmed serial explicitly')
    p.add_argument('--location-note',required=True,help='Actual floor/platform/route; reviewed separately')
    p.add_argument('--start-offset',type=float,required=True,help='Seconds from first processed cloud')
    p.add_argument('--end-offset',type=float,required=True)
    p.add_argument('--output',type=Path)
    for name,default in [('window-seconds',1.),('radius',5.),('blind',.5),('voxel',.1),
                         ('distance',.5),('angle',20.),('min-interval',2.),
                         ('max-pose-age',10.),('max-odom-gap',.15),('max-cloud-gap',.3)]:
        p.add_argument('--'+name,type=float,default=default)
    p.add_argument('--min-points',type=int,default=1000)
    a = p.parse_args()
    values=[getattr(a,k.replace('-','_')) for k in ['window-seconds','radius','voxel','distance',
            'angle','min-interval','max-pose-age','max-odom-gap','max-cloud-gap']]
    if not all(np.isfinite(x) and x>0 for x in values) or not (0<=a.blind<a.radius) or not (
            np.isfinite(a.start_offset) and np.isfinite(a.end_offset) and 0<=a.start_offset<a.end_offset) or a.min_points<1:
        p.error('Invalid interval or sampling parameters')
    build(a)


if __name__ == '__main__':
    main()
