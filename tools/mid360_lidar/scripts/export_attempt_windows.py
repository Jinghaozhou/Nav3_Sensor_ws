#!/usr/bin/env python3
"""Export exact selected windows and their timestamp-matched coverage from a ROS bag."""
import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import shutil

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from kiss_matcher_localization.msg import LocalizationInput
from std_msgs.msg import String
from local_map_window import parse_cloud


def ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def coverage(status, topic):
    end = status.get('cloud_stamp_ns')
    if end is None:
        return None
    if topic == status.get('global_topic'):
        start = status.get('long_window_start_ns')
        result = {'frames': status['retained_frames'], 'requested_seconds': status['long_requested_seconds']}
    elif topic == status.get('local_topic'):
        start = status.get('window_start_ns')
        result = {'frames': status['frames'], 'requested_seconds': status['requested_seconds']}
    else:
        return None
    if start is None:
        return None
    result.update(window_start_ns=start, cloud_stamp_ns=end, actual_seconds=(end - start) / 1e9)
    return result


def export(session, output, until_ns=None):
    bag = session / 'bag' if (session / 'bag/metadata.yaml').is_file() else session
    output.mkdir(parents=True, exist_ok=False)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[
        '/localization/input_snapshot', '/fast_lio/window_status']))
    index = {}
    rows = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/fast_lio/window_status':
            status = json.loads(deserialize_message(data, String).data)
            for source, point_key in [(status.get('local_topic'), 'points'),
                                       (status.get('global_topic'), 'long_points')]:
                if not source:
                    continue
                value = coverage(status, source)
                if value is not None:
                    key = (source, status['cloud_stamp_ns'], status[point_key])
                    bucket = index.setdefault(key, [])
                    if value not in bucket:
                        bucket.append(value)
            continue
        capture = deserialize_message(data, LocalizationInput)
        if until_ns is not None and ns(capture.selected_stamp) > until_ns:
            continue
        cloud = capture.cloud
        stamp = ns(cloud.header.stamp)
        # Each envelope carries its own cloud, so repeated stamps and fallback stages stay distinct.
        filename = f'attempt_{capture.attempt:06d}_{capture.stage}_{len(rows)+1:06d}.pcd'
        xyz, intensity = parse_cloud(cloud)
        values = np.column_stack([xyz, intensity]).astype('<f4')
        if shutil.disk_usage(output).free - values.nbytes < 2**30:
            raise RuntimeError('Less than 1GiB would remain; choose --output on another disk. '
                               'The original bag and already exported PCDs are preserved.')
        header = (f'VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n'
                  f'COUNT 1 1 1 1\nWIDTH {len(values)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                  f'POINTS {len(values)}\nDATA binary\n')
        with (output / filename).open('wb') as stream:
            stream.write(header.encode())
            stream.write(values.tobytes())
        rows.append({'attempt': capture.attempt, 'stage': capture.stage,
                     'source_topic': capture.source_topic, 'selected_ns': ns(capture.selected_stamp),
                     'cloud_stamp_ns': stamp, 'frame': cloud.header.frame_id,
                     'message_points': cloud.width * cloud.height, 'exported_points': len(values),
                     'pcd': filename})
    # Read all metadata before joining: DDS order between different topics is not guaranteed.
    for row in rows:
        matches = index.get((row['source_topic'], row['cloud_stamp_ns'], row['message_points']), [])
        if len(matches) == 1:
            row.update(matches[0], span_status='matched')
        else:
            row.update(window_start_ns='', actual_seconds='', frames='', requested_seconds='',
                       span_status='missing' if not matches else 'ambiguous')
    fields = ['attempt', 'stage', 'source_topic', 'selected_ns', 'cloud_stamp_ns',
              'window_start_ns', 'actual_seconds', 'requested_seconds', 'frames',
              'message_points', 'exported_points', 'frame', 'span_status', 'pcd']
    with (output / 'windows.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    summary = {'snapshots': len(rows), 'span_matched': sum(r['span_status'] == 'matched' for r in rows),
               'span_unresolved': sum(r['span_status'] != 'matched' for r in rows),
               'note': 'PCDs are exact selected window XYZ/intensity, after window voxel filtering '
                       'and before localization voxel filtering. Raw sensor messages remain in bag. '
                       'Empty span means missing/ambiguous metadata, not zero coverage.'}
    (output / 'export_summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path, help='Experiment folder or its bag subdirectory')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--until', help='Optional ISO cutoff with timezone, e.g. 2026-10-24T17:18:00+08:00')
    args = parser.parse_args()
    cutoff = None
    if args.until:
        value = datetime.fromisoformat(args.until)
        if value.tzinfo is None:
            parser.error('--until must include a timezone')
        cutoff = int(value.timestamp() * 1e9)
    export(args.session, args.output or args.session / 'window_exports', cutoff)


if __name__ == '__main__':
    main()
