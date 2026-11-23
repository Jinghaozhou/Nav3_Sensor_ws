"""Offline bag fixture verifies exact cloud identity, stage separation and span joins."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import rosbag2_py
from builtin_interfaces.msg import Time
from kiss_matcher_localization.msg import LocalizationInput
from rclpy.serialization import serialize_message
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from export_attempt_windows import export
from local_map_window import build_cloud


class Export(unittest.TestCase):
    def test_stage_identity_metadata_order_and_missing_span(self):
        with tempfile.TemporaryDirectory(prefix='window_export_fixture_') as temp:
            root = Path(temp)
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(root / 'bag'), storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('', ''))
            for name, typ in [('/localization/input_snapshot', 'kiss_matcher_localization/msg/LocalizationInput'),
                              ('/fast_lio/window_status', 'std_msgs/msg/String')]:
                writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format='cdr'))
            original = []
            for i, stage in enumerate(['global', 'prior', 'fallback_global', 'global']):
                m = LocalizationInput()
                m.attempt = i + 1
                m.stage = stage
                m.source_topic = '/fast_lio/local_map' if stage == 'prior' else '/fast_lio/local_map_global'
                m.selected_stamp.sec = 140 + i
                xyz = np.array([[i, 1, 2], [i, 3, 4], [i, 5, 6]], np.float32)
                original.append(xyz)
                m.cloud = build_cloud(Time(sec=130 if i < 3 else 131), 'odom', xyz, np.array([7, 8, 9]))
                writer.write('/localization/input_snapshot', serialize_message(m), (140 + i) * 10**9)
            # Status appears later in bag order. Both source windows share end stamp and point count.
            status = dict(cloud_stamp_ns=130*10**9, window_start_ns=100*10**9,
                          long_window_start_ns=10*10**9, long_requested_seconds=120.,
                          local_topic='/fast_lio/local_map', global_topic='/fast_lio/local_map_global',
                          points=3, long_points=3, frames=301, retained_frames=1201, requested_seconds=30.)
            writer.write('/fast_lio/window_status', serialize_message(String(data=json.dumps(status))), 144*10**9)
            del writer
            rows = export(root, root / 'exports')
            self.assertEqual([r['actual_seconds'] for r in rows[:3]], [120., 30., 120.])
            self.assertEqual(rows[-1]['span_status'], 'missing')
            self.assertEqual(rows[-1]['actual_seconds'], '')
            for i, row in enumerate(rows):
                data = (root / 'exports' / row['pcd']).read_bytes().split(b'DATA binary\n', 1)[1]
                np.testing.assert_array_equal(np.frombuffer(data, '<f4').reshape(-1, 4)[:, :3], original[i])
            limited = export(root, root / 'limited', until_ns=141*10**9)
            self.assertEqual(len(limited), 2)


if __name__ == '__main__':
    unittest.main()
