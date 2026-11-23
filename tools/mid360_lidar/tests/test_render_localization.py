"""Verify cutoff reconstruction, no-success handling and isolated output directories."""
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from render_localization import load_data, read_reference, render_session


def fixture(folder):
    folder.mkdir()
    with (folder / 'odom_path.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['stamp_ns', 'x', 'y', 'z'])
        writer.writerows([[8*10**9, 0, 0, 0], [10*10**9, 1, 0, 0],
                          [12*10**9, 2, 0, 0], [20*10**9, 3, 0, 0]])
    fields = ['attempt', 'completed_ns', 'cloud_ns', 'applied', 'reason',
              'coarse_inliers', 'overlap_percent', 'registration_ms'] + [f'T{i}{j}' for i in range(4) for j in range(4)]
    rows = []
    for attempt, second, shift in [(1, 10, 10), (2, 20, 100)]:
        T = np.eye(4)
        T[0, 3] = shift
        row = dict(attempt=attempt, completed_ns=second*10**9, cloud_ns=second*10**9,
                   applied='True', reason='applied', coarse_inliers=11,
                   overlap_percent=80, registration_ms=100)
        row.update({f'T{i}{j}': T[i, j] for i in range(4) for j in range(4)})
        rows.append(row)
    with (folder / 'attempts.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return fields


class Rendering(unittest.TestCase):
    def test_cutoff_excludes_future_transform_and_preserves_sources(self):
        with tempfile.TemporaryDirectory(prefix='render_fixture_') as temp:
            root = Path(temp)
            fixture(root / 'scan')
            session = root / 'scan'
            before = (session / 'odom_path.csv').read_bytes()
            data = load_data(session, 15*10**9)
            np.testing.assert_array_equal(data['path'][:, 0], [10, 11, 12])
            np.testing.assert_array_equal(data['historical_path'][:, 0], [11, 12])
            self.assertEqual(len(data['accepted']), 1)
            self.assertEqual(data['markers'][0]['point'][0], 11)
            # Field order is intentionally non-XYZ to test PCD decoding.
            mapfile = root / 'map.pcd'
            header = b'FIELDS intensity z x y\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA binary\n'
            mapfile.write_bytes(header + np.array([[1, 0, 10, 0], [1, 0, 11, 1], [1, 0, 12, 0]], '<f4').tobytes())
            np.testing.assert_array_equal(read_reference(mapfile)[:, 0], [10, 11, 12])
            a = render_session(session, mapfile, 15*10**9)
            (session / 'run_metadata.json').write_text(json.dumps({
                'map_pcd': str(mapfile), 'map_size_bytes': mapfile.stat().st_size,
                'map_mtime_ns': mapfile.stat().st_mtime_ns}))
            b = render_session(session, None, 15*10**9)
            self.assertNotEqual(a, b)
            self.assertTrue((a / '全局参考地图与路径.png').is_file())
            self.assertTrue((b / '路径图.png').is_file())
            self.assertEqual(json.loads((a / 'visualization_metadata.json').read_text())['accepted'], 1)
            self.assertEqual(json.loads((b / 'visualization_metadata.json').read_text())['map_source'], 'run_metadata')
            self.assertEqual((session / 'odom_path.csv').read_bytes(), before)

    def test_no_accepted_match_uses_odom_not_identity_map_transform(self):
        with tempfile.TemporaryDirectory(prefix='unlocalized_fixture_') as temp:
            session = Path(temp) / 'scan'
            fields = fixture(session)
            with (session / 'attempts.csv').open('w', newline='') as stream:
                csv.DictWriter(stream, fieldnames=fields).writeheader()
            data = load_data(session)
            self.assertEqual(data['frame'], 'odom')
            self.assertFalse(data['markers'])
            self.assertEqual(len(data['historical_path']), 0)
            out = render_session(session, Path(temp) / 'missing_map.pcd')
            metadata = json.loads((out / 'visualization_metadata.json').read_text())
            self.assertEqual(metadata['accepted'], 0)
            self.assertEqual(metadata['path_frame'], 'odom')
            self.assertTrue(metadata['notes'])

    def test_historical_corrections_start_at_completion_not_window_time(self):
        with tempfile.TemporaryDirectory(prefix='historical_path_') as temp:
            session = Path(temp) / 'scan'
            fields = fixture(session)
            with (session / 'attempts.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            rows[0]['completed_ns'] = str(11*10**9)
            rows[1]['completed_ns'] = str(21*10**9)
            with (session / 'attempts.csv').open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(rows)
            data = load_data(session)
            np.testing.assert_array_equal(data['historical_stamps'], [12*10**9, 20*10**9])
            np.testing.assert_array_equal(data['historical_path'][:, 0], [12, 13])
            np.testing.assert_array_equal(data['path'][:, 0], [100, 101, 102, 103])


if __name__ == '__main__':
    unittest.main()
