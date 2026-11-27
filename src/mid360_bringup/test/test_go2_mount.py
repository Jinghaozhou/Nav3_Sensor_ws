"""Physical transform and launch contracts; run in the sourced Nav3 workspace."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml
from launch import LaunchContext
from mid360_bringup.mount_tf import go2_transforms

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('mapping_launch', ROOT/'launch/mid360_mapping.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


class MountTest(unittest.TestCase):
    def test_physical_composition_and_axis_direction(self):
        t = np.array([-.011, -.02329, .04412])
        mount = yaml.safe_load((ROOT/'config/go2_mount.yaml').read_text())
        imu_base, base_lidar = go2_transforms(t, np.eye(3), mount)
        expected = np.eye(4); expected[:3, 3] = t
        np.testing.assert_allclose(imu_base @ base_lidar, expected, atol=1e-12)
        np.testing.assert_allclose(base_lidar @ [0, 0, 0, 1], [*mount['xyz'], 1])
        # Positive pitch maps lidar forward toward body downward Z.
        np.testing.assert_allclose(base_lidar[:3, :3] @ [1, 0, 0],
                                   [np.cos(np.deg2rad(20)), 0, -np.sin(np.deg2rad(20))], atol=1e-8)
        # A moving IMU retains the same world cloud after adding the body frame.
        world_imu = np.eye(4); world_imu[:3, 3] = [1, 2, 3]
        point = [.4, -.2, 1.3, 1]
        np.testing.assert_allclose(world_imu @ imu_base @ base_lidar @ point,
                                   world_imu @ expected @ point, atol=1e-12)

    def test_invalid_mount_is_rejected(self):
        for mount in ({'xyz': [0, 0], 'rpy': [0, 0, 0]},
                      {'xyz': [0, 0, 0], 'rpy': [0, float('nan'), 0]}):
            with self.assertRaises(ValueError):
                go2_transforms(np.zeros(3), np.eye(3), mount)

    def context(self, model, output):
        ctx = LaunchContext()
        ctx.launch_configurations.update(dict(
            robot_model=model, config_file=str(ROOT/'config/mid360_robot.yaml'),
            mount_config=str(ROOT/'config/go2_mount.yaml'),
            driver_config=str(ROOT/'config/MID360_config.json'),
            map_output_dir=output, driver='false', use_sim_time='true',
            monitor='false', local_map='false', keyframe_candidates='false', rviz='false'))
        return ctx

    def test_launch_has_single_parent_and_correct_fastlio_body(self):
        with tempfile.TemporaryDirectory() as output:
            for model, body, edges in (
                ('go2', 'imu_link', {('imu_link', 'base_link'), ('base_link', 'lidar')}),
                ('imu', 'base_link', {('base_link', 'imu_link'), ('base_link', 'lidar')})):
                ctx = self.context(model, output)
                with patch.object(launch, 'Node', side_effect=lambda **kw: kw):
                    nodes = launch.start(ctx)
                static = [n for n in nodes if n['package'] == 'tf2_ros']
                got = set()
                for n in static:
                    args = n['arguments']
                    got.add((args[args.index('--frame-id')+1], args[args.index('--child-frame-id')+1]))
                self.assertEqual(got, edges)
                self.assertEqual(len(static), len({child for _, child in got}))
                fastlio = next(n for n in nodes if n['package'] == 'fast_lio')
                self.assertEqual(fastlio['parameters'][-1]['body_frame'], body)

    def test_driver_pretransform_is_rejected(self):
        with tempfile.TemporaryDirectory() as output:
            ctx = self.context('go2', output)
            driver = json.loads((ROOT/'config/MID360_config.json').read_text())
            driver['lidar_configs'][0]['extrinsic_parameter']['pitch'] = 20.
            path = Path(output)/'driver.json'; path.write_text(json.dumps(driver))
            ctx.launch_configurations['driver_config'] = str(path)
            with self.assertRaisesRegex(ValueError, 'zero Livox'):
                launch.start(ctx)


if __name__ == '__main__':
    unittest.main()
