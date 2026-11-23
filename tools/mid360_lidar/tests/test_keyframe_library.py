#!/usr/bin/env python3
"""Offline ROS bag tests. No nodes, publishers, hardware or original data writes."""
import argparse
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import rosbag2_py
from rclpy.serialization import serialize_message
from nav_msgs.msg import Odometry
from kiss_matcher_localization.msg import LocalizationResult
from builtin_interfaces.msg import Time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from local_map_window import build_cloud
from build_keyframe_library import build, interpolate, sha256, transformed


class KeyframeLibraryTest(unittest.TestCase):
    def make_session(self, root, applied=True):
        session = root/'session'; session.mkdir()
        source = session/'source_configs'; source.mkdir()
        (source/'mid360_robot.yaml').write_text('/**:\n  ros__parameters:\n    mapping:\n'
            '      extrinsic_est_en: false\n      extrinsic_T: [-0.011, -0.02329, 0.04412]\n')
        mapfile = root/'map.pcd'; mapfile.write_bytes(b'synthetic map identity only')
        (session/'run_metadata.json').write_text(json.dumps({'map_sha256':sha256(mapfile)}))
        writer=rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(session/'bag'),storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('',''))
        for topic,typ in [('/fast_lio/odom','nav_msgs/msg/Odometry'),
                          ('/fast_lio/local_cloud','sensor_msgs/msg/PointCloud2'),
                          ('/localization/status','kiss_matcher_localization/msg/LocalizationResult')]:
            writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type=typ,serialization_format='cdr'))
        rng=np.random.default_rng(19)
        local=rng.uniform([-2,-2,-1],[2,2,1],size=(1600,3)).astype('f4')
        for i in range(45):
            t=10**12+i*10**8; stamp=Time(sec=t//10**9,nanosec=t%10**9)
            o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base_link'
            o.pose.pose.orientation.w=1.;o.pose.pose.position.x=i*.04
            writer.write('/fast_lio/odom',serialize_message(o),t)
            cloud=build_cloud(stamp,'odom',local+np.array([i*.04,0,0]))
            writer.write('/fast_lio/local_cloud',serialize_message(cloud),t+1)
            if i==2:
                m=LocalizationResult();m.completed_stamp=stamp;m.local_map_stamp=stamp
                m.applied=applied;m.matched=True;m.overlap_percent=91.
                m.candidate_map_to_odom.header.frame_id='map';m.candidate_map_to_odom.child_frame_id='odom'
                m.candidate_map_to_odom.transform.rotation.w=1.
                m.candidate_map_to_odom.transform.translation.y=6.
                writer.write('/localization/status',serialize_message(m),t+2)
        del writer
        return argparse.Namespace(session=session,map=mapfile,region='space_2',device='synthetic',
            location_note='synthetic only',output=root/'library',start_offset=0.,end_offset=4.4,
            window_seconds=1.,radius=5.,blind=.5,voxel=.1,distance=.5,angle=20.,
            min_interval=2.,max_pose_age=10.,max_odom_gap=.15,max_cloud_gap=.3,min_points=1000)

    def test_export_uses_timestamp_pose_and_is_not_approved(self):
        with tempfile.TemporaryDirectory() as tmp:
            args=self.make_session(Path(tmp))
            with contextlib.redirect_stdout(io.StringIO()): out=build(args)
            info=json.loads((out/'manifest.json').read_text())
            self.assertEqual(info['state'],'exported_unreviewed')
            self.assertFalse(info['online_enabled']);self.assertFalse(info['descriptor_built'])
            rows=[json.loads(x) for x in (out/'keyframes.jsonl').read_text().splitlines()]
            self.assertGreaterEqual(len(rows),2)
            for row in rows:
                self.assertFalse(row['verified']);self.assertGreaterEqual(row['actual_span_seconds'],.8)
                self.assertAlmostEqual(row['map_T_base'][1][3],6.)
                expected=(row['stamp_ns']-10**12)/10**8*.04
                self.assertAlmostEqual(row['map_T_base'][0][3],expected)
                self.assertEqual(sha256(out/row['pcd']),row['pcd_sha256'])
                raw=(out/row['pcd']).read_bytes().split(b'DATA binary\n',1)[1]
                xyz=np.frombuffer(raw,dtype='<f4').reshape(-1,3)
                self.assertTrue(np.all(np.linalg.norm(xyz-[-.011,-.02329,.04412],axis=1)<=5.00001))
            self.assertIn('false,false',(out/'review.csv').read_text())

    def test_mismatched_map_rejected_before_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            args=self.make_session(Path(tmp));args.map.write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'SHA mismatch'): build(args)
            self.assertFalse(args.output.exists())

    def test_matched_without_applied_is_not_reference_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            args=self.make_session(Path(tmp),applied=False)
            with self.assertRaisesRegex(ValueError,'actually applied'): build(args)
            self.assertFalse(args.output.exists())

    def test_interpolation_does_not_extrapolate_or_bridge_gap(self):
        a=np.eye(4);b=np.eye(4);b[0,3]=2.
        self.assertIsNone(interpolate([0,100],[a,b],-1,200))
        self.assertIsNone(interpolate([0,100],[a,b],50,50))
        self.assertAlmostEqual(interpolate([0,100],[a,b],50,200)[0,3],1.)
        np.testing.assert_allclose(transformed(np.array([[1,2,3]]),b),[[3,2,3]])


if __name__=='__main__': unittest.main()
