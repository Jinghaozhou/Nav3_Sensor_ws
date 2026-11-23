"""ROS message-layout, actual FIFO duration and trace semantic regression tests."""
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from kiss_matcher_localization.msg import LocalizationResult
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from local_map_window import LocalMapWindow, build_cloud, parse_cloud
from localization_trace import LocalizationTrace, matrix, quat, interpolate


def stamp(sec):
    from builtin_interfaces.msg import Time
    return Time(sec=int(sec),nanosec=int(round((sec-int(sec))*1e9)))


class Components(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init()
    @classmethod
    def tearDownClass(cls): rclpy.shutdown()

    def test_pcl_nonterminal_intensity_with_row_padding(self):
        m=PointCloud2(); m.width=2; m.height=2; m.point_step=32; m.row_step=72
        m.fields=[PointField(name=n,offset=o,datatype=7,count=1) for n,o in
                  [('x',0),('y',4),('z',8),('intensity',16),('normal_x',20)]]
        raw=bytearray(144)
        for r in range(2):
            for c in range(2):
                arr=np.ndarray(8,dtype='<f4',buffer=raw,offset=r*72+c*32)
                arr[:3]=[r,c,1]; arr[4]=40+r*2+c
        m.data=bytes(raw); xyz,it=parse_cloud(m)
        np.testing.assert_equal(xyz,[[0,0,1],[0,1,1],[1,0,1],[1,1,1]])
        np.testing.assert_equal(it,[40,41,42,43])

    def test_true_120_second_window_and_adaptation(self):
        args=SimpleNamespace(window_time=120.,tracking_window_time=30.,window_frames=1500,
            max_buffer_points=3000000,voxel=.15,rate=1.,in_topic='/test/cloud',
            out_topic='/test/window',global_topic='/test/global',adaptive=False,recovery_failures=3)
        node=LocalMapWindow(args)
        try:
            for i in range(1301):
                xyz=np.array([[i*.2,0,0],[i*.2,1,0]],np.float32)
                node.on_cloud(build_cloud(stamp(i/10),'odom',xyz,np.array([10,20],np.float32)))
            long,n,span=node.make_window(120)
            self.assertEqual(n,1201); self.assertAlmostEqual(span,120)
            short,n,span=node.make_window(30)
            self.assertEqual(n,301); self.assertAlmostEqual(span,30)
            _,it=parse_cloud(short); self.assertTrue(set(it).issubset({10,20}))
            result=LocalizationResult(); result.applied=True; node.on_status(result)
            self.assertTrue(node.tracking)
            stats=[]
            from rclpy.time import Time as RosTime
            from rclpy.clock import ClockType
            import json
            node.get_clock=lambda:SimpleNamespace(now=lambda:RosTime(seconds=130,clock_type=ClockType.ROS_TIME))
            node.stats_pub=SimpleNamespace(publish=lambda m:stats.append(json.loads(m.data)))
            node.publish_window()
            self.assertEqual(stats[-1]['cloud_stamp_ns'],130000000000)
            self.assertEqual(stats[-1]['window_start_ns'],100000000000)
            self.assertEqual(stats[-1]['long_window_start_ns'],10000000000)
            self.assertEqual(stats[-1]['long_requested_seconds'],120.)
            result.applied=False
            for _ in range(3): node.on_status(result)
            self.assertFalse(node.tracking)
        finally: node.destroy_node()

    def test_path_and_success_vs_candidate(self):
        with tempfile.TemporaryDirectory() as folder:
            node=LocalizationTrace(folder)
            try:
                for i in range(4):
                    m=Odometry(); m.header.frame_id='odom'; m.child_frame_id='base_link'
                    m.header.stamp=stamp(10+i*.1); m.pose.pose.position.x=float(i)
                    m.pose.pose.orientation.w=1.; node.on_odom(m)
                result=LocalizationResult(); result.local_map_stamp=stamp(10.2)
                result.completed_stamp=stamp(11); result.matched=True; result.applied=True
                result.reason='applied'; result.overlap_percent=80.
                result.candidate_map_to_odom.transform.translation.x=10.
                result.candidate_map_to_odom.transform.rotation.w=1.
                node.on_status(result)
                self.assertEqual(len(node.markers),2)
                self.assertAlmostEqual(node.markers[0].pose.position.x,12)
                result.applied=False; result.reason='jump_exceeds_limit'
                result.candidate_map_to_odom.transform.translation.x=100.
                node.on_status(result)
                self.assertEqual(node.T[0,3],10.)
                self.assertEqual(node.markers[-2].ns,'candidate')
                lines=(Path(folder)/'map_path.csv').read_text().splitlines()
                self.assertEqual(len(lines),5)
                self.assertEqual(float(lines[1].split(',')[1]),10.)
                self.assertEqual(float(lines[-1].split(',')[1]),13.)
            finally: node.attempt_file.close(); node.destroy_node()

    def test_quaternion_and_interpolation(self):
        T=matrix([1,2,3],[.2,-.3,.4,.8]); Q=matrix([1,2,3],quat(T[:3,:3]))
        np.testing.assert_allclose(T,Q,atol=1e-12)
        T0=matrix([0,0,0],[0,0,0,1]); T1=matrix([2,0,0],[0,0,0,1])
        self.assertAlmostEqual(interpolate([0,200000000],[T0,T1],100000000)[0,3],1)
        self.assertIsNone(interpolate([0,2000000000],[T0,T1],1000000000))

if __name__=='__main__': unittest.main()
