"""Slow search, bounded pending work, reset, and reception tests. Isolated ROS."""
from pathlib import Path
from types import SimpleNamespace
import os,sys,time,unittest
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

assert os.environ.get('ROS_DOMAIN_ID') not in (None,'','0')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from bbs_retriever import BbsRetriever
from local_map_window import build_cloud

class SlowDatabase:
    region_id=2;origin=np.zeros(3);map_sha='a'*64;library_sha='b'*64;profile_sha='c'*64
    def __init__(self,delay):self.delay=delay;self.count=0;self.last_metrics={};self.calls=[];self.busy=0;self.maximum_busy=0
    def retrieve(self,p,up,ob):
        assert not p.flags.writeable and not up.flags.writeable and not ob.flags.writeable
        n=self.query_cloud_ns;self.calls.append(n);self.busy+=1;self.maximum_busy=max(self.maximum_busy,self.busy)
        time.sleep(self.delay);self.count+=1;self.busy-=1
        self.last_metrics=dict(search_ms=self.delay*1000,total_ms=self.delay*1000,points=len(p),peaks=1,complete=True)
        return [dict(id='bbs_peak_00',distance=0.,map_T_odom=np.eye(4))],0.,self.delay*1000

class AsyncTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setup_node(self,delay,body_frame='base_link'):
        self.db=SlowDatabase(delay);self.node=BbsRetriever(SimpleNamespace(output=None,odom_body_frame=body_frame),self.db)
        self.node.timer.cancel();self.messages=[];self.node.pub=SimpleNamespace(publish=self.messages.append)
        self.points=np.random.default_rng(1).uniform([.6,-1,-1],[2,1,1],(1200,3)).astype('f4')
        now=self.node.get_clock().now().nanoseconds
        for i in range(10):self.feed(now-(9-i)*100000000)
    def feed(self,n):
        stamp=Time(sec=n//10**9,nanosec=n%10**9)
        m=Odometry();m.header.frame_id='odom';m.header.stamp=stamp;m.child_frame_id=self.node.a.odom_body_frame;m.pose.pose.orientation.w=1.
        self.node.odom_cb(m)
        g=Vector3Stamped();g.header=m.header;g.vector.z=-9.81;self.node.gravity_cb(g)
        self.node.cloud(build_cloud(stamp,'odom',self.points))
    def pump(self,seconds,feed=False):
        end=time.monotonic()+seconds;last=0
        while time.monotonic()<end:
            if feed and time.monotonic()-last>.1:
                self.feed(self.node.get_clock().now().nanoseconds);last=time.monotonic()
            rclpy.spin_once(self.node,timeout_sec=.005)
    def tearDown(self):
        self.node.close_search();self.node.destroy_node()
    def test_go2_imu_frame_keeps_query_ready(self):
        self.setup_node(.02,body_frame='imu_link');self.node.encode_query();self.pump(.15)
        self.assertTrue(self.messages[-1].ready)
        count=len(self.node.odom)
        wrong=Odometry();wrong.header.frame_id='odom';wrong.child_frame_id='base_link'
        wrong.header.stamp=self.node.get_clock().now().to_msg();wrong.pose.pose.orientation.w=1.
        self.node.odom_cb(wrong)
        self.assertEqual(len(self.node.odom),count)
    def test_slow_worker_does_not_block_reception(self):
        self.setup_node(1.2);self.node.encode_query();self.pump(1.4,feed=True)
        self.assertTrue(any(m.ready for m in self.messages))
        self.assertGreaterEqual((self.node.clouds[-1][0]-self.node.clouds[0][0])/1e9,.8)
        self.assertEqual(self.db.maximum_busy,1)
    def test_only_newest_pending_query_is_run(self):
        self.setup_node(.6);self.node.encode_query();newest=None
        for _ in range(3):
            self.pump(.1,feed=True);self.node.encode_query();newest=self.node.pending[1]['n']
        self.pump(1.1,feed=True)
        self.assertEqual(len(self.db.calls),2)
        self.assertEqual(self.db.calls[-1],newest)
        self.assertEqual(self.db.maximum_busy,1)
    def test_epoch_reset_discards_inflight_result(self):
        self.setup_node(.15);self.node.encode_query();self.node.reset();self.pump(.3)
        self.assertFalse(any(m.ready for m in self.messages))
    def test_hard_invalid_input_discards_inflight_result(self):
        self.setup_node(.15);self.node.encode_query();self.node.unavailable('short_cloud_stale_or_future');self.pump(.3)
        self.assertFalse(any(m.ready for m in self.messages))
    def test_tracking_application_discards_inflight_result(self):
        self.setup_node(.15);self.node.encode_query()
        self.node.tracking_status(SimpleNamespace(applied=True));self.pump(.3)
        self.assertFalse(any(m.ready for m in self.messages))
        self.assertFalse(self.node.searching_enabled())
    def test_soft_wait_does_not_overwrite_ready_packet(self):
        self.setup_node(.02);self.node.encode_query();self.pump(.15)
        self.assertTrue(self.messages[-1].ready);count=len(self.messages)
        self.node.unavailable('short_cloud_warmup')
        self.assertEqual(len(self.messages),count)

if __name__=='__main__':unittest.main()
