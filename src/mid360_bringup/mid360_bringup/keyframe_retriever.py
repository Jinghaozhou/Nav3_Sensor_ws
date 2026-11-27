#!/usr/bin/python3
"""Live short query encoder. Publishes hypotheses only; cannot apply localization."""
import argparse,json,time,csv
from pathlib import Path
from collections import deque
from bisect import bisect_left
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped,Transform
from std_msgs.msg import String
from scipy.spatial.transform import Rotation
from .keyframe_geometry import ns, message_pose, interpolate, transformed
from .local_map_window import parse_cloud, voxel_indices, build_cloud
from .keyframe_database import KeyframeDatabase, default_library, validate_sensor_config


class Retriever(Node):
    def __init__(self,a):
        super().__init__('keyframe_retriever');self.a=a;self.db=KeyframeDatabase(a.library,a.map)
        if a.sensor_config:
            validate_sensor_config(a.sensor_config, self.db)
        from kiss_matcher_localization.msg import KeyframeCandidates
        self.Msg=KeyframeCandidates;self.clouds=deque();self.odom={};self.gravity={};self.last_cloud=None;self.last_query=None;self.count=0;self.last_reason=None
        self.pub=self.create_publisher(KeyframeCandidates,a.candidates_topic,2)
        self.status=self.create_publisher(String,a.status_topic,5)
        self.create_subscription(PointCloud2,a.cloud_topic,self.cloud,qos_profile_sensor_data)
        self.create_subscription(Odometry,a.odom_topic,self.odom_cb,qos_profile_sensor_data)
        self.create_subscription(Vector3Stamped,a.gravity_topic,self.gravity_cb,qos_profile_sensor_data)
        self.timer=self.create_timer(1.,self.query)
        self.file=None
        if a.output:
            a.output.mkdir(parents=True,exist_ok=True);self.file=(a.output/'keyframe_queries.csv').open('x',newline='');self.writer=csv.writer(self.file);self.writer.writerow(['query','cloud_ns','start_ns','span_s','points','encoding_ms','retrieval_ms','ids','distances'])
        self.get_logger().warn('EXPERIMENTAL keyframes: provisional reference labels, all candidates, VGICP/two-window still required; no TF publication')
    def reset(self):self.clouds.clear();self.odom.clear();self.gravity.clear();self.last_query=None
    def odom_cb(self,m):
        if (m.header.frame_id,m.child_frame_id)!=('odom',self.a.odom_body_frame):return
        n=ns(m.header.stamp)
        try:self.odom[n]=message_pose(m.pose.pose)
        except ValueError:return
        for k in list(self.odom):
            if k<n-20_000_000_000:del self.odom[k]
    def gravity_cb(self,m):
        if m.header.frame_id!='odom':return
        v=np.array([m.vector.x,m.vector.y,m.vector.z]);n=ns(m.header.stamp)
        if not np.isfinite(v).all() or not 8<=np.linalg.norm(v)<=12:return
        self.gravity[n]=v
        for k in list(self.gravity):
            if k<n-20_000_000_000:del self.gravity[k]
    def cloud(self,m):
        if m.header.frame_id!='odom':return
        n=ns(m.header.stamp)
        if self.last_cloud is not None and n<=self.last_cloud:self.reset()
        elif self.last_cloud is not None and n-self.last_cloud>300_000_000:self.clouds.clear()
        self.last_cloud=n
        xyz,_=parse_cloud(m);self.clouds.append((n,m.header.stamp,xyz))
        while self.clouds and n-self.clouds[0][0]>1_000_000_000:self.clouds.popleft()
    def unavailable(self,why):
        if why!=self.last_reason:self.get_logger().warn("KEYFRAME_QUERY unavailable: "+why);self.last_reason=why
        m=self.Msg();m.region_id=self.db.region_id;m.reason=why
        if self.clouds:m.cloud.header.stamp=self.clouds[-1][1]
        m.cloud.header.frame_id='odom';self.pub.publish(m);self.status.publish(String(data=json.dumps({'ready':False,'reason':why})))
    def query(self):
        try:self.encode_query()
        except Exception as e:self.unavailable(type(e).__name__+': '+str(e));self.get_logger().error('KEYFRAME_QUERY rejected: '+str(e))
    def encode_query(self):
        if not self.clouds:return self.unavailable('no_short_cloud')
        n,stamp,_=self.clouds[-1]
        age=(self.get_clock().now().nanoseconds-n)/1e9
        if age< -1 or age>2:return self.unavailable('short_cloud_stale_or_future')
        if self.last_query==n:return
        start=self.clouds[0][0]
        if n-start<800_000_000:return self.unavailable('short_cloud_warmup')
        ts=sorted(self.odom);ob=interpolate(ts,[self.odom[k] for k in ts],n,150_000_000)
        if ob is None:return self.unavailable('query_odom_missing')
        gs=sorted(self.gravity);i=bisect_left(gs,n)
        if i<len(gs) and gs[i]==n:g=self.gravity[n]
        elif 0<i<len(gs) and gs[i]-gs[i-1]<=300_000_000:
            f=(n-gs[i-1])/(gs[i]-gs[i-1]);g=(1-f)*self.gravity[gs[i-1]]+f*self.gravity[gs[i]]
        else:return self.unavailable('estimated_gravity_missing_at_query_stamp')
        up=ob[:3,:3].T@(-g);up/=np.linalg.norm(up)
        p=transformed(np.concatenate([x[2] for x in self.clouds]),np.linalg.inv(ob));radius=np.linalg.norm(p-self.db.origin,axis=1);p=p[(radius>=.5)&(radius<=5)];p=p[voxel_indices(p,.1)]
        hypotheses,enc,ret=self.db.retrieve(p,up,ob)
        m=self.Msg();m.cloud=build_cloud(stamp,'odom',transformed(p,ob));m.window_start_ns=start;m.region_id=self.db.region_id;m.map_sha256=self.db.map_sha;m.library_sha256=self.db.library_sha;m.profile_sha256=self.db.profile_sha;m.encoding_ms=enc;m.retrieval_ms=ret;m.ready=True;m.reason='all_reference_candidates'
        for h in hypotheses:
            t=h['map_T_odom'];q=Rotation.from_matrix(t[:3,:3]).as_quat();v=Transform();v.translation.x=float(t[0,3]);v.translation.y=float(t[1,3]);v.translation.z=float(t[2,3]);v.rotation.x=float(q[0]);v.rotation.y=float(q[1]);v.rotation.z=float(q[2]);v.rotation.w=float(q[3]);m.map_to_odom.append(v);m.ids.append(h['id']);m.scores.append(h['distance'])
        if self.last_reason is not None:self.get_logger().info("KEYFRAME_QUERY ready: candidates="+str(len(hypotheses)));self.last_reason=None
        self.pub.publish(m);self.last_query=n;self.count+=1
        self.status.publish(String(data=json.dumps(dict(ready=True,cloud_ns=n,span_s=(n-start)/1e9,points=len(p),candidates=len(hypotheses),encoding_ms=enc,retrieval_ms=ret,top1=m.ids[0],top1_distance=m.scores[0]))))
        if self.file:self.writer.writerow([self.count,n,start,(n-start)/1e9,len(p),enc,ret,json.dumps(list(m.ids)),json.dumps(list(m.scores))]);self.file.flush()


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--library',type=Path,default=default_library())
    p.add_argument('--map',type=Path,required=True)
    p.add_argument('--sensor-config',type=Path)
    p.add_argument('--odom-body-frame',default='base_link',choices=['base_link','imu_link'])
    p.add_argument('--output',type=Path)
    p.add_argument('--cloud-topic',default='/fast_lio/local_cloud')
    p.add_argument('--odom-topic',default='/fast_lio/odom')
    p.add_argument('--gravity-topic',default='/fast_lio/gravity')
    p.add_argument('--candidates-topic',default='/localization/keyframe_candidates')
    p.add_argument('--status-topic',default='/localization/keyframe_status')
    a,ros=p.parse_known_args();rclpy.init(args=ros);node=Retriever(a)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node.file:node.file.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
