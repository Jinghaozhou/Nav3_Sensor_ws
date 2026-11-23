#!/usr/bin/env python3
"""Record every localization attempt and visualize a continuous map-frame trajectory.

Path uses latest accepted T_map_odom to reproject buffered odometry, including
poses before first success. This is not ground truth or pose-graph optimization.
Markers preserve each accepted candidate's pose at its input-cloud timestamp.
"""
import argparse
import signal
import bisect
import csv
from datetime import datetime
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from visualization_msgs.msg import Marker, MarkerArray
from kiss_matcher_localization.msg import LocalizationResult


def matrix(translation, rotation):
    x,y,z,w = rotation
    q = np.array([x,y,z,w], float)
    norm = np.linalg.norm(q)
    if norm < 1e-8: raise ValueError('invalid quaternion')
    x,y,z,w = q/norm
    R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=translation
    return T


def quat(R):
    # Stable for rotations near 180 degrees as well as near identity.
    vals,vecs=np.linalg.eigh(np.array([
        [R[0,0]-R[1,1]-R[2,2],R[1,0]+R[0,1],R[2,0]+R[0,2],R[2,1]-R[1,2]],
        [R[1,0]+R[0,1],R[1,1]-R[0,0]-R[2,2],R[2,1]+R[1,2],R[0,2]-R[2,0]],
        [R[2,0]+R[0,2],R[2,1]+R[1,2],R[2,2]-R[0,0]-R[1,1],R[1,0]-R[0,1]],
        [R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1],R.trace()]]))
    q=vecs[:,np.argmax(vals)]
    return q if q[3]>=0 else -q


def xyzq(obj):
    t=obj.position if hasattr(obj,'position') else obj.translation
    q=obj.orientation if hasattr(obj,'orientation') else obj.rotation
    return matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])


def stamp_ns(stamp): return stamp.sec*10**9+stamp.nanosec


def pose_message(ns,T):
    p=PoseStamped(); p.header.frame_id='map'
    p.header.stamp.sec=int(ns//10**9); p.header.stamp.nanosec=int(ns%10**9)
    p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,T[:3,3])
    q=quat(T[:3,:3])
    p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=map(float,q)
    return p


def interpolate(times,poses,ns,max_gap=0.3):
    if not times: return None
    i=bisect.bisect_left(times,ns)
    if i<len(times) and times[i]==ns: return poses[i]
    if i==0 or i==len(times):
        k=0 if i==0 else len(times)-1
        return poses[k] if abs(times[k]-ns)<=max_gap*1e9 else None
    if times[i]-times[i-1]>max_gap*1e9: return None
    a=(ns-times[i-1])/(times[i]-times[i-1])
    q0,q1=quat(poses[i-1][:3,:3]),quat(poses[i][:3,:3])
    if q0@q1<0: q1=-q1
    q=(1-a)*q0+a*q1; q/=np.linalg.norm(q)
    return matrix((1-a)*poses[i-1][:3,3]+a*poses[i][:3,3],q)


class LocalizationTrace(Node):
    def __init__(self,output):
        super().__init__('localization_trace')
        self.folder=Path(output); self.folder.mkdir(parents=True,exist_ok=True)
        if (self.folder/'attempts.csv').exists():
            raise FileExistsError(f'session already exists: {self.folder}; choose a new output directory')
        self.times,self.poses=[],[]
        self.T=None; self.markers=[]; self.events=0; self.pending=[]
        self.attempt_file=(self.folder/'attempts.csv').open('w',newline='')
        self.writer=csv.writer(self.attempt_file)
        self.writer.writerow(['attempt','completed_ns','cloud_ns','matched','applied','reason',
                              'coarse_inliers','overlap_percent','registration_ms',
                              'delta_m','delta_deg','pose_available']+[f'T{i}{j}' for i in range(4) for j in range(4)])
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub=self.create_publisher(PathMsg,'/localization/path',qos)
        self.marker_pub=self.create_publisher(MarkerArray,'/localization/match_markers',qos)
        self.create_subscription(LocalizationResult,'/localization/status',self.on_status,qos)
        self.create_subscription(Odometry,'/fast_lio/odom',self.on_odom,20)
        self.create_timer(1.0,self.publish_path)
        self.create_timer(5.0,self.save_path)
        self.get_logger().info(f'CSV output: {self.folder}; green=applied, yellow=matched but not applied')

    def on_odom(self,msg):
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':
            self.get_logger().error('odometry must be odom -> base_link'); return
        ns=stamp_ns(msg.header.stamp)
        if self.times and ns<=self.times[-1]: return
        # Hard limit ~2.8 hours at 10Hz; do not silently discard the start of a session.
        if len(self.times)>=100000:
            self.get_logger().error('trace reached 100000 poses; finish recording and start a new session'); return
        self.times.append(ns); self.poses.append(xyzq(msg.pose.pose))
        for result in list(self.pending):
            if self.add_marker(result): self.pending.remove(result)
            elif ns-stamp_ns(result.local_map_stamp)>10**9:
                self.get_logger().warn('matched result has no time-aligned odometry; CSV retained, marker omitted')
                self.pending.remove(result)

    def on_status(self,result):
        self.events+=1
        T=xyzq(result.candidate_map_to_odom.transform) if result.matched else None
        odom=interpolate(self.times,self.poses,stamp_ns(result.local_map_stamp))
        self.writer.writerow([self.events,stamp_ns(result.completed_stamp),stamp_ns(result.local_map_stamp),
            result.matched,result.applied,result.reason,result.inliers,result.overlap_percent,
            result.registration_ms,result.translation_delta_m,result.rotation_delta_deg,
            odom is not None]+(T.reshape(-1).tolist() if T is not None else ['']*16))
        self.attempt_file.flush()
        if result.applied: self.T=T
        if result.matched and not self.add_marker(result): self.pending.append(result)
        self.get_logger().info(f'#{self.events} matched={result.matched} applied={result.applied} '
            f'overlap={result.overlap_percent:.2f}% time={result.registration_ms:.0f}ms reason={result.reason}')
        self.publish_path(); self.save_path()

    def add_marker(self,result):
        ns=stamp_ns(result.local_map_stamp)
        odom=interpolate(self.times,self.poses,ns)
        if odom is None: return False
        T=xyzq(result.candidate_map_to_odom.transform)@odom
        sphere=Marker(); sphere.header.frame_id='map'; sphere.header.stamp=result.completed_stamp
        sphere.ns='applied' if result.applied else 'candidate'; sphere.id=len(self.markers)
        sphere.type=Marker.SPHERE; sphere.action=Marker.ADD
        sphere.pose=pose_message(ns,T).pose
        sphere.scale.x=sphere.scale.y=sphere.scale.z=0.3
        sphere.color.r=0.1 if result.applied else 1.0
        sphere.color.g=1.0; sphere.color.b=0.1; sphere.color.a=1.0
        text=Marker(); text.header=sphere.header; text.ns='labels'; text.id=sphere.id
        text.type=Marker.TEXT_VIEW_FACING; text.action=Marker.ADD
        text.pose=pose_message(ns,T).pose; text.pose.position.z+=0.5
        text.scale.z=0.3; text.color=sphere.color
        text.text=f'#{sphere.id//2+1} {"APPLIED" if result.applied else "CANDIDATE"} {result.overlap_percent:.1f}% {result.registration_ms/1000:.1f}s'
        self.markers.extend([sphere,text]); self.marker_pub.publish(MarkerArray(markers=self.markers))
        return True

    def publish_path(self):
        if self.T is None or not self.times: return
        path=PathMsg(); path.header.frame_id='map'; path.header.stamp=self.get_clock().now().to_msg()
        path.poses=[pose_message(ns,self.T@T) for ns,T in zip(self.times,self.poses)]
        self.path_pub.publish(path)

    def save_path(self):
        for name,transform in [('odom_path.csv',np.eye(4)),('map_path.csv',self.T)]:
            if transform is None: continue
            tmp=self.folder/(name+'.tmp')
            with tmp.open('w',newline='') as f:
                w=csv.writer(f); w.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw'])
                for ns,T in zip(self.times,self.poses):
                    pose=transform@T; w.writerow([ns]+pose[:3,3].tolist()+quat(pose[:3,:3]).tolist())
            tmp.replace(self.folder/name)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', default=str(Path(__file__).resolve().parents[1]/'slam_output'/'localization_sessions'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
    args,ros_args=ap.parse_known_args(); rclpy.init(args=ros_args)
    node=LocalizationTrace(args.output)
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.save_path(); node.attempt_file.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': main()
