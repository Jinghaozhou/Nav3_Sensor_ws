#!/usr/bin/python3
"""发布当前区域、RViz文字与跨区事件；不修改定位TF或初始化提示。"""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from kiss_matcher_localization.msg import LocalizationResult
from region_tracking import RegionClassifier,RegionState
from region_profiles import ROOT
from build_keyframe_library import sha256

def stamp(t):return t.sec+t.nanosec/1e9
class RegionTracker(Node):
 def __init__(self,args):
  super().__init__('region_tracker');m=json.loads((args.index/'manifest.json').read_text())
  if sha256(args.map)!=m['map_sha256'] or sha256(args.index/'regions.npz')!=m['index_sha256']:raise ValueError('区域索引与地图校验不一致')
  data=np.load(args.index/'regions.npz',allow_pickle=False);self.classifier=RegionClassifier(data['xyz'],data['labels']);self.state=RegionState();self.pose=None;self.last_applied=None;self.latest_applied=False;self.last_clock=None;self.was_confirmed=False
  self.file=None
  if args.output:
   args.output.mkdir(parents=True,exist_ok=True);self.file=(args.output/'region_transitions.jsonl').open('x')
  qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.publisher=self.create_publisher(String,'/localization/current_region',qos);self.events=self.create_publisher(String,'/localization/region_transition',10);self.marker=self.create_publisher(Marker,'/localization/region_marker',qos)
  self.create_subscription(PoseStamped,'/localization/map_pose',self.on_pose,10);self.create_subscription(LocalizationResult,'/localization/status',self.on_status,qos);self.create_timer(.5,self.tick)
  self.get_logger().info('区域维护已启动：13区三维标签投票；边界2秒/3帧确认；定位失效时报告不确定，不更改定位器。')
 def on_pose(self,msg):
  if msg.header.frame_id=='map':self.pose=msg
 def on_status(self,msg):
  self.latest_applied=bool(msg.applied)
  if msg.applied:self.last_applied=stamp(msg.completed_stamp)
 def tick(self):
  now=self.get_clock().now().nanoseconds/1e9
  if self.last_clock is not None and now<self.last_clock:
   self.pose=None;self.last_applied=None;self.latest_applied=False;self.state=RegionState()
  self.last_clock=now;reason='not_initialized';confidence=0.;region=0;pose_stamp=None;position=None
  if self.pose is not None:
   pose_stamp=stamp(self.pose.header.stamp);p=self.pose.pose.position;position=[p.x,p.y,p.z]
   pose_age=now-pose_stamp;success_age=None if self.last_applied is None else now-self.last_applied
   if success_age is None:reason='not_initialized'
   elif pose_age<-.2 or success_age<-.2:reason='future_timestamp'
   elif pose_age>2.:reason='pose_stale'
   elif success_age>10.:reason='localization_stale'
   elif not self.latest_applied:reason='tracking_unconfirmed'
   else:region,confidence,reason=self.classifier.classify(position)
  event=self.state.update(region,pose_stamp if pose_stamp is not None else now,region>0)
  if event:
   event.update(source='accepted_localization_pose_estimate',observed_ns=int(now*1e9),position_map=position,confidence=confidence)
   if event['kind']=='transition':text=f"跨区提醒：space_{event['from_region']} → space_{event['to_region']}"
   elif event['kind']=='initial':text=f"当前区域已确认：space_{event['to_region']}"
   else:text=f"区域识别已恢复：space_{event['to_region']}"
   self.get_logger().info(text);self.events.publish(String(data=json.dumps(event,ensure_ascii=False)))
   if self.file:self.file.write(json.dumps(event,ensure_ascii=False)+'\n');self.file.flush()
  confirmed=self.state.current if region>0 else 0
  if self.was_confirmed and not confirmed:self.get_logger().warn('当前区域不确定：'+reason+'；保留上次区域供参考，不发送跨区结论。')
  self.was_confirmed=bool(confirmed)
  status=dict(stamp_ns=int(now*1e9),region_id=confirmed,region_name=f'space_{confirmed}' if confirmed else 'unknown',last_confirmed_region=self.state.last_known,candidate_region=region,pending_region=self.state.pending,confidence=confidence,reason=('boundary_confirmation' if region>0 and self.state.pending else reason),position_map=position,pose_stamp_ns=int(pose_stamp*1e9) if pose_stamp is not None else None,position_reference='Fast-LIO IMU body; not calibrated Go2 chassis')
  self.publisher.publish(String(data=json.dumps(status,ensure_ascii=False)))
  marker=Marker();marker.header.frame_id='map';marker.header.stamp=self.get_clock().now().to_msg();marker.ns='current_region';marker.id=0;marker.type=Marker.TEXT_VIEW_FACING;marker.action=Marker.ADD;marker.pose.orientation.w=1.;marker.scale.z=.35;marker.color.a=1.
  if position is not None:marker.pose.position.x=position[0];marker.pose.position.y=position[1];marker.pose.position.z=position[2]+.8
  # RViz默认字体不保证中文字形；界面用统一英文space名，终端和日志保留中文。
  marker.text=f'Current: space_{confirmed}' if confirmed else 'Region: uncertain'
  if self.state.pending:marker.text+=f'\nConfirming: space_{self.state.pending}'
  marker.color.g=1. if confirmed else .6;marker.color.r=0. if confirmed else 1.;self.marker.publish(marker)
 def destroy_node(self):
  if self.file:self.file.close()
  super().destroy_node()
def main():
 p=argparse.ArgumentParser();p.add_argument('--map',type=Path,required=True);p.add_argument('--index',type=Path,default=ROOT/'reference_maps/region_tracking');p.add_argument('--output',type=Path);a,ros=p.parse_known_args();rclpy.init(args=ros);node=None
 try:node=RegionTracker(a);rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:
  if node:node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
