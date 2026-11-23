"""Isolated production node + live retriever on re-stamped recorded processed clouds.
No true-domain publication, no hardware, no surveyed-accuracy claim.
"""
from pathlib import Path
import os,sys,time,json,csv,subprocess,signal,copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Transform,TransformStamped,Vector3Stamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation
from kiss_matcher_localization.msg import KeyframeCandidates
from kiss_matcher_localization.srv import Relocalize
R=Path('/home/pc/人形');P=R/'导航工程/mid360_lidar';sys.path.insert(0,str(P/'scripts'))
from keyframe_database import KeyframeDatabase
from build_keyframe_library import reader,ns,interpolate,message_pose,transformed
from local_map_window import build_cloud
from scan_context_descriptor import load_xyz
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
assert os.environ.get('ROS_DOMAIN_ID') not in [None,'','0'],'Use explicitly isolated domain'
lib=P/'reference_maps/keyframe_libraries/space_2/experiment_20261109_170630_971075';mp=R/'导航工程/kiss_localization_ws1.1/src/localization/maps/building_split_space.pcd';db=KeyframeDatabase(lib,mp)
src=R/'联调记录/20261109_201727_独立查询验证';qs={x['id']:x for x in map(json.loads,(src/'query_index.jsonl').read_text().splitlines())};selected=['q_011','q_012','q_013'];stamps={n for k in selected for n in qs[k]['source_message_stamps']};clouds={};br=reader(P/'bags/experiment_20261109_201727_141866/bag',['/fast_lio/local_cloud'])
while br.has_next():
 _,data,_=br.read_next();m=deserialize_message(data,PointCloud2);n=ns(m.header.stamp)
 if n in stamps:clouds[n]=m
 if n>max(stamps):break
assert stamps<=set(clouds)
with (P/'bags/experiment_20261109_201727_141866/odom_path.csv').open() as f:od=list(csv.DictReader(f))
ts=[int(x['stamp_ns']) for x in od];poses=[]
for x in od:
 T=np.eye(4);T[:3,:3]=Rotation.from_quat([float(x[k]) for k in ['qx','qy','qz','qw']]).as_matrix();T[:3,3]=[float(x[k]) for k in ['x','y','z']];poses.append(T)
g=np.load(src/'gravity_track.npz',allow_pickle=False);gts=g['stamp_ns'];grav=g['gravity_odom']
rclpy.init();node=Node('recorded_keyframe_test');local=node.create_publisher(PointCloud2,'/test/keyframe_local_cloud',10);cp=node.create_publisher(KeyframeCandidates,'/localization/keyframe_candidates',10);raw=node.create_publisher(PointCloud2,'/fast_lio/local_cloud',10);op=node.create_publisher(Odometry,'/fast_lio/odom',10);gp=node.create_publisher(Vector3Stamped,'/fast_lio/gravity',10);tf=TransformBroadcaster(node);selected_clouds=[];selected_sub=node.create_subscription(PointCloud2,"/localization/selected_cloud",selected_clouds.append,10);received={};sub=node.create_subscription(KeyframeCandidates,'/localization/keyframe_candidates',lambda m:received.update({ns(m.cloud.header.stamp):m}) if m.ready else None,10);client=node.create_client(Relocalize,'/localization/relocalize');children=[];files=[];results=[]
def spin(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
def wait(test,timeout=50):
 end=time.monotonic()+timeout
 while not test():
  assert all(p.poll() is None for p in children),'child exited'
  if time.monotonic()>end:raise TimeoutError('isolation wait')
  spin(.05)
def stamp(n):
 from builtin_interfaces.msg import Time
 return Time(sec=n//10**9,nanosec=n%10**9)
def sendtf(n,T):
 m=TransformStamped();m.header.frame_id='odom';m.child_frame_id='base_link';m.header.stamp=stamp(n);q=Rotation.from_matrix(T[:3,:3]).as_quat();m.transform.translation.x=float(T[0,3]);m.transform.translation.y=float(T[1,3]);m.transform.translation.z=float(T[2,3]);m.transform.rotation.x=float(q[0]);m.transform.rotation.y=float(q[1]);m.transform.rotation.z=float(q[2]);m.transform.rotation.w=float(q[3]);tf.sendTransform(m);return m.transform

def invoke(label):
 req=Relocalize.Request();req.apply_if_safe=True;f=client.call_async(req);wait(f.done,70);r=f.result().result;v=dict(label=label,matched=r.matched,applied=r.applied,reason=r.reason,overlap=r.overlap_percent,registration_ms=r.registration_ms);results.append(v);print(json.dumps(v),flush=True);return r

def start(cmd,name):
 f=(out/name).open('w');files.append(f);p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
params=dict(map_pcd=str(mp),region_labels_csv=str(R/'导航工程/kiss_localization_ws1.1/src/localization/maps/region_labels.csv'),initial_region='"space_2"',local_map_topic='/test/keyframe_local_cloud',region_multistart_enabled='true',region_seed_file=str(P/'config/region_initialization_space.yaml'),keyframe_initialization_enabled='true',keyframe_library_sha256=db.library_sha,keyframe_profile_sha256=db.profile_sha,retry_interval_sec='9999.0',periodic_interval_sec='0.0',max_local_map_age_sec='10.0',min_inliers='7',min_overlap_percent='60.0',use_prior_tracking='true',vgicp_threads='4',metrics_csv=str(out/'matching_metrics.csv'),verbose='false')
if '--hierarchical' in sys.argv[2:]:params['hierarchical_initialization_enabled']='true'
cmd=[str(R/'导航工程/kiss_localization_ws1.1/install/kiss_matcher_localization/lib/kiss_matcher_localization/navigation_localizer'),'--ros-args','-r','__ns:=/localization']
for k,v in params.items():cmd+=['-p',f'{k}:={v}']
try:
 start(cmd,'localizer.log');wait(client.service_is_ready);spin(.5)
 q=qs['q_011'];points=load_xyz(src/q['pcd']);ob=np.array(q['odom_T_base']);h,_,_=db.retrieve(points,q['up_base'],ob)
 def packet():
  n=node.get_clock().now().nanoseconds;m=KeyframeCandidates();m.cloud=build_cloud(stamp(n),'odom',transformed(points,ob));m.window_start_ns=n-900000000;m.region_id=2;m.map_sha256=db.map_sha;m.library_sha256=db.library_sha;m.profile_sha256=db.profile_sha;m.ready=True
  for x in h:
   T=x['map_T_odom'];v=Transform();rot=Rotation.from_matrix(T[:3,:3]).as_quat();v.translation.x=float(T[0,3]);v.translation.y=float(T[1,3]);v.translation.z=float(T[2,3]);v.rotation.x=float(rot[0]);v.rotation.y=float(rot[1]);v.rotation.z=float(rot[2]);v.rotation.w=float(rot[3]);m.map_to_odom.append(v);m.ids.append(x['id']);m.scores.append(x['distance'])
  return m
 for kind in ['wrong_map','wrong_profile','truncated','duplicate_ids','stale']:
  m=packet()
  if kind=='wrong_map':m.map_sha256='0'*64
  if kind=='wrong_profile':m.profile_sha256='0'*64
  if kind=='truncated':m.map_to_odom=m.map_to_odom[:-1];m.ids=m.ids[:-1];m.scores=m.scores[:-1]
  if kind=='duplicate_ids':m.ids=[m.ids[0]]*28
  if kind=='stale':m.cloud.header.stamp=stamp(ns(m.cloud.header.stamp)-20000000000);m.window_start_ns-=20000000000
  local.publish(m.cloud);cp.publish(m);spin(.3);r=invoke(kind);assert not r.applied and not r.matched
 # Synthetic unmatchable query: must end without any coarse registration.
 m=packet();n=ns(m.cloud.header.stamp);sendtf(n,ob);rng=np.random.default_rng(42);z=rng.normal(size=(1200,3));z/=np.linalg.norm(z,axis=1)[:,None];z*=rng.uniform(.6,4.7,len(z))[:,None];m.cloud=build_cloud(stamp(n),'odom',transformed(z,ob));local.publish(m.cloud);cp.publish(m);spin(.3);fallback=invoke('synthetic_no_match_no_coarse');assert not fallback.applied
 assert 'keyframe_kiss_fallback' not in (out/'matching_metrics.csv').read_text()
 start(['/usr/bin/python3',str(P/'scripts/keyframe_retriever.py'),'--library',str(lib),'--map',str(mp),'--output',str(out)],'retriever.log');wait(lambda:raw.get_subscription_count()>0);spin(.5)
 # Warm a valid short shape without estimated gravity: must not invent vertical.
 base=node.get_clock().now().nanoseconds
 for i in range(10):
  n=base+i*100000000;tr=sendtf(n,ob);om=Odometry();om.header.frame_id='odom';om.child_frame_id='base_link';om.header.stamp=stamp(n);om.pose.pose.position.x=tr.translation.x;om.pose.pose.position.y=tr.translation.y;om.pose.pose.position.z=tr.translation.z;om.pose.pose.orientation=tr.rotation;op.publish(om);raw.publish(build_cloud(stamp(n),'odom',transformed(points/10.,ob)));spin(.1)
 spin(1.1);missing=invoke('missing_estimated_gravity');assert not missing.applied and missing.reason=='keyframe_query_unavailable'
 for key in selected:
  q=qs[key];old=q['source_message_stamps'];base=node.get_clock().now().nanoseconds;new_end=base+(old[-1]-old[0]);start_wall=time.monotonic()
  for oldn in old:
   n=base+oldn-old[0];T=interpolate(ts,poses,oldn,150000000);assert T is not None;tr=sendtf(n,T);om=Odometry();om.header.frame_id='odom';om.child_frame_id='base_link';om.header.stamp=stamp(n);om.pose.pose.position.x=tr.translation.x;om.pose.pose.position.y=tr.translation.y;om.pose.pose.position.z=tr.translation.z;om.pose.pose.orientation=tr.rotation;op.publish(om)
   i=int(np.searchsorted(gts,oldn));f=float(oldn-gts[i-1])/float(gts[i]-gts[i-1]);v=(1-f)*grav[i-1]+f*grav[i];gm=Vector3Stamped();gm.header.frame_id='odom';gm.header.stamp=stamp(n);gm.vector.x=float(v[0]);gm.vector.y=float(v[1]);gm.vector.z=float(v[2]);gp.publish(gm)
   m=copy.deepcopy(clouds[oldn]);m.header.stamp=stamp(n);raw.publish(m);spin(.1)
  wait(lambda:new_end in received,5);km=received[new_end];assert len(km.ids)==28 and km.map_sha256==db.map_sha
  local.publish(km.cloud);spin(.2);r=invoke(key)
  if key=='q_011':assert r.matched and not r.applied and r.reason=='initialization_wait_confirmation'
  elif key=='q_012':assert r.matched and r.applied
  else:assert r.matched and r.applied
 bad=packet();bad.ready=False;bad.cloud.header.stamp=stamp(0);bad.map_sha256='0'*64;cp.publish(bad);spin(.15)
 stable=invoke('invalid_candidate_packet_does_not_reset_prior');assert stable.matched and stable.applied
 # Recovery state test takes exclusive ownership of candidate publication.
 # Stop only this test's retriever: its stale warnings must not race replayed packets.
 retriever=children.pop();os.killpg(retriever.pid,signal.SIGINT);retriever.wait(timeout=10)
 # Reject a corrupted prior input; next attempts must use database candidates,
 # even though the initial region hint has already been consumed.
 n=node.get_clock().now().nanoseconds;local.publish(build_cloud(stamp(n),'odom',z+100));spin(.2)
 lost=invoke('prior_rejected_no_coarse');assert not lost.applied and lost.reason=='prior_registration_rejected'
 saved=[received[k] for k in sorted(received)][-2:]
 for index,old_packet in enumerate(saved):
  m=copy.deepcopy(old_packet);span=ns(m.cloud.header.stamp)-m.window_start_ns;n=node.get_clock().now().nanoseconds
  m.cloud.header.stamp=stamp(n);m.window_start_ns=n-span
  # Two distinct recorded inputs, fresh synthetic timestamps; no field claim.
  q=qs['q_012' if index==0 else 'q_013'];sendtf(n,np.array(q['odom_T_base']))
  local.publish(m.cloud);cp.publish(m);spin(.25);r=invoke('database_recovery_'+str(index+1))
  assert r.matched and r.applied==(index==1)
  spin(1.05)
 with (out/'matching_metrics.csv').open() as f:rows=list(csv.DictReader(f))
 assert all(x['coarse_ran']=='0' for x in rows),'Continuous runtime must never run coarse registration'
 assert not any('kiss' in x['mode'].lower() for x in rows)
 assert len(selected_clouds)>=4 and any(ns(x.header.stamp)==new_end for x in selected_clouds)
 (out/'result.json').write_text(json.dumps(dict(passed=True,tests=results,domain=os.environ['ROS_DOMAIN_ID'],recorded_source='experiment_20261109_201727_141866',restamped=True,live_retriever_used=True,original_map_age_limit_seconds=10,not_field_validation=True,hierarchical="--hierarchical" in sys.argv[2:]),indent=2))
finally:
 for p in reversed(children):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=12)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 for f in files:f.close()
 node.destroy_node();rclpy.shutdown()
