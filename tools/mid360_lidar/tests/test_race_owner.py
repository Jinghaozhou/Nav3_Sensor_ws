"""Isolated service-contract test, not field accuracy or proof of branch quality."""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster
from kiss_matcher_localization.srv import RaceInitialization,Relocalize
from kiss_matcher_localization.msg import LocalizationResult
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from build_keyframe_library import sha256
from local_map_window import build_cloud
import numpy as np
assert os.environ.get('ROS_DOMAIN_ID') not in ('0',None,'')
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
# Generate a small local fixture; do not depend on archived experiments or old overlays.
fixture=a.output/'fixture';fixture.mkdir()
import yaml
xyz=np.random.default_rng(42).uniform(-3,3,(500,3)).astype('<f4')
data=np.zeros(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),('region_id','<i4')])
data['x'],data['y'],data['z']=xyz.T
(fixture/'map.pcd').write_bytes((f'VERSION 0.7\nFIELDS x y z intensity region_id\nSIZE 4 4 4 4 4\nTYPE F F F F I\nCOUNT 1 1 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nPOINTS {len(xyz)}\nDATA binary\n').encode()+data.tobytes())
(fixture/'labels.csv').write_text('region_id,region_name\n0,test_region\n')
(fixture/'seeds.yaml').write_text(yaml.safe_dump(dict(schema_version=1,map_frame='map',base_frame='base_link',regions=[dict(region_id=0,dynamic_candidates_only=True,seeds=[])])))
rclpy.init();node=rclpy.create_node('race_contract_test');tf=TransformBroadcaster(node)
local=node.create_publisher(PointCloud2,'/fast_lio/local_map',10)
params=dict(map_pcd=str(fixture/'map.pcd'),region_labels_csv=str(fixture/'labels.csv'),region_seed_file=str(fixture/'seeds.yaml'),
    region_multistart_enabled=True,external_initialization_enabled=True,race_bbs_sha256='b'*64,race_virtual_sha256='c'*64,
    race_bbs_profile_sha256='d'*64,race_virtual_profile_sha256='e'*64,retry_interval_sec=9999.,periodic_interval_sec=9999.,use_prior_tracking=True,
    local_map_topic='/fast_lio/local_map',local_map_reliability='reliable',min_overlap_percent=60.,verbose=False)
cmd=[str(Path(__import__('ament_index_python.packages',fromlist=['get_package_prefix']).get_package_prefix('kiss_matcher_localization'))/'lib/kiss_matcher_localization/navigation_localizer'),'--ros-args','-r','__ns:=/localization']
for k,v in params.items():cmd+=['-p',k+':='+str(v).lower() if isinstance(v,bool) else k+':='+str(v)]
log=(a.output/'node.log').open('w');child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
client=node.create_client(RaceInitialization,'/localization/race_initialization');reloc=node.create_client(Relocalize,'/localization/relocalize')
results=[]
def spin(t):
 end=time.monotonic()+t
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
def call(req):
 f=client.call_async(req);end=time.monotonic()+10
 while not f.done():
  assert time.monotonic()<end;rclpy.spin_once(node,timeout_sec=.02)
 return f.result()
def send():
 t=TransformStamped();t.header.frame_id='odom';t.child_frame_id='base_link';t.header.stamp=node.get_clock().now().to_msg();t.transform.rotation.w=1.;tf.sendTransform(t)
 return t.header.stamp
try:
 assert client.wait_for_service(timeout_sec=25)
 r=call(RaceInitialization.Request());epoch=r.epoch;assert r.waiting
 for _ in range(5):stamp=send();spin(.1)
 def request(kind):
  stamp=send();spin(.05);p=LocalizationResult();p.local_map_stamp=stamp;p.completed_stamp=node.get_clock().now().to_msg();p.matched=True;p.applied=True;p.reason='applied';p.overlap_percent=90.
  p.candidate_map_to_odom.header.frame_id='map';p.candidate_map_to_odom.child_frame_id='odom';p.candidate_map_to_odom.transform.rotation.w=1.
  q=RaceInitialization.Request();q.submit=True;q.epoch=epoch;q.source='bbs';q.map_sha256=sha256(fixture/'map.pcd');q.library_sha256='b'*64;q.profile_sha256='d'*64;q.proposal=p
  if kind in ('virtual_winner','virtual_wrong_profile'):
   q.source='virtual';q.library_sha256='c'*64;q.profile_sha256='e'*64
  if kind=='virtual_wrong_profile':q.profile_sha256='d'*64
  if kind=='disabled_keyframes':q.source='keyframes';q.library_sha256='c'*64;q.profile_sha256='e'*64
  if kind=='first_window':q.proposal.applied=False
  if kind=='wrong_epoch':q.epoch+=1
  if kind=='wrong_version':q.library_sha256='f'*64
  if kind=='stale':q.proposal.local_map_stamp.sec-=20
  if kind=='nan':q.proposal.candidate_map_to_odom.transform.translation.x=float('nan')
  return q
 for kind in ('first_window','wrong_epoch','wrong_version','stale','nan','disabled_keyframes','virtual_wrong_profile','virtual_winner','late_loser'):
  r=call(request(kind));results.append(dict(case=kind,reason=r.result.reason,applied=r.result.applied,waiting=r.waiting))
  assert r.result.applied==(kind=='virtual_winner'),results
  if kind=='virtual_winner':assert not r.waiting
 print('PASS virtual source, disabled keyframes, version/profile/epoch/freshness/confirmation guards and single winner',flush=True)
finally:
 child.send_signal(signal.SIGINT);child.wait(timeout=15);log.close();node.destroy_node();rclpy.shutdown();(a.output/'results.json').write_text(json.dumps(results,indent=2))
