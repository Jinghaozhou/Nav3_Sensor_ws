"""Isolated synthetic integration of region initialization; no field-accuracy claim.
Run with sourced ROS/KISS and an unused ROS_DOMAIN_ID, output-dir argument.
"""
import csv,json,os,signal,subprocess,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster
from kiss_matcher_localization.msg import LocalizationResult
from kiss_matcher_localization.srv import Relocalize,AcceptCandidate
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from local_map_window import build_cloud
missing_tf=len(sys.argv)>2 and sys.argv[2]=='--missing-tf'
reference_bounds=len(sys.argv)>2 and sys.argv[2]=='--reference-bounds'
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
rng=np.random.default_rng(923);parts=[]
for i in range(12):
 center=rng.uniform([-6,-5,-1],[6,5,3]); size=rng.uniform(.4,1.5,3)
 for axis in range(3):
  for sign in (-1,1):
   v=rng.uniform(-1,1,(110,3))*size;v[:,axis]=sign*size[axis];parts.append(v+center)
xyz=np.vstack(parts).astype(np.float32)
def pose(rpy,translation):
 t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',rpy,degrees=True).as_matrix();t[:3,3]=translation;return t
H=pose([10,-5,35],[1.2,-2.3,.2]);O=pose([15,8,-20],[4,-1,.8]);seed=H@O
source=((xyz-H[:3,3])@H[:3,:3]).astype(np.float32)
a=np.column_stack([xyz,np.ones(len(xyz))]).astype('<f4');mapfile=out/'map.pcd'
mapfile.write_bytes((f'VERSION 0.7\nFIELDS x y z region_id\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(a)}\nHEIGHT 1\nPOINTS {len(a)}\nDATA binary\n').encode()+a.tobytes())
labels=out/'labels.csv';labels.write_text('region_id,region_name\n1,entry\n')
def item(name,t):return dict(id=name,position=t[:3,3].tolist(),orientation_xyzw=Rotation.from_matrix(t[:3,:3]).as_quat().tolist())
profile=out/'seeds.yaml';duplicate=seed.copy();duplicate[0,3]+=.08
wrong=seed.copy();wrong[2,3]+=5
profile.write_text(yaml.safe_dump(dict(schema_version=1,map_frame='map',base_frame='base_link',regions=[dict(region_id=1,position_min=(seed[:3,3]-[2,2,2]).tolist(),position_max=(seed[:3,3]+[2,2,2]).tolist(),seeds=[item('correct',seed),item('same_basin',duplicate),item('wrong_floor',wrong)])])))
if reference_bounds:
 data=yaml.safe_load(profile.read_text());data['regions'][0]['position_bounds_mode']='initial_reference'
 profile.write_text(yaml.safe_dump(data))
rclpy.init();node=Node('region_multistart_test');pub=node.create_publisher(PointCloud2,'/test/window',10);br=TransformBroadcaster(node)
client=node.create_client(Relocalize,'/localization/relocalize');accept=node.create_client(AcceptCandidate,'/localization/accept_candidate')
binary='/home/pc/kiss_localization_ws1.1/install/kiss_matcher_localization/lib/kiss_matcher_localization/navigation_localizer'
log=(out/'node.log').open('w');cmd=[binary,'--ros-args','-r','__ns:=/localization']
params=dict(map_pcd=str(mapfile),region_labels_csv=str(labels),initial_region='"1"',local_map_topic='/test/window',region_multistart_enabled='true',region_seed_file=str(profile),retry_interval_sec='9999.0',periodic_interval_sec='0.0',max_local_map_age_sec='60.0',min_inliers='7',min_overlap_percent='60.0',use_prior_tracking='true',metrics_csv=str(out/'matching_metrics.csv'),verbose='false')
for k,v in params.items():cmd+=['-p',f'{k}:={v}']
proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);cloud=None;last=0

def spin(seconds=.05):
 global last
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  if proc.poll() is not None:raise RuntimeError('localizer exited; see '+str(out/'node.log'))
  if cloud is not None and time.monotonic()-last>.1:pub.publish(cloud);last=time.monotonic()
  rclpy.spin_once(node,timeout_sec=.01)
def wait(cond,seconds=40):
 end=time.monotonic()+seconds
 while not cond():
  if time.monotonic()>end:raise TimeoutError('test condition timeout; '+str(out/'node.log'))
  spin()
def publish_window(od):
 global cloud
 ns=node.get_clock().now().nanoseconds-(2000000000 if missing_tf else 100000000)
 for dt in [-50000000,0,50000000]:
  tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='base_link';n=ns+dt+(2000000000 if missing_tf else 0);tf.header.stamp.sec=n//10**9;tf.header.stamp.nanosec=n%10**9
  q=Rotation.from_matrix(od[:3,:3]).as_quat();tf.transform.translation.x=float(od[0,3]);tf.transform.translation.y=float(od[1,3]);tf.transform.translation.z=float(od[2,3]);tf.transform.rotation.x=float(q[0]);tf.transform.rotation.y=float(q[1]);tf.transform.rotation.z=float(q[2]);tf.transform.rotation.w=float(q[3]);br.sendTransform(tf)
 stamp=node.get_clock().now().to_msg();stamp.sec=ns//10**9;stamp.nanosec=ns%10**9
 cloud=build_cloud(stamp,'odom',source,np.ones(len(source),np.float32));spin(.3)
def call():
 req=Relocalize.Request();req.apply_if_safe=True;future=client.call_async(req);wait(future.done);r=future.result().result
 print(r.reason,r.matched,r.applied,r.overlap_percent,flush=True);return r
def rows():
 p=out/'matching_metrics.csv';return list(csv.DictReader(p.open())) if p.exists() else []
try:
 wait(client.service_is_ready);spin(1)
 publish_window(O);wait(lambda:len(rows())>=1)
 if missing_tf:
  r=call();assert not r.applied and r.reason=='initialization_tf_unavailable',r
  assert not any(v['applied']=='1' for v in rows())
  (out/'result.json').write_text(json.dumps(dict(passed=True,scope='synthetic only',checks=['latest TF cannot substitute missing snapshot TF']),indent=2))
  print('PASS: missing exact-time TF rejected',flush=True)
  sys.exit(0)
 # Same snapshot cannot count twice, and waiting candidates have no accept ID.
 r=call();assert not r.applied and 'confirmation' in r.reason,r
 assert not r.candidate_id
 assert len(rows())>=2 and not any(v['applied']=='1' for v in rows())
 spin(1.2)
 moved=pose([19,9,-15],[4.3,-.8,.9])
 if reference_bounds:
  moved=O.copy();moved[:3,3]+=H[:3,:3].T@np.array([0.,0.,4.5])
  assert (H@moved)[2,3]>seed[2,3]+2.,'test must leave the old current-position box'
 publish_window(moved)
 r=call();assert r.applied,r
 elapsed=(int(rows()[-1]['cloud_ns'])-int(rows()[0]['cloud_ns']))/1e9
 assert 1.0 <= elapsed < 5.0, elapsed
 print('PASS: confirmation with window stamp gap below old 5s:',elapsed,flush=True)
 tf=r.candidate_map_to_odom.transform;q=tf.rotation
 got=np.eye(4);got[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();got[:3,3]=[tf.translation.x,tf.translation.y,tf.translation.z]
 assert np.linalg.norm(got[:3,3]-H[:3,3])<.08,(got,H)
 assert Rotation.from_matrix(got[:3,:3]@H[:3,:3].T).magnitude()<np.deg2rad(2)
 publish_window(moved);r=call();assert r.applied,r
 assert rows()[-1]['mode']=='prior_vgicp',rows()[-1]
 candidates=list(csv.DictReader((out/'region_candidates.csv').open()))
 assert any(v['seed_id']=='wrong_floor' and v['eligible']=='0' for v in candidates),candidates
 assert any(v['seed_id']=='same_basin' and v['eligible']=='1' for v in candidates),candidates
 (out/'result.json').write_text(json.dumps(dict(passed=True,scope='synthetic integration only',checks=['nonzero roll/pitch and shifted odom','duplicate seeds same cluster','wrong floor seed not accepted','duplicate timestamp not confirmation','moving second window propagation','manual candidate unavailable before confirmation','prior tracking after hint consumed']),indent=2))
 print('PASS: multistart initialization integration',flush=True)
 if reference_bounds: print('PASS: upstairs current pose outside entry box confirms and continues prior tracking',flush=True)
finally:
 if proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(10)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 log.close();node.destroy_node();rclpy.shutdown()
