"""Isolated node test using two exact recorded window shapes and interpolated odom.
Re-stamps messages for offline execution; not original-time bag playback or field truth.
"""
import csv,json,os,signal,sqlite3,subprocess,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from kiss_matcher_localization.msg import LocalizationInput
from kiss_matcher_localization.srv import Relocalize
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from local_map_window import parse_cloud,build_cloud
root=Path('/home/pc/人形');project=root/'导航工程/mid360_lidar';session=Path(os.environ.get('MID360_TEST_SESSION',str(project/'bags/experiment_20261102_162807_733328')))
selected_attempts=[int(v) for v in os.environ.get('MID360_TEST_ATTEMPTS','2,5').split(',')]
assert len(selected_attempts)==2 and selected_attempts[0]!=selected_attempts[1]
selected_stamps=[int(v) for v in os.environ.get('MID360_TEST_WINDOW_STAMPS','').split(',') if v]
if selected_stamps: assert len(selected_stamps)==2 and len(set(selected_stamps))==2
expire_during=len(sys.argv)>2 and sys.argv[2]=='--expire-during-registration'
reset_during=len(sys.argv)>2 and sys.argv[2]=='--reset-during-registration'
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False);windows={}
for db in sorted((session/'bag').glob('*.db3')):
 with sqlite3.connect(db.as_uri()+'?mode=ro',uri=True) as c:
  tid=c.execute('select id from topics where name=?',('/fast_lio/local_map' if selected_stamps else '/localization/input_snapshot',)).fetchone()
  if tid:
   for data, in c.execute('select data from messages where topic_id=? order by timestamp',tid):
    if selected_stamps:
     m=deserialize_message(data,PointCloud2);n=m.header.stamp.sec*10**9+m.header.stamp.nanosec
     if n in selected_stamps:windows[selected_attempts[selected_stamps.index(n)]]=m
    else:
     m=deserialize_message(data,LocalizationInput)
     if m.attempt in selected_attempts:windows[int(m.attempt)]=m.cloud
assert set(windows)==set(selected_attempts),windows.keys()
rows=list(csv.DictReader((session/'odom_path.csv').open()));times=np.array([int(r['stamp_ns']) for r in rows],dtype=np.int64)
poses=np.array([[float(r[k]) for k in ['x','y','z','qx','qy','qz','qw']] for r in rows])
def odom(n):
 i=np.searchsorted(times,n)
 if i<len(times) and times[i]==n:
  a=poses[i];return a[:3],a[3:]
 assert 0<i<len(times) and times[i]-times[i-1]<300000000
 frac=float(n-times[i-1])/float(times[i]-times[i-1]);p=poses[i-1,:3]*(1-frac)+poses[i,:3]*frac
 q=Slerp([0,1],Rotation.from_quat(poses[i-1:i+1,3:]))([frac]).as_quat()[0];return p,q
rclpy.init();node=Node('recorded_region_initializer_test');pub=node.create_publisher(PointCloud2,'/test/recorded_window',10);br=TransformBroadcaster(node);client=node.create_client(Relocalize,'/localization/relocalize')
binary='/home/pc/kiss_localization_ws1.1/install/kiss_matcher_localization/lib/kiss_matcher_localization/navigation_localizer';log=(out/'node.log').open('w')
params=dict(map_pcd=str(root/'导航工程/kiss_localization_ws1.1/src/localization/maps/3_part_complete_candidate.pcd'),region_labels_csv=str(root/'导航工程/kiss_localization_ws1.1/src/localization/maps/region_labels.csv'),initial_region='"1"',local_map_topic='/test/recorded_window',region_multistart_enabled='true',region_seed_file=str(project/'config/region_initialization_building5.yaml'),retry_interval_sec='9999.0',periodic_interval_sec='0.0',max_local_map_age_sec='60.0',min_inliers='7',min_overlap_percent='60.0',use_prior_tracking='true',vgicp_threads='2',metrics_csv=str(out/'matching_metrics.csv'),verbose='false')
# Optional overrides for map-migration regression; defaults retain the historical fixture.
for key, env in [('map_pcd','MID360_TEST_MAP_PCD'),('region_labels_csv','MID360_TEST_LABELS'),
                 ('region_seed_file','MID360_TEST_SEEDS'),('initial_region','MID360_TEST_REGION')]:
 if env in os.environ:params[key]=os.environ[env]
if expire_during:params['max_local_map_age_sec']='0.9'
if 'MID360_TEST_MAX_MAP_AGE_SEC' in os.environ:params['max_local_map_age_sec']=os.environ['MID360_TEST_MAX_MAP_AGE_SEC']
if 'MID360_TEST_VGICP_THREADS' in os.environ:params['vgicp_threads']=os.environ['MID360_TEST_VGICP_THREADS']
cmd=[binary,'--ros-args','-r','__ns:=/localization']
for k,v in params.items():cmd+=['-p',f'{k}:={v}']
proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);cloud=None;last=0;manifest=[]
def spin(sec=.1):
 global last
 end=time.monotonic()+sec
 while time.monotonic()<end:
  if proc.poll() is not None:raise RuntimeError('node exited; '+str(out/'node.log'))
  if cloud is not None and time.monotonic()-last>.15:pub.publish(cloud);last=time.monotonic()
  rclpy.spin_once(node,timeout_sec=.01)
def wait(cond,seconds=120):
 end=time.monotonic()+seconds
 while not cond():
  if time.monotonic()>end:raise TimeoutError('see '+str(out/'node.log'))
  spin()
def publish(a):
 global cloud
 old=windows[a];n=old.header.stamp.sec*10**9+old.header.stamp.nanosec;p,q=odom(n);new=node.get_clock().now().nanoseconds+(700000000 if expire_during else -100000000)
 for dt in [-50000000,0,50000000]:
  t=TransformStamped();t.header.frame_id='odom';t.child_frame_id='base_link';ns=new+dt;t.header.stamp.sec=ns//10**9;t.header.stamp.nanosec=ns%10**9
  t.transform.translation.x=float(p[0]);t.transform.translation.y=float(p[1]);t.transform.translation.z=float(p[2]);t.transform.rotation.x=float(q[0]);t.transform.rotation.y=float(q[1]);t.transform.rotation.z=float(q[2]);t.transform.rotation.w=float(q[3]);br.sendTransform(t)
 xyz,intensity=parse_cloud(old);stamp=node.get_clock().now().to_msg();stamp.sec=new//10**9;stamp.nanosec=new%10**9;cloud=build_cloud(stamp,'odom',xyz,intensity)
 manifest.append(dict(original_attempt=a,original_cloud_ns=n,replay_cloud_ns=new,points=len(xyz),odom_position=p.tolist(),odom_quaternion_xyzw=q.tolist(),source_topic='/fast_lio/local_map' if selected_stamps else '/localization/input_snapshot'));spin(.5)
def metrics():
 f=out/'matching_metrics.csv';return list(csv.DictReader(f.open())) if f.exists() else []
def call():
 req=Relocalize.Request();req.apply_if_safe=True;f=client.call_async(req)
 if reset_during:
  wait(lambda:'LOC_BEGIN attempt=2' in (out/'node.log').read_text())
  cloud.header.stamp.sec-=1
  spin(.4)
 wait(f.done);r=f.result().result;print(r.reason,r.matched,r.applied,r.overlap_percent,flush=True);return r
try:
 wait(client.service_is_ready);spin(1);publish(selected_attempts[0]);wait(lambda:len(metrics())>=1)
 first=metrics()[0];assert first['applied']=='0',first
 if expire_during:
  assert first['reason']=='initialization_input_expired_during_registration',first
  (out/'result.json').write_text(json.dumps(dict(passed=True,scope='synthetic time boundary using recorded geometry',first=first),indent=2))
  print('PASS: input expires during registration and is not applied',flush=True)
  sys.exit(0)
 spin(5.2);publish(selected_attempts[1]);result=call()
 data=dict(scope='fixed recorded geometry, not field success rate or ground truth',source_session=str(session),selected_attempts=selected_attempts,first=first,second=metrics()[-1],applied_second=bool(result.applied),windows=manifest)
 (out/'result.json').write_text(json.dumps(data,ensure_ascii=False,indent=2))
 if reset_during:
  assert not result.applied and result.reason=='initialization_epoch_changed',data
  print('PASS: in-flight input reset blocks automatic application',flush=True)
 else:
  assert result.applied,data
  print('PASS: recorded calibration + later-window confirmation through production node',flush=True)
finally:
 (out/'input_manifest.json').write_text(json.dumps(manifest,indent=2))
 if proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(15)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 log.close();node.destroy_node();rclpy.shutdown()
