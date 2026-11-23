"""VGICP-only continuous node regression. Explicit isolated ROS domain required."""
from pathlib import Path
import os,sys,time,tempfile,subprocess,signal,csv,json
import numpy as np,yaml,rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import PointCloud2
from kiss_matcher_localization.msg import LocalizationResult,LocalizationInput
from rclpy.qos import QoSProfile,DurabilityPolicy
assert os.environ.get('ROS_DOMAIN_ID') not in (None,'','0')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from local_map_window import build_cloud
rng=np.random.default_rng(12);xyz=rng.uniform([-4,-4,-2],[4,4,3],(7000,3)).astype('f4');shift=np.array([1.2,-2.3,.2],dtype='f4')
rclpy.init();node=Node('vgicp_only_regression');pub=node.create_publisher(PointCloud2,'/test/local',10);br=TransformBroadcaster(node);received=[];inputs=[]
node.create_subscription(LocalizationResult,'/localization/status',lambda x:received.append(x),QoSProfile(depth=20,durability=DurabilityPolicy.TRANSIENT_LOCAL));node.create_subscription(LocalizationInput,'/localization/input_snapshot',inputs.append,10)
with tempfile.TemporaryDirectory(prefix='vgicp_only_') as temp:
 t=Path(temp);mapfile=t/'map.pcd';data=np.zeros(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),('region_id','<i4')]);data['x'],data['y'],data['z']=xyz.T
 mapfile.write_bytes((f'VERSION 0.7\nFIELDS x y z intensity region_id\nSIZE 4 4 4 4 4\nTYPE F F F F I\nCOUNT 1 1 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nPOINTS {len(xyz)}\nDATA binary\n').encode()+data.tobytes())
 profile=t/'seeds.yaml';profile.write_text(yaml.safe_dump(dict(schema_version=1,map_frame='map',base_frame='base_link',regions=[dict(region_id=0,seeds=[dict(id='known',position=shift.tolist(),orientation_xyzw=[0,0,0,1])])])));(t/'labels.csv').write_text('region_id,region_name\n0,test_region\n')
 cmd=['/home/pc/kiss_localization_ws1.1/install/kiss_matcher_localization/lib/kiss_matcher_localization/navigation_localizer','--ros-args','-r','__ns:=/localization']
 params=dict(map_pcd=str(mapfile),region_labels_csv=str(t/'labels.csv'),initial_region='"0"',local_map_topic='/test/local',region_multistart_enabled='true',region_seed_file=str(profile),use_prior_tracking='true',publish_attempt_inputs='true',min_overlap_percent='60.0',metrics_csv=str(t/'metrics.csv'),periodic_interval_sec='1.0',retry_interval_sec='1.0',verbose='false')
 for k,v in params.items():cmd+=['-p',f'{k}:={v}']
 log=(t/'node.log').open('w');p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  deadline=time.monotonic()+35;last=0
  while len(received)<6 and time.monotonic()<deadline:
   assert p.poll() is None
   if time.monotonic()-last>.08:
    last=time.monotonic();n=node.get_clock().now().nanoseconds;stamp=node.get_clock().now().to_msg()
    if len(received)>=5:stamp.sec+=20
    tf=TransformStamped();tf.header.frame_id='odom';tf.header.stamp=stamp;tf.child_frame_id='base_link';tf.transform.rotation.w=1.;br.sendTransform(tf)
    pts=xyz+100 if len(received)==3 else xyz-shift
    pub.publish(build_cloud(stamp,'odom',pts))
   rclpy.spin_once(node,timeout_sec=.02)
  print('RESULTS',[(x.matched,x.applied,x.reason) for x in received])
  assert len(received)==6,[(x.matched,x.applied,x.reason) for x in received]
  assert received[0].matched and not received[0].applied
  assert received[1].applied and received[2].applied
  assert received[3].reason=='prior_registration_rejected' and not received[3].applied
  assert received[4].applied and received[5].reason=='local_map_stale_or_future_dated'
  with (t/'metrics.csv').open() as f:rows=list(csv.DictReader(f))
  assert all(x['coarse_ran']=='0' for x in rows)
  assert not any('kiss' in x['mode'].lower() for x in rows)
  gaps=[(int(y['completed_ns'])-int(x['completed_ns']))/1e9-float(y['total_ms'])/1000 for x,y in zip(rows,rows[1:])];assert all(g>=.98 for g in gaps),gaps
  assert len(inputs)==5 and all(x.stage in ['global','prior'] for x in inputs)
  print('PASS VGICP initialization/two windows, prior, rejected prior without inline recovery, next-input prior success, stale input, exact snapshots and completion cooldown',gaps)
 finally:
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=10)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  log.close()
  print((t/'node.log').read_text()[-1500:])
node.destroy_node();rclpy.shutdown()
