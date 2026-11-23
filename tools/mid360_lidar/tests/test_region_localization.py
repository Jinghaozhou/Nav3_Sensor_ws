"""Isolated synthetic test; never evidence of Building-5 field accuracy.
Source KISS install; set an unused ROS_DOMAIN_ID before running.
"""
import csv,json,os,signal,subprocess,sys,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from kiss_matcher_localization.msg import LocalizationResult
from kiss_matcher_localization.srv import SetRegion
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from local_map_window import build_cloud
no_fallback=len(sys.argv)>2 and sys.argv[2]=='--no-fallback'
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
rng=np.random.default_rng(12);parts=[]
for i in range(14):
 center=rng.uniform([-12,-7,-2],[12,7,4]);size=rng.uniform(.4,2.5,3)
 for axis in range(3):
  for sign in (-1,1):
   points=rng.uniform(-1,1,(130,3))*size;points[:,axis]=sign*size[axis];parts.append(points+center)
xyz=np.vstack(parts).astype(np.float32)
other=(rng.normal(size=(4000,3))*[3,5,2]+[70,20,0]).astype(np.float32)
arr=np.column_stack([np.vstack([xyz,other]),np.r_[np.ones(len(xyz)),np.zeros(len(other))]]).astype('<f4')
mapfile=out/'synthetic_regions.pcd'
mapfile.write_bytes((f'VERSION 0.7\nFIELDS x y z region_id\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(arr)}\nHEIGHT 1\nPOINTS {len(arr)}\nDATA binary\n').encode()+arr.tobytes())
labels=out/'region_labels.csv';labels.write_text('region_id,region_name\n0,其他区域\n1,5号楼入口楼梯\n')
rclpy.init();node=Node('test_region_localization');pub=node.create_publisher(PointCloud2,'/fast_lio/local_map',10)
longpub=node.create_publisher(PointCloud2,'/fast_lio/local_map_global',10);results=[]
node.create_subscription(LocalizationResult,'/localization/status',results.append,QoSProfile(depth=20,durability=DurabilityPolicy.TRANSIENT_LOCAL))
client=node.create_client(SetRegion,'/localization/set_region')
binary='/home/pc/kiss_localization_ws1.1/install/kiss_matcher_localization/lib/kiss_matcher_localization/navigation_localizer'
log=(out/'node.log').open('w')
cmd=[binary,'--ros-args','-r','__ns:=/localization','-p',f'map_pcd:={mapfile}','-p',f'region_labels_csv:={labels}','-p','initial_region:=我现在在五号楼入口楼梯','-p','local_map_topic:=/fast_lio/local_map','-p','global_local_map_topic:=/fast_lio/local_map_global','-p','use_prior_tracking:=true','-p','retry_interval_sec:=1.0','-p','periodic_interval_sec:=1.0','-p','min_inliers:=20','-p','min_overlap_percent:=70.0','-p',f'metrics_csv:={out}/matching_metrics.csv']
cmd += ['-p','region_full_map_fallback:='+('false' if no_fallback else 'true')]
process=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
last=0
br=TransformBroadcaster(node)

def tick():
 global last
 if process.poll() is not None:raise RuntimeError('node exited: '+str(process.returncode))
 if time.monotonic()-last>.1:
  last=time.monotonic();tf=TransformStamped();tf.header.stamp=node.get_clock().now().to_msg();tf.header.frame_id='odom';tf.child_frame_id='base_link';tf.transform.rotation.w=1.;br.sendTransform(tf)
  msg=build_cloud(node.get_clock().now().to_msg(),'odom',xyz-np.array([1.2,-2.3,.2],np.float32),np.ones(len(xyz),np.float32));pub.publish(msg);longpub.publish(msg)
 rclpy.spin_once(node,timeout_sec=.01)
def wait(condition,seconds=75):
 end=time.monotonic()+seconds
 while not condition():
  if time.monotonic()>end:raise TimeoutError('condition timeout')
  tick()
def call(text):
 wait(lambda:client.service_is_ready(),10)
 request=SetRegion.Request();request.region=text;f=client.call_async(request);wait(f.done,60);return f.result()
try:
 wait(lambda:sum(m.applied for m in results)>=2)
 rows=list(csv.DictReader((out/'matching_metrics.csv').open()))
 assert rows[0]['mode']=='region_kiss' and rows[0]['applied']=='1',rows
 assert int(rows[0]['region_target_points'])<int(rows[0]['global_points'])
 assert any(r['mode']=='prior_vgicp' for r in rows)
 assert not call('999').success
 assert not call('我要去5号楼入口楼梯').success
 assert call('全图').region_id==-1
 assert call('0').success
 if no_fallback:
  wait(lambda:sum(r['mode']=='region_kiss' and r['region_id']=='0' and r['applied']=='0' for r in csv.DictReader((out/'matching_metrics.csv').open()))>=2)
  assert call('全图').success
  wait(lambda:any(r['mode']=='global_kiss' and r['applied']=='1' for r in csv.DictReader((out/'matching_metrics.csv').open())))
 else:
  wait(lambda:'region_fallback_global' in (out/'node.log').read_text())
  wait(lambda:any(r['mode']=='region_fallback_global' and r['applied']=='1' for r in csv.DictReader((out/'matching_metrics.csv').open())))
 assert call('我现在在5号楼入口楼梯').region_id==1
 print('fallback_enabled=',not no_fallback)
 print('PASS: semantic/numeric ID, invalid destination/ID rejection, region init, consumed hint -> prior; '+('failed hint retained across attempts until explicit clear' if no_fallback else 'wrong region -> full fallback'))
 (out/'result.json').write_text(json.dumps({'passed':True,'scope':'synthetic only','attempts_observed':len(results)},indent=2))
finally:
 if process.poll() is None:
  os.killpg(process.pid,signal.SIGINT)
  try:process.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
 log.close();node.destroy_node();rclpy.shutdown()
