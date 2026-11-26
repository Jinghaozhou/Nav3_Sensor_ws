import os,sys,json,time,subprocess,signal
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped,Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation
from kiss_matcher_localization.msg import LocalizationResult
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from scan_context_descriptor import load_xyz
from local_map_window import build_cloud
from build_keyframe_library import transformed
assert os.environ.get('ROS_DOMAIN_ID')=='179'
OUT=Path(sys.argv[1]);OUT.mkdir(parents=True,exist_ok=False)
source=Path(os.environ.get('NAV3_TEST_FIXTURE_ROOT',str(ROOT.parents[1])))/'联调记录/20261111_space2关键帧扩充';q=json.loads((source/'queries.json').read_text())[0]
ob=np.array(q['odom_T_base']);points=load_xyz(source/q['pcd'])
if os.environ.get('NAV3_TEST_QUERY_NPZ'):
 z=np.load(os.environ['NAV3_TEST_QUERY_NPZ']);ob=np.eye(4);points=z['points'];q={'up_base':z['up_base'].tolist()}
rclpy.init();node=rclpy.create_node('race_real_geometry_test');tf=TransformBroadcaster(node)
pubs={t:node.create_publisher(PointCloud2,t,10) for t in ['/fast_lio/local_cloud','/fast_lio/local_map','/fast_lio/local_map_global']}
odom=node.create_publisher(Odometry,'/fast_lio/odom',10);gravity=node.create_publisher(Vector3Stamped,'/fast_lio/gravity',10)
permissions=[];node.create_subscription(Bool,"/localization/navigation_allowed",lambda m:permissions.append(m.data),10)
results=[];node.create_subscription(LocalizationResult,'/localization/status',results.append,10)
log=(OUT/'launch.log').open('w');p=subprocess.Popen(['ros2','launch',str(ROOT/'launch/race_localization.launch.py'),'output:='+str(OUT),'rviz:=false','record_inputs:=false','initial_region:='+os.environ.get('NAV3_TEST_REGION','space_2')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
last=0;success=False;stopped_bbs=None;stage='initial';before_recovery=0;tf_gids=set();peak_rss_kib=0
from tf2_msgs.msg import TFMessage
def tf_seen(m):
 if any(t.header.frame_id=='map' and t.child_frame_id=='odom' for t in m.transforms):tf_gids.add('map_to_odom_seen')
node.create_subscription(TFMessage,'/tf',tf_seen,10)

def rss_tree():
 records=[]
 for entry in Path('/proc').iterdir():
  if not entry.name.isdigit():continue
  try:
   text=(entry/'status').read_text();fields={x.split(':',1)[0]:x.split(':',1)[1].strip() for x in text.splitlines() if ':' in x}
   records.append((int(entry.name),int(fields['PPid']),int(fields.get('VmRSS','0 kB').split()[0])))
  except (OSError,KeyError,ValueError):pass
 owned={p.pid}
 for _ in range(5):owned.update(pid for pid,parent,rss in records if parent in owned)
 return sum(rss for pid,parent,rss in records if pid in owned)

try:
 end=time.monotonic()+100
 while time.monotonic()<end:
  if p.poll() is not None:raise RuntimeError('launch exited '+str(p.returncode))
  if '--virtual-first' in sys.argv and stopped_bbs is None:
   import re
   text=(OUT/'launch.log').read_text()
   match=re.search(r'\[python3-2\]: process started with pid \[(\d+)\]',text)
   if match:stopped_bbs=int(match.group(1));os.kill(stopped_bbs,signal.SIGSTOP)
  if time.monotonic()-last>.1:
   last=time.monotonic();stamp=node.get_clock().now().to_msg();rotation=Rotation.from_matrix(ob[:3,:3]).as_quat();t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='odom';t.child_frame_id='base_link'
   t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,ob[:3,3]);t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,rotation);tf.sendTransform(t)
   m=Odometry();m.header=t.header;m.child_frame_id='base_link';m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,ob[:3,3]);m.pose.pose.orientation=t.transform.rotation;odom.publish(m)
   g=Vector3Stamped();g.header=t.header;v=-ob[:3,:3]@np.array(q['up_base'])*9.81;g.vector.x,g.vector.y,g.vector.z=map(float,v);gravity.publish(g)
   pc=build_cloud(stamp,'odom',transformed(points,ob))
   for name,pub in pubs.items():
    if stage=='inject_failure' and name=='/fast_lio/local_map':pub.publish(build_cloud(stamp,'odom',transformed(points,ob)+100.))
    else:pub.publish(pc)
   peak_rss_kib=max(peak_rss_kib,rss_tree())
  rclpy.spin_once(node,timeout_sec=.015)
  if stage=='initial' and sum(m.applied for m in results)>=3:
   stage='inject_failure';before_recovery=sum(m.applied for m in results)
  elif stage=='inject_failure' and results and results[-1].reason=='prior_registration_rejected':stage='recover'
  elif stage=='recover' and sum(m.applied for m in results)>=before_recovery+2:success=True;break
 assert success,[(m.reason,m.overlap_percent) for m in results]
 assert any(m.reason=='applied' for m in results)
 assert len(tf_gids)==1,tf_gids
 publishers=[x for x in node.get_publishers_info_by_topic('/tf') if x.node_namespace.startswith('/localization')]
 assert len(publishers)==1 and publishers[0].node_namespace=='/localization',[(x.node_name,x.node_namespace) for x in publishers]
 permission_publishers=node.get_publishers_info_by_topic('/localization/navigation_allowed')
 assert len(permission_publishers)==1 and permission_publishers[0].node_namespace=='/localization',[(x.node_name,x.node_namespace) for x in permission_publishers]
 events=[json.loads(x) for x in (OUT/'race_events.jsonl').read_text().splitlines()]
 winners=[x['winner'] for x in events if x['event']=='race_closed']
 assert len(winners)>=2,events
 if '--virtual-first' in sys.argv:assert winners[0]=='virtual',winners
 assert True in permissions and False in permissions,permissions
 stale_deadline=time.monotonic()+3.5
 while time.monotonic()<stale_deadline:rclpy.spin_once(node,timeout_sec=.05)
 assert permissions and permissions[-1] is False,permissions
 print('PASS permission initially denied, granted after application, revoked on stale input',flush=True)
 print('PASS',[(m.reason,m.overlap_percent) for m in results[-6:]],flush=True)
finally:
 if stopped_bbs is not None:os.kill(stopped_bbs,signal.SIGCONT)
 p.send_signal(signal.SIGINT)
 try:p.wait(timeout=40)
 except subprocess.TimeoutExpired:raise RuntimeError('owned launch did not exit')
 log.close();(OUT/'results.json').write_text(json.dumps(dict(success=success,peak_child_rss_mib=peak_rss_kib/1024,map_odom_publisher_gids=list(tf_gids),results=[dict(matched=m.matched,applied=m.applied,reason=m.reason,overlap=m.overlap_percent,registration_ms=m.registration_ms) for m in results]),indent=2));node.destroy_node();rclpy.shutdown()
