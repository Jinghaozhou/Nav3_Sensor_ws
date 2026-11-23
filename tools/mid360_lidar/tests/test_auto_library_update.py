"""Finalized synthetic bag through real admission/descriptor/merge transaction.
Uses production map geometry; explicitly NOT an independent Go2 accuracy test.
"""
import json,sys,tempfile,time
from pathlib import Path
from unittest.mock import patch
import numpy as np
import rosbag2_py
from rclpy.serialization import serialize_message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from kiss_matcher_localization.msg import LocalizationResult
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import auto_keyframe_update as auto
from keyframe_database import KeyframeDatabase,default_library
from build_keyframe_library import sha256,transformed
from scan_context_descriptor import load_xyz
from local_map_window import build_cloud
from scipy.spatial.transform import Rotation
MAP=ROOT.parent/'kiss_localization_ws1.1/src/localization/maps/building_split_space.pcd'
BASE=default_library();database=KeyframeDatabase(BASE,MAP)
cache=ROOT/'reference_maps/race_map_cache'/(database.map_sha+'_voxel_0.1')
region=load_xyz(cache/'region_2.pcd');row=json.loads((BASE/'keyframes.jsonl').read_text().splitlines()[0]);mb=np.array(row['map_T_base']);ob=np.eye(4)
points=region[(np.linalg.norm(region-mb[:3,3],axis=1)<4.8)&(np.linalg.norm(region-mb[:3,3],axis=1)>.6)]
assert len(points)>1000
base_points=transformed(points,np.linalg.inv(mb));up=mb[:3,:3].T@np.array([0.,0.,1.]);g=-up*9.81
before=sha256(BASE/'keyframes.jsonl')
with tempfile.TemporaryDirectory(prefix='auto_library_contract_') as temp:
 root=Path(temp);(root/'config').mkdir();(root/'reference_maps/keyframe_libraries/space_2').mkdir(parents=True)
 selection=dict(schema_version=1,region='space_2',library=str(BASE));auto.save(root/'config/keyframe_library.json',selection)
 session=root/'experiment_test';(session/'source_configs').mkdir(parents=True)
 import shutil;shutil.copy2(ROOT/'mid360_robot.yaml',session/'source_configs/mid360_robot.yaml')
 auto.save(session/'run_metadata.json',dict(map_sha256=database.map_sha));auto.save(session/'acquisition_profile.json',dict(platform='isolated_synthetic_fixture_not_field',calibrated=False))
 writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(session/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
 topics={'/fast_lio/local_cloud':'sensor_msgs/msg/PointCloud2','/fast_lio/odom':'nav_msgs/msg/Odometry','/fast_lio/gravity':'geometry_msgs/msg/Vector3Stamped','/localization/status':'kiss_matcher_localization/msg/LocalizationResult'}
 for name,typ in topics.items():writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=typ,serialization_format='cdr'))
 start=int(time.time())*10**9
 for i in range(61):
  stamp_ns=start+i*100000000
  from builtin_interfaces.msg import Time
  stamp=Time(sec=stamp_ns//10**9,nanosec=stamp_ns%10**9)
  od=Odometry();od.header.stamp=stamp;od.header.frame_id='odom';od.child_frame_id='base_link';od.pose.pose.orientation.w=1.
  grav=Vector3Stamped();grav.header=od.header;grav.vector.x,grav.vector.y,grav.vector.z=map(float,g)
  for topic,msg in [('/fast_lio/odom',od),('/fast_lio/gravity',grav),('/fast_lio/local_cloud',build_cloud(stamp,'odom',base_points))]:writer.write(topic,serialize_message(msg),stamp_ns)
  if i%10==0:
   m=LocalizationResult();m.completed_stamp=stamp;m.local_map_stamp=stamp;m.applied=True;m.matched=True;m.reason='applied';m.overlap_percent=97.;m.candidate_map_to_odom.header.frame_id='map';m.candidate_map_to_odom.child_frame_id='odom';m.candidate_map_to_odom.transform.translation.x,m.candidate_map_to_odom.transform.translation.y,m.candidate_map_to_odom.transform.translation.z=map(float,mb[:3,3]);q=Rotation.from_matrix(mb[:3,:3]).as_quat();m.candidate_map_to_odom.transform.rotation.x,m.candidate_map_to_odom.transform.rotation.y,m.candidate_map_to_odom.transform.rotation.z,m.candidate_map_to_odom.transform.rotation.w=map(float,q)
   writer.write('/localization/status',serialize_message(m),stamp_ns)
 del writer
 # A tiny exact geometry subset suffices for isolated policy/transaction checks.
 with patch.object(auto,'ROOT',root),patch.object(auto,'default_library',lambda:BASE),patch.object(auto,'map_cache',return_value=(region,np.full(len(region),2))):
  result=auto.update(session,MAP);assert result['status']=='updated',result
  new=KeyframeDatabase(Path(result['library']),MAP);assert len(new.index)>40
  manifest=new.manifest;assert not manifest['operator_session_pose_floor_confirmed'];assert manifest['automatic_admission_policy']=='space2_auto_v1'
  assert auto.update(session,MAP)==result,'Idempotent rerun must not append again'
  assert sha256(BASE/'keyframes.jsonl')==before,'Frozen baseline changed'
  # Contents are validated again by the next launch, including automatic evidence.
  rows=[json.loads(x) for x in (new.path/'keyframes.jsonl').read_text().splitlines()];rows[-1]['auto_admission']['region_vote']=.1
  (new.path/'keyframes.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in rows));m=manifest.copy();m['index_sha256']=sha256(new.path/'keyframes.jsonl');auto.save(new.path/'manifest.json',m)
  try:KeyframeDatabase(new.path,MAP)
  except ValueError as e:assert 'admission evidence failed' in str(e),e
  else:raise AssertionError('Bad region evidence accepted')
  print('PASS finalized bag -> strict admission -> immutable version -> next-run selection; baseline unchanged; rerun idempotent; corrupt evidence refused',flush=True)
