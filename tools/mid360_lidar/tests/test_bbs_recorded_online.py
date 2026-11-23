"""Re-stamped real cloud geometry through LIVE BBS -> VGICP -> two windows.
Only an explicit isolated ROS_DOMAIN_ID is allowed. No radar/driver is started.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import rclpy
from geometry_msgs.msg import Transform, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation
from kiss_matcher_localization.msg import KeyframeCandidates
from kiss_matcher_localization.srv import Relocalize

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from bbs_database import BbsDatabase
from local_map_window import build_cloud
from build_keyframe_library import transformed
from scan_context_descriptor import load_xyz

def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'','0'), 'Use an isolated ROS domain'
    parser=argparse.ArgumentParser(); parser.add_argument('--output',type=Path,required=True); a=parser.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    source=ROOT.parents[1]/'联调记录/20261111_space2关键帧扩充'
    q=json.loads((source/'queries.json').read_text())[0]
    ob=np.array(q['odom_T_base']); points=load_xyz(source/q['pcd'])
    map_path=ROOT.parent/'kiss_localization_ws1.1/src/localization/maps/building_split_space.pcd'
    db=BbsDatabase(map_path); db.close()
    rclpy.init(); node=rclpy.create_node('bbs_recorded_live_test')
    cloud=node.create_publisher(PointCloud2,'/fast_lio/local_cloud',10)
    local=node.create_publisher(PointCloud2,'/fast_lio/local_map',10)
    long=node.create_publisher(PointCloud2,'/fast_lio/local_map_global',10)
    odom=node.create_publisher(Odometry,'/fast_lio/odom',10)
    gravity=node.create_publisher(Vector3Stamped,'/fast_lio/gravity',10)
    candidates=node.create_publisher(KeyframeCandidates,'/localization/keyframe_candidates',10)
    packets=[]; node.create_subscription(KeyframeCandidates,'/localization/keyframe_candidates',packets.append,10)
    tf=TransformBroadcaster(node); client=node.create_client(Relocalize,'/localization/relocalize')
    children=[]; logs=[]; last=0; n=0
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def start(cmd,name):
        log=(a.output/name).open('w'); logs.append(log)
        p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True); children.append(p)
        return p
    def wait(predicate,timeout=35):
        end=time.monotonic()+timeout
        while not predicate():
            if any(p.poll() is not None for p in children): raise RuntimeError('Test child exited')
            if time.monotonic()>end: raise TimeoutError('Test wait timeout')
            spin(.03)
    def send():
        nonlocal last,n
        if time.monotonic()-last<.1:return
        last=time.monotonic(); stamp=node.get_clock().now().to_msg();n=stamp.sec*10**9+stamp.nanosec
        rotation=Rotation.from_matrix(ob[:3,:3]).as_quat(); t=TransformStamped()
        t.header.frame_id='odom';t.child_frame_id='base_link';t.header.stamp=stamp
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,ob[:3,3])
        t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,rotation)
        tf.sendTransform(t);m=Odometry();m.header=t.header;m.child_frame_id='base_link'
        m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,ob[:3,3]);m.pose.pose.orientation=t.transform.rotation;odom.publish(m)
        g=Vector3Stamped();g.header=t.header;v=-ob[:3,:3]@np.array(q['up_base'])*9.81
        g.vector.x,g.vector.y,g.vector.z=map(float,v);gravity.publish(g)
        pc=build_cloud(stamp,'odom',transformed(points,ob));cloud.publish(pc);local.publish(pc);long.publish(pc)
    def invoke():
        req=Relocalize.Request();req.apply_if_safe=True;f=client.call_async(req);wait(f.done);return f.result().result
    params=dict(map_pcd=str(map_path),region_labels_csv=str(map_path.parent/'region_labels.csv'),initial_region='"space_2"',
        local_map_topic='/fast_lio/local_map',global_local_map_topic='/fast_lio/local_map_global',
        region_multistart_enabled='true',region_seed_file=str(ROOT/'config/region_initialization_space.yaml'),
        keyframe_initialization_enabled='true',candidate_source='bbs',keyframe_library_sha256=db.library_sha,
        keyframe_profile_sha256=db.profile_sha,keyframe_expected_candidates='8',hierarchical_min_initial_overlap='80.0',
        retry_interval_sec='9999.0',periodic_interval_sec='0.0',max_local_map_age_sec='10.0',
        min_overlap_percent='60.0',use_prior_tracking='true',vgicp_threads='4',verbose='false',
        metrics_csv=str(a.output/'matching_metrics.csv'))
    cmd=[str(ROOT.parent/'kiss_localization_ws1.1/install/kiss_matcher_localization/lib/kiss_matcher_localization/navigation_localizer'),
        '--ros-args','-r','__ns:=/localization']
    for key,value in params.items():cmd+=['-p',f'{key}:={value}']
    results=[]
    try:
        start(cmd,'localizer.log');wait(client.service_is_ready)
        # Reject wrong source labels and empty candidate arrays before starting producer.
        send();spin(.2)
        for kind in ['incomplete','empty','wrong_map','stale']:
            m=KeyframeCandidates();m.cloud=build_cloud(node.get_clock().now().to_msg(),'odom',transformed(points,ob))
            m.window_start_ns=m.cloud.header.stamp.sec*10**9+m.cloud.header.stamp.nanosec-900000000
            m.region_id=2;m.map_sha256=db.map_sha;m.library_sha256=db.library_sha;m.profile_sha256=db.profile_sha
            m.ready=True;m.reason='bbs_competitors_complete';t=Transform();t.rotation.w=1.
            m.map_to_odom=[t];m.ids=['bbs_peak_00'];m.scores=[0.]
            if kind=='incomplete':m.reason='bbs_competitors_incomplete'
            if kind=='empty':m.map_to_odom=[];m.ids=[];m.scores=[]
            if kind=='wrong_map':m.map_sha256='0'*64
            if kind=='stale':m.cloud.header.stamp.sec-=20;m.window_start_ns-=20*10**9
            candidates.publish(m);local.publish(m.cloud);spin(.2);r=invoke()
            assert not r.matched and not r.applied,(kind,r.reason)
            results.append(dict(stage=kind,matched=r.matched,applied=r.applied,reason=r.reason))
        packets.clear()
        start(['/usr/bin/python3',str(ROOT/'scripts/bbs_retriever.py'),'--map',str(map_path),'--output',str(a.output)],'bbs_retriever.log')
        end=time.monotonic()+55; previous_stamp=0
        while time.monotonic()<end:
            send();spin(.03)
            ready=[m for m in packets if m.ready and m.reason=='bbs_competitors_complete']
            if not ready:continue
            m=ready[-1];stamp=m.cloud.header.stamp.sec*10**9+m.cloud.header.stamp.nanosec
            if stamp-previous_stamp<1100000000:continue
            previous_stamp=stamp;r=invoke()
            results.append(dict(stage='live_bbs',matched=r.matched,applied=r.applied,reason=r.reason,
                overlap=r.overlap_percent,registration_ms=r.registration_ms,candidates=len(m.ids)))
            if r.applied:break
        assert results[-1]['applied'],results
        assert any(x['matched'] and not x['applied'] for x in results if x['stage']=='live_bbs'),results
        # Genuine follow-up branch uses the short/long maps without rerunning candidates.
        send();spin(.2);r=invoke()
        results.append(dict(stage='prior',matched=r.matched,applied=r.applied,reason=r.reason,overlap=r.overlap_percent))
        assert r.applied,results
        def pump(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:send();spin(.03)
        pump(.6)
        count_before=len((a.output/'bbs_queries.csv').read_text().splitlines())
        pump(1.6)
        assert len((a.output/'bbs_queries.csv').read_text().splitlines())==count_before, 'BBS should pause during accepted prior tracking'
        send();spin(.1)
        bad=build_cloud(node.get_clock().now().to_msg(),'odom',transformed(points,ob)+100.)
        local.publish(bad);spin(.2);r=invoke()
        results.append(dict(stage='prior_rejected',matched=r.matched,applied=r.applied,reason=r.reason))
        assert r.reason=='prior_registration_rejected' and not r.applied,results
        previous_stamp=0;end=time.monotonic()+35
        while time.monotonic()<end:
            send();spin(.03)
            ready=[m for m in packets if m.ready and m.reason=='bbs_competitors_complete']
            if not ready:continue
            m=ready[-1];stamp=m.cloud.header.stamp.sec*10**9+m.cloud.header.stamp.nanosec
            if stamp-previous_stamp<1100000000:continue
            previous_stamp=stamp;r=invoke()
            results.append(dict(stage='recovery',matched=r.matched,applied=r.applied,reason=r.reason,overlap=r.overlap_percent))
            if r.applied:break
        assert results[-1]['applied'],results
        assert any(x['stage']=='recovery' and x['matched'] and not x['applied'] for x in results),results
        assert len((a.output/'bbs_queries.csv').read_text().splitlines())>count_before
        send();spin(.2);r=invoke()
        assert r.applied
        with (a.output/'matching_metrics.csv').open() as f:metrics=list(csv.DictReader(f))
        assert any(x['mode']=='bbs_multistart' for x in metrics)
        assert metrics[-1]['mode']=='prior_vgicp',metrics[-1]
        assert all(x['coarse_ran']=='0' for x in metrics)
        (a.output/'results.json').write_text(json.dumps(dict(results=results,real_recorded_shape=True,
            timestamps_rebased=True,bbs_computed_in_live_node=True,hardware_started=False,
            independent_accuracy_test=False,tracking_search_pause_verified=True,recovery_two_windows_verified=True),indent=2)+'\n')
        print(json.dumps(results,indent=2))
    finally:
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=12)
                except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=6)
        for f in logs:f.close()
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__': main()
