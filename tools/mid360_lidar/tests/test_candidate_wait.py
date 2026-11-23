"""Production C++ candidate wait/dedup/reset protocol in an isolated ROS domain.

Synthetic geometry tests the state machine, not field localization accuracy.
"""
import argparse,csv,hashlib,json,os,signal,subprocess,sys,time
from pathlib import Path
import numpy as np
import rclpy,yaml
from geometry_msgs.msg import Transform,TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster
from kiss_matcher_localization.msg import KeyframeCandidates
from kiss_matcher_localization.srv import Relocalize,SetRegion

assert os.environ.get('ROS_DOMAIN_ID') not in (None,'','0')
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from local_map_window import build_cloud

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    xyz=np.random.default_rng(17).uniform([-3,-3,-1],[3,3,3],(7000,3)).astype('f4')
    data=np.zeros(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('region_id','<i4')])
    data['x'],data['y'],data['z']=xyz.T;data['region_id']=2
    mapfile=a.output/'map.pcd'
    mapfile.write_bytes((f'VERSION 0.7\nFIELDS x y z region_id\nSIZE 4 4 4 4\nTYPE F F F I\nCOUNT 1 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nPOINTS {len(xyz)}\nDATA binary\n').encode()+data.tobytes())
    sha=hashlib.sha256(mapfile.read_bytes()).hexdigest()
    profile=a.output/'seeds.yaml';profile.write_text(yaml.safe_dump(dict(schema_version=1,map_frame='map',base_frame='base_link',regions=[dict(region_id=2,seeds=[dict(id='known',position=[0,0,0],orientation_xyzw=[0,0,0,1])])])) )
    labels=a.output/'labels.csv';labels.write_text('region_id,region_name\n2,space_2\n')
    rclpy.init();node=rclpy.create_node('candidate_wait_protocol_test')
    pub=node.create_publisher(KeyframeCandidates,'/localization/keyframe_candidates',10)
    local=node.create_publisher(PointCloud2,'/test/local',10);br=TransformBroadcaster(node)
    client=node.create_client(Relocalize,'/localization/relocalize');region=node.create_client(SetRegion,'/localization/set_region')
    params=dict(map_pcd=str(mapfile),region_labels_csv=str(labels),initial_region='"space_2"',local_map_topic='/test/local',
        global_local_map_topic='/test/local',region_multistart_enabled='true',region_seed_file=str(profile),
        keyframe_initialization_enabled='true',candidate_source='bbs',keyframe_library_sha256='b'*64,keyframe_profile_sha256='c'*64,
        keyframe_expected_candidates='8',hierarchical_min_initial_overlap='80.0',retry_interval_sec='9999.0',periodic_interval_sec='0.0',
        max_local_map_age_sec='10.0',min_overlap_percent='60.0',vgicp_threads='2',verbose='false',metrics_csv=str(a.output/'metrics.csv'))
    cmd=[str(ROOT.parent/'kiss_localization_ws1.1/install/kiss_matcher_localization/lib/kiss_matcher_localization/navigation_localizer'),'--ros-args','-r','__ns:=/localization']
    for k,v in params.items():cmd+=['-p',f'{k}:={v}']
    log=(a.output/'node.log').open('w');child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    results=[]
    def spin(sec):
        end=time.monotonic()+sec
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    def call(service,req):
        f=service.call_async(req);end=time.monotonic()+20
        while not f.done():
            assert child.poll() is None
            if time.monotonic()>end:raise TimeoutError('service')
            spin(.02)
        return f.result()
    def send(m):
        local.publish(m.cloud);pub.publish(m);spin(.12)
    def packet(ready=True,reason='bbs_competitors_complete'):
        stamp=node.get_clock().now().to_msg();t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='odom';t.child_frame_id='base_link';t.transform.rotation.w=1.;br.sendTransform(t)
        m=KeyframeCandidates();m.cloud=build_cloud(stamp,'odom',xyz);m.window_start_ns=stamp.sec*10**9+stamp.nanosec-900000000
        m.region_id=2;m.map_sha256=sha;m.library_sha256='b'*64;m.profile_sha256='c'*64;m.ready=ready;m.reason=reason
        if ready:
            v=Transform();v.rotation.w=1.;m.map_to_odom=[v];m.ids=['bbs_peak_00'];m.scores=[0.]
        return m
    def attempt(stage):
        req=Relocalize.Request();req.apply_if_safe=True;r=call(client,req).result
        with (a.output/'metrics.csv').open() as f:row=list(csv.DictReader(f))[-1]
        item=dict(stage=stage,matched=r.matched,applied=r.applied,reason=r.reason,confirmations=int(row['confirmation_count']),fine_ran=row['fine_ran'])
        results.append(item);return item
    def reset():
        req=SetRegion.Request();req.region='space_2';assert call(region,req).success
    try:
        end=time.monotonic()+30
        while not (client.service_is_ready() and region.service_is_ready()):
            assert child.poll() is None
            if time.monotonic()>end:raise TimeoutError('startup')
            spin(.05)
        spin(.3)
        # Consume the automatic first-input attempt before service-driven cases.
        local.publish(packet().cloud);spin(.4)
        with (a.output/'metrics.csv').open() as f:priming=list(csv.DictReader(f))
        assert priming and priming[-1]['reason']=='keyframe_query_unavailable',priming
        first=packet();send(first);r=attempt('first');assert r['matched'] and not r['applied'] and r['confirmations']==1,r
        r=attempt('duplicate');assert r['reason']=='keyframe_query_already_processed' and r['confirmations']==1 and r['fine_ran']=='0',r
        for why in ['short_cloud_warmup','bbs_search_pending','query_odom_missing','estimated_gravity_missing_at_query_stamp']:
            send(packet(False,why));r=attempt(why);assert not r['matched'] and not r['applied'] and r['confirmations']==1,r
        spin(1.05);send(packet());r=attempt('second');assert r['applied'] and r['confirmations']==2,r
        for kind in ['hard_error','wrong_version','wrong_wait_version','stale']:
            reset();send(packet());r=attempt(kind+'_first');assert r['confirmations']==1 and not r['applied'],r
            m=packet(False,'worker_failed') if kind=='hard_error' else packet(False,'short_cloud_warmup') if kind=='wrong_wait_version' else packet()
            if kind=='wrong_version':m.map_sha256='0'*64
            if kind=='wrong_wait_version':m.map_sha256='0'*64
            if kind=='stale':m.cloud.header.stamp.sec-=20;m.window_start_ns-=20*10**9
            send(m);r=attempt(kind);assert not r['applied'] and r['confirmations']==0,r
            spin(1.05);send(packet());r=attempt(kind+'_fresh');assert not r['applied'] and r['confirmations']==1,r
        (a.output/'results.json').write_text(json.dumps(results,indent=2)+'\n')
        print(json.dumps(results,indent=2))
    finally:
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGINT);child.wait(timeout=15)
        log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
