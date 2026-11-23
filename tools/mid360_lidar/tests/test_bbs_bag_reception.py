"""Replay recorded processed clouds/odom/gravity into production async BBS.

No raw radar, Fast-LIO or RViz is started. Original bag is opened read-only;
timestamps are uniformly rebased and events retain recorded receive intervals.
"""
import argparse,csv,json,os,signal,sqlite3,subprocess,time
from pathlib import Path
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from kiss_matcher_localization.srv import Relocalize
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from bbs_database import BbsDatabase

def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'','0')
    p=argparse.ArgumentParser();p.add_argument('--session',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=42.);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    names={'/fast_lio/local_cloud','/fast_lio/odom','/fast_lio/gravity','/fast_lio/local_map','/fast_lio/local_map_global'}
    events=[]
    for path in sorted((a.session/'bag').glob('*.db3')):
        db=sqlite3.connect('file:'+str(path)+'?mode=ro',uri=True)
        topics={i:(name,get_message(typ)) for i,name,typ in db.execute('select id,name,type from topics') if name in names}
        for tid,(name,cls) in topics.items():
            for n,data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
                events.append((n,name,cls,data))
        db.close()
    events.sort(key=lambda x:x[0]);assert events
    first=events[0][0];events=[e for e in events if e[0]<=first+a.seconds*1e9]
    mapfile=ROOT.parent/'kiss_localization_ws1.1/src/localization/maps/building_split_space.pcd'
    db=BbsDatabase(mapfile);db.close()
    params=dict(map_pcd=str(mapfile),region_labels_csv=str(mapfile.parent/'region_labels.csv'),initial_region='"space_2"',
        local_map_topic='/fast_lio/local_map',global_local_map_topic='/fast_lio/local_map_global',region_multistart_enabled='true',
        region_seed_file=str(ROOT/'config/region_initialization_space.yaml'),keyframe_initialization_enabled='true',candidate_source='bbs',
        keyframe_library_sha256=db.library_sha,keyframe_profile_sha256=db.profile_sha,keyframe_expected_candidates='8',
        hierarchical_min_initial_overlap='80.0',retry_interval_sec='1.0',periodic_interval_sec='1.0',max_local_map_age_sec='10.0',
        min_overlap_percent='60.0',use_prior_tracking='true',vgicp_threads='4',verbose='false',metrics_csv=str(a.output/'metrics.csv'))
    cmd=[str(ROOT.parent/'kiss_localization_ws1.1/install/kiss_matcher_localization/lib/kiss_matcher_localization/navigation_localizer'),'--ros-args','-r','__ns:=/localization']
    for k,v in params.items():cmd+=['-p',f'{k}:={v}']
    logs=[];children=[];counts={name:0 for name in names};max_delay=0
    def start(cmd,name):
        f=(a.output/name).open('w');logs.append(f);c=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(c)
    rclpy.init();node=rclpy.create_node('bbs_recorded_receive_replay');br=TransformBroadcaster(node)
    types={name:cls for _,name,cls,_ in events}
    pubs={name:node.create_publisher(cls,name,10) for name,cls in types.items()}
    client=node.create_client(Relocalize,'/localization/relocalize')
    try:
        start(cmd,'localizer.log');deadline=time.monotonic()+60
        while not client.service_is_ready():
            assert children[0].poll() is None
            if time.monotonic()>deadline:raise TimeoutError('startup')
            rclpy.spin_once(node,timeout_sec=.05)
        start(['/usr/bin/python3',str(ROOT/'scripts/bbs_retriever.py'),'--map',str(mapfile),'--output',str(a.output)],'bbs.log')
        # Model loading may take a few seconds; replay starts after node discovery.
        deadline=time.monotonic()+20
        while not any(name=='bbs_retriever' for name,_ in node.get_node_names_and_namespaces()):
            if time.monotonic()>deadline:raise TimeoutError('BBS startup')
            rclpy.spin_once(node,timeout_sec=.05)
        begin=time.monotonic()+.4;offset=node.get_clock().now().nanoseconds+400000000-first
        for record,name,cls,data in events:
            target=begin+(record-first)/1e9
            while time.monotonic()<target:
                rclpy.spin_once(node,timeout_sec=min(.01,max(0,target-time.monotonic())))
            assert all(c.poll() is None for c in children)
            max_delay=max(max_delay,time.monotonic()-target)
            m=deserialize_message(data,cls);n=m.header.stamp.sec*10**9+m.header.stamp.nanosec+offset
            m.header.stamp.sec=n//10**9;m.header.stamp.nanosec=n%10**9
            if name=='/fast_lio/odom':
                t=TransformStamped();t.header=m.header;t.child_frame_id=m.child_frame_id
                t.transform.translation.x=m.pose.pose.position.x;t.transform.translation.y=m.pose.pose.position.y;t.transform.translation.z=m.pose.pose.position.z
                t.transform.rotation=m.pose.pose.orientation;br.sendTransform(t)
            pubs[name].publish(m);counts[name]+=1;rclpy.spin_once(node,timeout_sec=0)
    finally:
        for c in reversed(children):
            if c.poll() is None:os.killpg(c.pid,signal.SIGINT);c.wait(timeout=20)
        for f in logs:f.close()
        node.destroy_node();rclpy.shutdown()
    with (a.output/'metrics.csv').open() as f:rows=list(csv.DictReader(f))
    assert all(c.returncode==0 for c in children),[c.returncode for c in children]
    successes=[r for r in rows if r['applied']=='1'];assert successes,rows
    t0=int(rows[0]['completed_ns'])-round(float(rows[0]['total_ms'])*1e6)
    summary=dict(source=str(a.session),seconds=a.seconds,hardware_started=False,rviz_started=False,timestamps_rebased=True,
        counts=counts,maximum_publish_lateness_sec=max_delay,attempts=len(rows),matched=sum(r['matched']=='1' for r in rows),
        applied=len(successes),first_attempt_to_application_sec=(int(successes[0]['completed_ns'])-t0)/1e9,
        first_application_overlap=float(successes[0]['overlap_percent']),independent_accuracy_test=False)
    (a.output/'results.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
