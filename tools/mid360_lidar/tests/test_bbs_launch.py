"""Combined localization launch smoke check, explicitly isolated and no hardware."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from kiss_matcher_localization.srv import Relocalize

assert os.environ.get('ROS_DOMAIN_ID') not in (None,'','0')
ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
assert not args.output.exists()
child=subprocess.Popen(['ros2','launch',str(ROOT/'launch/continuous_localization.launch.py'),
    'candidate_source:=bbs','keyframe_initialization_enabled:=true','region_multistart_enabled:=true',
    'initial_region:=space_2','rviz:=false','output:='+str(args.output)],start_new_session=True)
rclpy.init();node=rclpy.create_node('bbs_launch_smoke_readiness')
client=node.create_client(Relocalize,'/localization/relocalize')
try:
    deadline=time.monotonic()+80
    while not client.service_is_ready():
        assert child.poll() is None,'Combined launch exited early'
        assert time.monotonic()<deadline,'Localizer service did not become ready'
        rclpy.spin_once(node,timeout_sec=.1)
    metadata=json.loads((args.output/'run_metadata.json').read_text())
    assert metadata['candidate_source']=='bbs'
    assert metadata['bbs_maximum_candidates']==8
    assert metadata['hierarchical_min_initial_overlap']==80
    print('PASS combined launch: BBS source, metadata and running process; no hardware started.',flush=True)
finally:
    node.destroy_node();rclpy.shutdown()
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
        try:child.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=10)
assert child.returncode==0,child.returncode
print('PASS combined launch clean shutdown.',flush=True)
