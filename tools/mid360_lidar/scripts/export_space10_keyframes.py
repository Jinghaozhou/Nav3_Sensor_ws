#!/usr/bin/python3
"""Export measured space_10 draft references; never activate or merge virtual templates."""
import argparse,json
from pathlib import Path
from types import SimpleNamespace
from build_keyframe_library import build
ROOT=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('session',type=Path);p.add_argument('--start-offset',type=float,required=True);p.add_argument('--end-offset',type=float,required=True);p.add_argument('--output',type=Path);a=p.parse_args()
 if not 0<=a.start_offset<a.end_offset:p.error('Specify a reviewed valid time interval relative to the first processed cloud')
 if not (a.session/'bag/metadata.yaml').is_file():p.error('Stop recording and wait for bag finalization before export')
 meta=json.loads((a.session/'run_metadata.json').read_text())
 if meta.get('initial_region') not in ('space_10','10'):p.error('Session was not initialized for space_10')
 map_path=Path(meta['map_pcd']);out=a.output or ROOT/'reference_maps/keyframe_libraries/space_10'/a.session.name
 args=SimpleNamespace(session=a.session,map=map_path,region='space_10',device='MID360_192.168.1.134_serial_unconfirmed',location_note='operator: third-floor navigation workstation area; Go2 0.5m 20deg down; per-frame region/pose review pending',start_offset=a.start_offset,end_offset=a.end_offset,output=out,window_seconds=1.,radius=5.,blind=.5,voxel=.1,distance=.5,angle=20.,min_interval=2.,max_pose_age=2.5,max_odom_gap=.15,max_cloud_gap=.3,min_points=1000,minimum_applied_count=3,minimum_applied_overlap=85.)
 result=build(args);path=result/'manifest.json';m=json.loads(path.read_text());m.update(source='measured_go2_pointclouds_bootstrapped_from_unreviewed_online_poses',bootstrap_candidate_source=meta.get('candidate_source'),operator_session_pose_floor_confirmed=False,requires_region_and_geometry_review=True,virtual_templates_included=False,online_enabled=False);path.write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n')
 print('DRAFT ONLY: review per-frame region, floor, pose, geometry and descriptors before activation:',result)
if __name__=='__main__':main()
