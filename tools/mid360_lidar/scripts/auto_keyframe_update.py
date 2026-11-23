#!/usr/bin/python3
"""Post-session space_2 reference admission; immutable versions and atomic pointer.
Accepted poses are pseudo-labels, never surveyed truth. No live ROS publishers.
"""
import argparse
from bisect import bisect_right
from datetime import datetime
import fcntl
import json
from pathlib import Path
import shutil
from types import SimpleNamespace
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from build_keyframe_library import build, reader, ns, sha256, pose, transformed
from keyframe_database import KeyframeDatabase, default_library
from scan_context_descriptor import load_xyz

ROOT=Path(__file__).resolve().parents[1]
POLICY='space2_auto_v1'


def save(path,data):
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')


def map_cache(map_path, digest):
    from build_complete_reference import read_pcd
    from local_map_window import voxel_indices
    folder=ROOT/'reference_maps/auto_library_cache';folder.mkdir(exist_ok=True)
    path=folder/(digest+'.npz')
    if not path.exists():
        data,_=read_pcd(map_path);parts=[];labels=[]
        for start in range(0,len(data),500000):
            block=data[start:start+500000];xyz=np.column_stack([block[k] for k in ('x','y','z')]);finite=np.isfinite(xyz).all(axis=1)&np.isfinite(block['region_id'])
            xyz=xyz[finite];ids=block['region_id'][finite].astype(np.int32);i=voxel_indices(xyz,.1)
            parts.append(xyz[i]);labels.append(ids[i])
        xyz=np.concatenate(parts);ids=np.concatenate(labels);i=voxel_indices(xyz,.1)
        temp=path.with_suffix('.tmp.npz');np.savez_compressed(temp,xyz=xyz[i],region_id=ids[i]);temp.replace(path)
    with np.load(path,allow_pickle=False) as z:return z['xyz'].copy(),z['region_id'].copy()


def update(session,map_path):
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import Vector3Stamped
    from kiss_matcher_localization.msg import LocalizationResult
    from prepare_keyframe_descriptors import prepare
    from merge_keyframe_libraries import merge
    session=session.resolve();report=session/'auto_library_update.json'
    if report.exists():return json.loads(report.read_text())
    result=dict(status='skipped',policy=POLICY,source_session=str(session),ground_truth=False)
    def done(reason,**kw):
        result.update(reason=reason,**kw);save(report,result);return result
    if not (session/'bag/metadata.yaml').is_file():return done('bag_not_finalized')
    if not (session/'run_metadata.json').is_file():return done('no_localization_metadata')
    if shutil.disk_usage(ROOT).free<3*1024**3:return done('less_than_3GiB_reserve')
    if not (session/'acquisition_profile.json').is_file():return done('acquisition_profile_missing')
    acquisition=json.loads((session/'acquisition_profile.json').read_text())
    selection_path=ROOT/'config/keyframe_library.json'
    selection=json.loads(selection_path.read_text());active=default_library()
    current=KeyframeDatabase(active,map_path)
    digest=current.map_sha;folder=ROOT/'reference_maps/keyframe_libraries/space_2/automatic';folder.mkdir(exist_ok=True)
    work=folder/(session.name+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));work.mkdir()
    result['work_directory']=str(work)
    # All output stays in this version directory; original forty references remain untouched.
    args=SimpleNamespace(session=session,map=map_path,region='space_2',device='MID360_192.168.1.134_serial_unconfirmed',
        location_note='automatic space_2 map-label support; floor/operator pose unconfirmed',start_offset=0.,end_offset=1e9,output=work/'addition',
        window_seconds=1.,radius=5.,blind=.5,voxel=.1,distance=.5,angle=20.,min_interval=2.,max_pose_age=2.5,max_odom_gap=.15,max_cloud_gap=.3,min_points=1000,minimum_applied_count=3,minimum_applied_overlap=85.)
    try:lib=build(args)
    except ValueError as e:return done('no_usable_export',detail=str(e))
    rows=[json.loads(x) for x in (lib/'keyframes.jsonl').read_text().splitlines()]
    r=reader(session/'bag',['/fast_lio/gravity','/localization/status']);gravity={};statuses=[]
    while r.has_next():
        topic,data,_=r.read_next()
        if topic=='/fast_lio/gravity':
            m=deserialize_message(data,Vector3Stamped)
            if m.header.frame_id=='odom':gravity[ns(m.header.stamp)]=[m.vector.x,m.vector.y,m.vector.z]
        else:
            m=deserialize_message(data,LocalizationResult);statuses.append((ns(m.completed_stamp),m))
    statuses.sort(key=lambda x:x[0]);stamps=[x[0] for x in statuses];gs=np.array(sorted(gravity),dtype=np.int64)
    if len(gs)<2:return done('missing_recorded_gravity')
    gv=np.array([gravity[int(k)] for k in gs]);np.savez_compressed(work/'gravity.npz',stamp_ns=gs,gravity_odom=gv)
    xyz,labels=map_cache(map_path,digest);tree=cKDTree(xyz);region_tree=cKDTree(xyz[labels==2]);audit=[];kept=[]
    for row in rows:
        why=[];j=bisect_right(stamps,row['stamp_ns']);recent=statuses[max(0,j-3):j]
        if len(recent)<3 or any(not (m.matched and m.applied and m.overlap_percent>=85) for _,m in recent):why.append('need_three_recent_applied_85percent')
        if recent:
            transforms=[pose([m.candidate_map_to_odom.transform.translation.x,m.candidate_map_to_odom.transform.translation.y,m.candidate_map_to_odom.transform.translation.z],[m.candidate_map_to_odom.transform.rotation.x,m.candidate_map_to_odom.transform.rotation.y,m.candidate_map_to_odom.transform.rotation.z,m.candidate_map_to_odom.transform.rotation.w]) for _,m in recent]
            for a,b in zip(transforms,transforms[1:]):
                if np.linalg.norm(a[:3,3]-b[:3,3])>.3 or np.rad2deg(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude())>3:why.append('unstable_correction')
        mb=np.array(row['map_T_base']);points=transformed(load_xyz(lib/row['pcd']),mb)
        dist,_=tree.query(points);near=dist<=.3;support=float(near.mean());rmse=float(np.sqrt(np.mean(dist[near]**2))) if near.any() else 999.
        _,neighbors=tree.query(mb[:3,3],k=min(128,len(xyz)));vote=float(np.mean(labels[neighbors]==2))
        # Reference surface label support is conservative; it is not a measured free-space polygon.
        region_dist,_=region_tree.query(points);region_support=float(np.mean(region_dist<=.3))
        if support<.95 or rmse>.10:why.append('fixed_pose_geometry')
        if vote<.9 or region_support<.9:why.append('outside_or_ambiguous_space_2')
        i=int(np.searchsorted(gs,row['stamp_ns']))
        if not 0<i<len(gs) or gs[i]-gs[i-1]>300000000:why.append('gravity_gap')
        if why:
            audit.append(dict(id=row['id'],accepted=False,reasons=why));continue
        row['source_session']=str(session)
        row['acquisition_profile']=acquisition
        row['auto_admission']=dict(policy=POLICY,geometry_support=support,geometry_rmse_m=rmse,region_vote=vote,region_support=region_support,three_recent_applied=True,operator_pose_floor_confirmed=False)
        kept.append(row);audit.append(dict(id=row['id'],accepted=True,**row['auto_admission']))
    if len(kept)>24:kept=[kept[int(i)] for i in np.linspace(0,len(kept)-1,24,dtype=int)]
    save(work/'admission_review.json',dict(frames=audit,policy=POLICY))
    if not kept:return done('no_frames_passed_admission',exported=len(rows),admitted=0)
    # Descriptor builder receives only admitted rows. The raw export remains in this immutable work version.
    (lib/'keyframes_raw.jsonl').write_text((lib/'keyframes.jsonl').read_text())
    (lib/'keyframes.jsonl').write_text(''.join(json.dumps(x,ensure_ascii=False)+'\n' for x in kept))
    manifest=json.loads((lib/'manifest.json').read_text());manifest.update(index_sha256=sha256(lib/'keyframes.jsonl'),keyframes=len(kept));save(lib/'manifest.json',manifest)
    annotations=dict(source_index_sha256=manifest['index_sha256'],frames={x['id']:dict(section='space_2_auto',floor='unconfirmed',source='automatic_map_label_not_operator_confirmation') for x in kept})
    save(work/'annotations.json',annotations);save(work/'gravity_provenance.json',dict(source_session=str(session),p95_time_residual_ms=0,max_time_residual_ms=0,source='recorded_timestamped_fast_lio_gravity; interpolated <=300ms'))
    from build_keyframe_library import write_pcd
    write_pcd(work/'audit_map.pcd',xyz)
    summary=prepare(SimpleNamespace(library=lib,gravity_provenance=work/'gravity_provenance.json',gravity_track=work/'gravity.npz',annotations=work/'annotations.json',audit_map=work/'audit_map.pcd',output=lib/'descriptors_v1'))
    summary['note']='Automatic fixed-pose surface support and recorded gravity; floor/operator pose not confirmed; pseudo-labels are not surveyed truth or independent accuracy.'
    save(lib/'descriptors_v1/summary.json',summary)
    save(lib/'region_selection.json',dict(policy=POLICY,frames=audit,not_ground_truth=True))
    manifest.update(state=summary['state'],descriptor_built=True,descriptor_directory='descriptors_v1',descriptor_profile_sha256=summary['profile_sha256'],operator_session_pose_floor_confirmed=False,
        automatic_admission_policy=POLICY,region_selection_file='region_selection.json',region_selection_sha256=sha256(lib/'region_selection.json'))
    save(lib/'manifest.json',manifest);KeyframeDatabase(lib,map_path)
    merged=work/'library';review=merge(active,lib,map_path,merged,automatic=True)
    if not review['selected_addition']:return done('no_novel_frames_selected',exported=len(rows),admitted=len(kept))
    new=KeyframeDatabase(merged,map_path)
    # Full validation before atomically switching the next-session selection under a writer lock.
    selection.update(library=str(merged.relative_to(ROOT)),frozen_library=selection.get('frozen_library','reference_maps/keyframe_libraries/space_2/merged_20261111'),
        previous_library=str(active),keyframes=len(new.index),automatic_admission_policy=POLICY,status='experimental_automatic_pseudolabels_field_validation_pending')
    temp=selection_path.with_suffix('.tmp');save(temp,selection);temp.replace(selection_path)
    result['status']='updated';return done('next_session_version_activated',previous_library=str(active),library=str(merged),admitted=len(kept),merged_count=len(new.index),index_sha256=new.library_sha,retained_new=review['retained_new'])


def main():
    p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('--map',type=Path,required=True);a=p.parse_args()
    lock=ROOT/'config/.auto_library.lock'
    with lock.open('a') as f:
        fcntl.flock(f,fcntl.LOCK_EX)
        try:result=update(a.session,a.map)
        except Exception as e:
            result=dict(status='failed',reason=type(e).__name__+': '+str(e),active_library_unchanged=True);save(a.session/'auto_library_update.json',result)
    print(json.dumps(result,ensure_ascii=False,indent=2));return 1 if result['status']=='failed' else 0

if __name__=='__main__':raise SystemExit(main())
