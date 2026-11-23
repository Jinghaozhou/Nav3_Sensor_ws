#!/usr/bin/env python3
"""Audit reference geometry and build experimental gravity-aligned 5m descriptors.
No nodes, TF publication, pose optimization, or online activation.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scan_context_descriptor import DEFAULT_CONFIG, config_hash, load_xyz, encode, circular_distance
from build_keyframe_library import sha256, write_pcd


def prepare(a):
    lib=a.library.resolve();manifest=json.loads((lib/'manifest.json').read_text())
    if sha256(Path(manifest['map_path']))!=manifest['map_sha256']:
        raise ValueError('Reference map content changed')
    if sha256(lib/'keyframes.jsonl')!=manifest['index_sha256']:
        raise ValueError('Original keyframe index changed')
    meta=json.loads(a.gravity_provenance.read_text())
    if Path(meta['source_session']).resolve()!=Path(manifest['source_session']).resolve():
        raise ValueError('Gravity data belongs to another scan')
    if meta['p95_time_residual_ms']>10 or meta['max_time_residual_ms']>20:
        raise ValueError('Gravity/state time alignment too uncertain')
    annotations=json.loads(a.annotations.read_text())
    if annotations['source_index_sha256']!=manifest['index_sha256']:
        raise ValueError('Route annotations belong to another library')
    track=np.load(a.gravity_track,allow_pickle=False);ts=track['stamp_ns'];g=track['gravity_odom']
    if len(ts)<2 or np.any(np.diff(ts)<=0) or not np.isfinite(g).all():
        raise ValueError('Invalid gravity track')
    if np.any((np.linalg.norm(g,axis=1)<8)|(np.linalg.norm(g,axis=1)>12)):
        raise ValueError('Gravity must be Fast-LIO odom-state gravity in m/s2')
    rows=[json.loads(line) for line in (lib/'keyframes.jsonl').read_text().splitlines()]
    full_map=load_xyz(a.audit_map);tree=cKDTree(full_map)
    if a.output.exists():raise ValueError('Descriptor output already exists')
    a.output.mkdir(parents=True);(a.output/'canonical_clouds').mkdir()
    config=dict(DEFAULT_CONFIG);(a.output/'profile.json').write_text(json.dumps(config,indent=2))
    audit=[];index=[];descriptors=[];masks=[];rings=[];sectors=[]
    for r in rows:
        p=lib/r['pcd']
        if sha256(p)!=r['pcd_sha256']:raise ValueError('PCD hash mismatch')
        n=r['stamp_ns'];i=int(np.searchsorted(ts,n))
        if not 0<i<len(ts) or ts[i]-ts[i-1]>300000000:
            raise ValueError('Missing timestamp-paired gravity')
        f=float(n-ts[i-1])/float(ts[i]-ts[i-1]);gravity=(1-f)*g[i-1]+f*g[i]
        ob=np.array(r['odom_T_base']);mb=np.array(r['map_T_base']);xyz=load_xyz(p)
        up_base=ob[:3,:3].T@(-gravity);up_base/=np.linalg.norm(up_base)
        encoded=encode(xyz,up_base,r['lidar_origin_base'],config)
        # Independent fixed-pose map-support check: no registration refinement.
        world=xyz@mb[:3,:3].T+mb[:3,3];dist,_=tree.query(world)
        near=dist<=.3;support=float(near.mean());rmse=float(np.sqrt(np.mean(dist[near]**2))) if near.any() else None
        rigid_error=float(np.linalg.norm(mb[:3,:3].T@mb[:3,:3]-np.eye(3)))
        passed=support>=.8 and rigid_error<1e-4
        label=annotations['frames'][r['id']]
        cp=a.output/'canonical_clouds'/Path(r['pcd']).name;write_pcd(cp,encoded['points'])
        frame=dict(id=r['id'],stamp_ns=n,source_pcd=r['pcd'],source_pcd_sha256=r['pcd_sha256'],
            canonical_pcd=str(cp.relative_to(a.output)),canonical_sha256=sha256(cp),
            base_T_descriptor=encoded['base_T_descriptor'].tolist(),
            map_T_descriptor=(mb@encoded['base_T_descriptor']).tolist(),
            up_base=up_base.tolist(),gravity_odom=gravity.tolist(),
            profile_sha256=encoded['config_sha256'],heading_fallback=encoded['heading_fallback'],
            source_points=len(xyz),canonical_points=len(encoded['points']),
            occupied_cells=int(encoded['occupied'].sum()),route=label,
            geometry_check_passed=passed)
        index.append(frame);descriptors.append(encoded['descriptor']);masks.append(encoded['occupied']);rings.append(encoded['ring_key']);sectors.append(encoded['sector_key'])
        audit.append(dict(id=r['id'],stamp_ns=n,map_x=mb[0,3],map_y=mb[1,3],map_z=mb[2,3],
            route_section=label['section'],floor=label['floor'],floor_source=label['source'],
            nearest_map_30cm_fraction=support,nearest_map_inlier_rmse_m=rmse,
            pose_rigidity_error=rigid_error,geometry_check_passed=passed,
            surveyed_ground_truth=False))
    with (a.output/'descriptors.npz').open('xb') as f:
        np.savez_compressed(f,ids=np.array([r['id'] for r in rows]),descriptors=np.stack(descriptors),
            occupied=np.stack(masks),ring_keys=np.stack(rings),sector_keys=np.stack(sectors),
            profile_sha256=np.array(config_hash(config)))
    (a.output/'index.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in index))
    with (a.output/'geometry_review.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(audit[0]));w.writeheader();w.writerows(audit)
    # Internal consistency test only; self retrieval says nothing about field success.
    self_checks=[]
    for i,d in enumerate(descriptors):
        score,shift=circular_distance(d,d)
        if score>1e-6 or shift!=0:raise ValueError('Descriptor self consistency failed')
        self_checks.append(dict(id=rows[i]['id'],distance=score,right_shift=shift))
    matrix=np.array([[circular_distance(x,y)[0] for y in descriptors] for x in descriptors])
    np.save(a.output/'pairwise_distance.npy',matrix,allow_pickle=False)
    result=dict(state='geometry_checked_descriptors_ready' if all(r['geometry_check_passed'] for r in audit) else 'geometry_review_required',
        keyframes=len(rows),geometry_passed=sum(r['geometry_check_passed'] for r in audit),
        min_map_30cm_fraction=min(r['nearest_map_30cm_fraction'] for r in audit),
        max_map_inlier_rmse_m=max(r['nearest_map_inlier_rmse_m'] for r in audit),
        profile_sha256=config_hash(config),map_sha256=manifest['map_sha256'],
        source_index_sha256=manifest['index_sha256'],gravity_track_sha256=sha256(a.gravity_track),
        gravity_provenance=meta,annotations_sha256=sha256(a.annotations),
        audit_map_path=str(a.audit_map.resolve()),audit_map_sha256=sha256(a.audit_map),
        self_checks=self_checks,online_enabled=False,independent_retrieval_test_passed=False,
        note='Map support is fixed-pose nearest-neighbor geometry, NOT VGICP overlap or surveyed accuracy; see per-frame route source for manual, inferred, or unconfirmed floor provenance')
    (a.output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ['self_checks','gravity_provenance']},ensure_ascii=False,indent=2))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['library','gravity-track','gravity-provenance','annotations','audit-map','output']:
        p.add_argument('--'+name,type=Path,required=True)
    prepare(p.parse_args())


if __name__=='__main__':main()
