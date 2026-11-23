#!/usr/bin/python3
"""Build a persistent BBS model from the active labeled map; never starts ROS."""
import argparse
import json
from pathlib import Path
import subprocess
import numpy as np
from build_complete_reference import read_pcd
from build_keyframe_library import sha256
from keyframe_database import default_library

ROOT = Path(__file__).resolve().parents[1]

def prepare(config_path, map_path, library, output):
    if output.exists():
        raise ValueError('Use a new output directory; do not overwrite a model')
    config = json.loads(config_path.read_text())
    ref = json.loads((library/'manifest.json').read_text())
    map_sha = sha256(map_path)
    if ref['map_sha256'] != map_sha:
        raise ValueError('Map/calibration SHA mismatch')
    rows = [json.loads(x) for x in (library/'keyframes.jsonl').read_text().splitlines()]
    idx = [json.loads(x) for x in (library/'descriptors_v1/index.jsonl').read_text().splitlines()]
    ups = np.array([np.array(r['map_T_base'])[:3,:3] @ i['up_base'] for r,i in zip(rows,idx)])
    up = np.median(ups,axis=0); up /= np.linalg.norm(up)
    x = np.array([1.,0,0]); x -= up*(x@up); x /= np.linalg.norm(x)
    basis = np.column_stack([x,np.cross(up,x),up])
    data, info = read_pcd(map_path)
    if 'region_id' not in data.dtype.names:
        raise ValueError('Labeled region_id map required')
    region = config['region_id']; low = np.full(3,np.inf); high = -low
    for start in range(0,len(data),500000):
        part = data[start:start+500000]; part = part[part['region_id']==region]
        if not len(part): continue
        xyz = np.column_stack([part[k] for k in ('x','y','z')]) @ basis
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if len(xyz): low=np.minimum(low,xyz.min(0)); high=np.maximum(high,xyz.max(0))
    if not np.isfinite(low).all(): raise ValueError('Region has no finite points')
    # Bounds describe sensor origins in this region; target also includes visible neighbours.
    low -= .35; high += .35; chunks=[]
    for start in range(0,len(data),500000):
        part = data[start:start+500000]
        xyz = np.column_stack([part[k] for k in ('x','y','z')]) @ basis
        xyz = xyz[np.isfinite(xyz).all(axis=1) & np.all((xyz>=low-5)&(xyz<=high+5),axis=1)]
        if not len(xyz): continue
        _, unique = np.unique(np.floor(xyz/.1).astype('i8'),axis=0,return_index=True)
        chunks.append(xyz[unique])
    points=np.concatenate(chunks); _,unique=np.unique(np.floor(points/.1).astype('i8'),axis=0,return_index=True)
    points=points[unique]; output.mkdir(parents=True)
    points.astype('<f4').tofile(output/'target.bin')
    (output/'worker.cfg').write_text(' '.join(map(str,list(low)+list(high)+[
        config['voxel_m'],config['levels'],config['threads'],config['timeout_ms'],
        config['minimum_occupancy_fraction'],config['competition_score_gap'],
        config['peak_translation_m'],np.deg2rad(config['peak_rotation_deg']),config['maximum_candidates']]))+'\n')
    manifest=dict(schema_version=1,region_id=region,map_sha256=map_sha,
        config_sha256=sha256(config_path),target_sha256=sha256(output/'target.bin'),
        worker_config_sha256=sha256(output/'worker.cfg'),basis_map_from_gravity=basis.tolist(),
        lidar_origin_base=rows[0]['lidar_origin_base'],bounds=[low.tolist(),high.tolist()],
        target_points=len(points),calibration_index_sha256=sha256(library/'keyframes.jsonl'),
        note='Reference poses calibrate map gravity only; no reference position/yaw hypotheses are loaded at runtime. Not field accuracy validation.')
    (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    build_worker()
    print(json.dumps(manifest,ensure_ascii=False))

def build_worker():
    src=ROOT/'vendor/3d_bbs/bbs3d'; dest=ROOT/'build/bbs'; dest.mkdir(parents=True,exist_ok=True)
    files=[ROOT/'scripts/bbs_worker.cpp']+[src/'src/cpu_bbs3d'/n for n in ['bbs3d.cpp','voxelmaps.cpp']]
    subprocess.run(['g++','-O3','-std=c++17','-fopenmp','-I/usr/include/eigen3',
        '-I'+str(src/'include'),*[str(p) for p in files],'-o',str(dest/'bbs_worker')],check=True)
    (dest/'source_sha256.json').write_text(json.dumps({str(p.relative_to(ROOT)):sha256(p)
        for p in files+list((src/'include').rglob('*.hpp'))},indent=2)+'\n')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'config/bbs_initialization.json')
    p.add_argument('--map',type=Path,default=__import__('workspace_paths').map_file())
    p.add_argument('--calibration-library',type=Path,default=default_library())
    p.add_argument('--output',type=Path,default=ROOT/'reference_maps/bbs_space_2')
    p.add_argument('--build-only',action='store_true')
    a=p.parse_args()
    if a.build_only: build_worker()
    else: prepare(a.config,a.map,a.calibration_library,a.output)

if __name__=='__main__': main()
