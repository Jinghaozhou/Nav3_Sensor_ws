#!/usr/bin/env python3
"""Shared offline/reference and future-query Scan-Context-style encoder.

Experimental 5m profile; NOT byte-compatible with upstream ScanContext defaults.
Input is a deskewed ~1s cloud in the END-STAMP IMU/base frame, with explicit up.
Never substitutes sensor-frame Z for measured/estimated gravity.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

DEFAULT_CONFIG = dict(schema_version=1, algorithm='scan_context_style_max_height_v1',
    input_frame='base_link_at_window_end', window_target_seconds=1.,
    radius_m=5., blind_m=.5, range_rule='3d_sphere_from_lidar_origin',
    voxel_m=.1, rings=20, sectors=60, height_bias_m=5.1,
    value='maximum_gravity_aligned_z_plus_numeric_bias',
    empty_cell=0., heading='base_X_projected_to_gravity_plane_else_base_Y',
    origin='lidar_origin_in_base', min_points=1000,
    upstream_reference='https://github.com/gisbi-kim/scancontext',
    compatibility='experimental_profile_not_upstream_database_compatible')


def config_hash(config):
    return hashlib.sha256(json.dumps(config, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def load_xyz(path):
    with Path(path).open('rb') as f:
        header = {}
        while True:
            line = f.readline()
            if not line:
                raise ValueError('Missing PCD DATA header')
            line = line.decode().strip()
            if line and not line.startswith('#'):
                header[line.split()[0]] = line.split()[1:]
            if line.startswith('DATA '):
                break
        if header['DATA'] != ['binary']:
            raise ValueError('Use binary PCD, not compressed/ascii')
        types = {'F': {4:'<f4',8:'<f8'}, 'U': {1:'u1',2:'<u2',4:'<u4'}, 'I': {1:'i1',2:'<i2',4:'<i4'}}
        counts = list(map(int, header.get('COUNT', ['1']*len(header['FIELDS']))))
        fields = [(n, types[t][int(s)], (c,)) if c != 1 else (n, types[t][int(s)])
                  for n,t,s,c in zip(header['FIELDS'],header['TYPE'],header['SIZE'],counts)]
        data = np.frombuffer(f.read(),dtype=np.dtype(fields))
        if len(data) != int(header['POINTS'][0]):
            raise ValueError('PCD point count mismatch')
        return np.column_stack([data[n] for n in ('x','y','z')]).astype(float)


def base_T_descriptor(up_base, lidar_origin_base):
    up = np.asarray(up_base,float); origin = np.asarray(lidar_origin_base,float)
    if up.shape!=(3,) or origin.shape!=(3,) or not np.isfinite(up).all() or not np.isfinite(origin).all() or np.linalg.norm(up)<1e-6:
        raise ValueError('Need finite, nonzero explicit gravity-up and LiDAR origin')
    z = up/np.linalg.norm(up)
    axis = np.array([1.,0,0]); x = axis-z*np.dot(axis,z); fallback=False
    if np.linalg.norm(x)<.1:
        axis=np.array([0.,1,0]);x=axis-z*np.dot(axis,z);fallback=True
    x/=np.linalg.norm(x);y=np.cross(z,x)
    t=np.eye(4);t[:3,:3]=np.column_stack([x,y,z]);t[:3,3]=origin
    return t,fallback


def encode(xyz_base, up_base, lidar_origin_base, config=None):
    c=DEFAULT_CONFIG if config is None else config
    if c.get('schema_version')!=1 or c.get('algorithm')!=DEFAULT_CONFIG['algorithm']:
        raise ValueError('Unsupported descriptor profile')
    for key in ['radius_m','voxel_m','height_bias_m','rings','sectors','min_points']:
        if not np.isfinite(c[key]) or c[key]<=0: raise ValueError('Invalid descriptor profile')
    if not 0<=c['blind_m']<c['radius_m'] or c['height_bias_m']<=c['radius_m']:
        raise ValueError('Invalid radius/bias; bias is numeric, not mounting height')
    p=np.asarray(xyz_base,float)
    if p.ndim!=2 or p.shape[1]!=3:raise ValueError('Need Nx3 points in end-stamp base frame')
    p=p[np.isfinite(p).all(axis=1)]
    t,fallback=base_T_descriptor(up_base,lidar_origin_base)
    p=(p-t[:3,3])@t[:3,:3]
    distance=np.linalg.norm(p,axis=1);p=p[(distance>=c['blind_m'])&(distance<=c['radius_m'])]
    _,indices=np.unique(np.floor(p/c['voxel_m']).astype('i8'),axis=0,return_index=True)
    p=p[np.sort(indices)]
    if len(p)<c['min_points']:raise ValueError('Too few canonical points')
    rings,sectors=int(c['rings']),int(c['sectors'])
    radial=np.hypot(p[:,0],p[:,1]);theta=np.mod(np.arctan2(p[:,1],p[:,0]),2*np.pi)
    ri=np.clip(np.ceil(radial/c['radius_m']*rings).astype(int)-1,0,rings-1)
    si=np.clip(np.ceil(theta/(2*np.pi)*sectors).astype(int)-1,0,sectors-1)
    descriptor=np.full((rings,sectors),-np.inf)
    np.maximum.at(descriptor,(ri,si),p[:,2]+c['height_bias_m'])
    occupied=np.isfinite(descriptor);descriptor[~occupied]=0
    return dict(descriptor=descriptor.astype('f4'),occupied=occupied,
                ring_key=descriptor.mean(axis=1).astype('f4'),
                sector_key=descriptor.mean(axis=0).astype('f4'),
                points=p.astype('f4'),base_T_descriptor=t,heading_fallback=fallback,
                config_sha256=config_hash(c))


def circular_distance(reference, query):
    """Returns min cosine distance and RIGHT shift of query sectors onto reference.

    Candidate generation only; do not interpret the result as a verified pose/yaw.
    """
    a=np.asarray(reference,float);b=np.asarray(query,float)
    if a.shape!=b.shape or a.ndim!=2 or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError('Incompatible descriptor arrays')
    best=(float('inf'),0)
    an=np.linalg.norm(a,axis=0)
    for shift in range(a.shape[1]):
        q=np.roll(b,shift,axis=1);bn=np.linalg.norm(q,axis=0);valid=(an>1e-12)&(bn>1e-12)
        if not valid.any():continue
        score=1-np.mean(np.sum(a[:,valid]*q[:,valid],axis=0)/(an[valid]*bn[valid]))
        if score<best[0]:best=(max(0.,float(score)),shift)
    return best


def estimate_stationary_up_odom(acc_base, rotation_odom_base, gyro_base):
    """Caller must first identify a stationary interval, with matched IMU/odom times.

    Sanity checks cannot distinguish constant linear acceleration from gravity.
    Acceleration scale may be g or m/s2, but must be consistent within the input.
    """
    a=np.asarray(acc_base,float);r=np.asarray(rotation_odom_base,float);w=np.asarray(gyro_base,float)
    if len(a)<300 or a.shape!=(len(a),3) or r.shape!=(len(a),3,3) or w.shape!=a.shape:
        raise ValueError('Need >=300 timestamp-paired stationary IMU/odom samples')
    if not all(np.isfinite(x).all() for x in (a,r,w)):
        raise ValueError('Nonfinite gravity observations')
    norm=np.linalg.norm(a,axis=1)
    if norm.mean()<1e-6 or norm.std()/norm.mean()>.03 or np.percentile(np.linalg.norm(w,axis=1),95)>.05:
        raise ValueError('Stationary gravity sanity check failed')
    up=np.median(np.einsum('nij,nj->ni',r,a),axis=0)
    return up/np.linalg.norm(up)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cloud',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--input-frame',choices=['base_link_at_window_end'],required=True)
    p.add_argument('--up-base',nargs=3,type=float,required=True)
    p.add_argument('--lidar-origin-base',nargs=3,type=float,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();c=json.loads(a.config.read_text())
    if a.output.exists():p.error('Output already exists')
    result=encode(load_xyz(a.cloud),a.up_base,a.lidar_origin_base,c)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('xb') as f:
        np.savez_compressed(f,**result)
    print(json.dumps({'output':str(a.output),'config_sha256':result['config_sha256'],
                      'points':len(result['points']),'online_publishing':False}))


if __name__=='__main__':main()
