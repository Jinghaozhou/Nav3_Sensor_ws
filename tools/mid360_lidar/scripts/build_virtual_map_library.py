#!/usr/bin/python3
"""Offline experimental map-only station sampling and LiDAR voxel raycasting.
No ROS nodes, no activation of a reference database, no claimed free-space truth.
"""
from pathlib import Path
import argparse,csv,json,subprocess,time
import numpy as np
from scipy.spatial import cKDTree,ConvexHull
from scipy.spatial.transform import Rotation
from build_keyframe_library import sha256,write_pcd
from scan_context_descriptor import encode,base_T_descriptor,DEFAULT_CONFIG
ROOT=Path(__file__).resolve().parents[1]

def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')

def generate(a):
 out=a.output.absolute();out.mkdir(parents=True,exist_ok=True)
 lib=a.reference_library;manifest=json.loads((lib/'manifest.json').read_text());rows=[json.loads(x) for x in (lib/'keyframes.jsonl').read_text().splitlines()];idx=[json.loads(x) for x in (lib/'descriptors_v1/index.jsonl').read_text().splitlines()]
 if sha256(a.map)!=manifest['map_sha256']:raise ValueError('Map SHA changed')
 # Calibration of gravity and mounting from reference poses; station positions are
 # generated solely from map geometry, never copied or shifted from reference route.
 ups=np.array([np.array(x['map_T_base'])[:3,:3]@y['up_base'] for x,y in zip(rows,idx)]);up=np.median(ups,0);up/=np.linalg.norm(up);gx=np.array([1.,0,0]);gx-=up*(gx@up);gx/=np.linalg.norm(gx);basis=np.column_stack([gx,np.cross(up,gx),up])
 ub=np.asarray(a.up_base,float) if a.up_base is not None else np.median([x['up_base'] for x in idx],0)
 if not np.isfinite(ub).all() or np.linalg.norm(ub)<1e-6:raise ValueError('Need finite nonzero base-frame gravity-up')
 ub/=np.linalg.norm(ub);offset=np.array(rows[0]['lidar_origin_base']);bd,_=base_T_descriptor(ub,offset);tilt=bd[:3,:3].T
 cache=np.load(a.map_cache);xyz=cache['xyz'];regions=cache['regions'];full=xyz@basis;s=full[regions==a.region];low=s.min(0);high=s.max(0);near=np.all((full>=low-[1,1,2])&(full<=high+[1,1,1]),1);p=full[near];labels=regions[near];tree=cKDTree(p)
 # Small-neighbour PCA retains stair/ramp surfaces as well as broad landings.
 d,ix=tree.query(p,k=8,workers=2);v=p[ix];v-=v.mean(1)[:,None,:];cov=np.einsum('nki,nkj->nij',v,v)/8;ev,vec=np.linalg.eigh(cov)
 terrain=(abs(vec[:,2,0])>np.cos(np.deg2rad(50)))&(ev[:,0]<.035**2)&(ev[:,0]/np.maximum(ev.sum(1),1e-12)<.08)&(d[:,-1]<.4)
 if getattr(a,'support_z_min',None) is not None:
  terrain &= (p[:,2]>=a.support_z_min)&(p[:,2]<=a.support_z_max)
 support=p[terrain];support_labels=labels[terrain]
 # Select actual local support samples in quantized XY-height bins; do not average
 # different floors into a fictitious middle position.
 keys=np.column_stack([np.floor(support[:,:2]/.22).astype('i8'),np.floor(support[:,2]/.1).astype('i8')]);_,unique=np.unique(keys,axis=0,return_index=True);proposals=support[unique]
 regiontree=cKDTree(full);othertree=cKDTree(full[regions!=a.region]);hull=ConvexHull(s);eq=hull.equations
 accepted=[];rejected=[];heights=a.heights
 for foot in proposals:
  local=support[np.linalg.norm(support[:,:2]-foot[:2],axis=1)<.24];local=local[abs(local[:,2]-foot[2])<.11]
  if len(local)<4 or np.linalg.eigvalsh(np.cov(local[:,:2].T))[0]<.0015:continue
  # Eye = physical LiDAR origin, not IMU origin. Body/base is derived with offset later.
  for height in heights:
   eye=foot+[0,0,height]
   if max(eq[:,:3]@eye+eq[:,3])>.35:continue
   dd,ii=regiontree.query(eye,k=128);vote=float(np.mean(regions[ii]==a.region));dr=float(np.min(dd[regions[ii]==a.region])) if np.any(regions[ii]==a.region) else 1e6;clear=float(othertree.query(eye)[0]-dr)
   if vote<.9 or clear<.15:continue
   # Surface-only evidence: no occupied samples in a slim vertical body envelope.
   occupied=tree.query_ball_point(eye,height+.25);around=p[occupied];xy=np.linalg.norm(around[:,:2]-eye[:2],axis=1);block=(xy<.15)&(around[:,2]>foot[2]+.22)&(around[:,2]<eye[2]+.15)
   sensor_clear=float(tree.query(eye)[0])
   if block.any() or sensor_clear<.22:continue
   if any(np.linalg.norm(eye-np.array(x['eye_gravity']))<a.spacing and abs(height-x['height_m'])<.01 for x in accepted):continue
   accepted.append(dict(id=f'station_{len(accepted):04d}',foot_gravity=foot.tolist(),eye_gravity=eye.tolist(),eye_map=(basis@eye).tolist(),height_m=height,region_id=a.region,region_vote128=vote,other_region_clearance_m=clear,sensor_surface_clearance_m=sensor_clear,review_status='map_surface_checked_operator_review_pending',free_space_certified=False))
 if not accepted:raise ValueError('No supported station; inspect terrain, region or height')
 (out/'stations.jsonl').write_text(''.join(json.dumps(x,ensure_ascii=False)+'\n' for x in accepted));np.savez_compressed(out/'geometry.npz',points=p,regions=labels,support=support,basis=basis)
 xyzgrav=full.astype('<f4');xyzgrav.tofile(out/'render_map.bin');np.savetxt(out/'lidar_offset.txt',offset)
 profiles=[]
 for station in accepted:
  for yaw in np.arange(0,360,a.yaw_step):
   r=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@tilt;eye=np.array(station['eye_gravity']);mb=np.eye(4);mb[:3,:3]=basis@r;mb[:3,3]=basis@(eye-r@offset)
   profiles.append(dict(id=f'virtual_{len(profiles):05d}',station=station['id'],region_id=a.region,yaw_gravity_deg=float(yaw),eye_gravity=eye.tolist(),rotation_gravity_base=r.tolist(),map_T_base=mb.tolist(),up_base=(r.T@np.array([0.,0,1.])).tolist(),height_m=station['height_m']))
 np.savetxt(out/'render_poses.txt',np.array([x['eye_gravity']+np.array(x['rotation_gravity_base']).ravel().tolist() for x in profiles]))
 refs=np.array([x['map_T_base'] for x in rows])[:,:3,3]@basis;centers=np.array([x['eye_gravity'] for x in accepted]);distance,nearest=cKDTree(centers).query(refs)
 save(out/'reference_position_coverage.json',dict(note='Approximate eye-to-old-base distance; mounting offset below 6cm; not independent localization recall',frames=[dict(id=x['id'],distance_m=float(d),nearest_station=accepted[int(i)]['id']) for x,d,i in zip(rows,distance,nearest)]))
 result=dict(region=a.region,stations=len(accepted),poses=len(profiles),height_m=heights,map_sha256=manifest['map_sha256'],map_cache_sha256=sha256(a.map_cache),source_map_cache=str(a.map_cache.resolve()),source_reference_index_sha256=sha256(lib/'keyframes.jsonl'),calibration_only_reference_positions_not_used_for_sampling=True,gravity_up_map=up.tolist(),reference_gravity_spread_max_deg=float(np.rad2deg(np.arccos(np.clip(ups@up,-1,1))).max()),basis_map_from_gravity=basis.tolist(),assumed_up_base=ub.tolist(),sensor_offset_base=offset.tolist(),profile=dict(range_m=5,blind_m=.5,map_occupancy_voxel_m=a.voxel,azimuth_step_deg=a.az_step,elevation_step_deg=a.el_step,vertical_FOV_deg=[-7,52],horizontal_FOV_deg=[0,360],beam_pattern='uniform angular experimental rays, NOT exact Livox nonrepetitive beam or motion simulation'),all_stations_review_pending=True,online_enabled=False,go2_mounting_not_calibrated=True)
 result['mounting_profile_source']='explicit base-frame up direction' if a.up_base is not None else 'median historical handheld up-base; current physical mounting still to verify'
 save(out/'sampling_summary.json',result);print(json.dumps(result,ensure_ascii=False),flush=True)
 if a.phase=='stations':return
 binary=out/'virtual_scan_renderer';subprocess.run(['g++','-O3','-std=c++17',str(ROOT/'scripts/virtual_scan_renderer.cpp'),'-o',str(binary)],check=True)
 subprocess.run([str(binary),str(out/'render_map.bin'),str(out/'render_poses.txt'),str(out/'rendered.bin'),str(a.voxel),'5','.5',str(a.az_step),str(a.el_step),str(out/'lidar_offset.txt')],check=True)
 (out/'clouds').mkdir(exist_ok=True);descriptors=[];index=[];skipped=[];start=time.perf_counter()
 with (out/'rendered.bin').open('rb') as f:
  for row in profiles:
   raw=f.read(4)
   if len(raw)!=4:raise ValueError('Missing renderer output')
   n=int(np.frombuffer(raw,dtype='<u4')[0]);points=np.frombuffer(f.read(n*12),dtype='<f4').reshape(-1,3).copy()
   try:encoded=encode(points,row['up_base'],offset)
   except ValueError as e:skipped.append(dict(id=row['id'],reason=str(e),raw_points=n));continue
   rel=f"clouds/{row['id']}.pcd";write_pcd(out/rel,points);mb=np.array(row['map_T_base']);row.update(pcd=rel,pcd_sha256=sha256(out/rel),base_T_descriptor=encoded['base_T_descriptor'].tolist(),map_T_descriptor=(mb@encoded['base_T_descriptor']).tolist(),point_count=n,canonical_points=len(encoded['points']),provisional_virtual_reference=True)
   index.append(row);descriptors.append(encoded['descriptor'])
  if f.read(1):raise ValueError('Unexpected trailing output')
 if not descriptors:raise ValueError('No usable virtual descriptors')
 np.savez_compressed(out/'descriptors.npz',ids=np.array([x['id'] for x in index]),descriptors=np.stack(descriptors));(out/'index.jsonl').write_text(''.join(json.dumps(x,ensure_ascii=False)+'\n' for x in index));save(out/'descriptor_profile.json',DEFAULT_CONFIG)
 result.update(usable_virtual_templates=len(index),skipped=skipped,descriptor_encode_seconds=time.perf_counter()-start,minimum_points=min(x['canonical_points'] for x in index));save(out/'build_summary.json',result);print('TEMPLATES_READY',len(index),flush=True)

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--map',type=Path,default=ROOT.parent/'kiss_localization_ws1.1/src/localization/maps/building_split_space.pcd');p.add_argument('--map-cache',type=Path,required=True);p.add_argument('--reference-library',type=Path,default=ROOT/'reference_maps/keyframe_libraries/space_2/merged_20261111');p.add_argument('--output',type=Path,required=True);p.add_argument('--region',type=int,default=2);p.add_argument('--heights',type=float,nargs='+',default=[1.2]);p.add_argument('--spacing',type=float,default=.5);p.add_argument('--yaw-step',type=float,default=45);p.add_argument('--voxel',type=float,default=.1);p.add_argument('--az-step',type=float,default=1.);p.add_argument('--el-step',type=float,default=1.5);p.add_argument('--support-z-min',type=float);p.add_argument('--support-z-max',type=float);p.add_argument('--phase',choices=['stations','all'],default='all');p.add_argument('--up-base',type=float,nargs=3,help='Measured gravity-up in current IMU/base frame; not a LiDAR-IMU extrinsic change');a=p.parse_args()
 if (a.support_z_min is None)!=(a.support_z_max is None) or (a.support_z_min is not None and (not np.isfinite([a.support_z_min,a.support_z_max]).all() or a.support_z_min>a.support_z_max)):p.error('Finite paired support height bounds required')
 values=a.heights+[a.spacing,a.yaw_step,a.voxel,a.az_step,a.el_step]
 if any(not np.isfinite(x) or x<=0 for x in values):p.error('Finite positive sampling parameters required')
 if a.output.exists() and any(a.output.iterdir()):p.error('Use a new empty output directory; do not overwrite prior experiments')
 generate(a)
if __name__=='__main__':main()
