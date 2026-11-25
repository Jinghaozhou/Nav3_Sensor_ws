#!/usr/bin/python3
"""从生产分区缓存生成三维区域索引；不使用包围盒决定归属。"""
import argparse,json
from pathlib import Path
import numpy as np
from build_complete_reference import read_pcd
from build_keyframe_library import sha256
from workspace_paths import map_file
from region_profiles import ROOT

def main():
 p=argparse.ArgumentParser();p.add_argument('--cache',type=Path,required=True);p.add_argument('--output',type=Path,default=ROOT/'reference_maps/region_tracking');a=p.parse_args()
 if a.output.exists():raise ValueError('索引目录已存在，不覆盖')
 points=[];labels=[];counts={}
 for i in range(1,14):
  raw,_=read_pcd(a.cache/f'region_{i}.pcd');xyz=np.column_stack([raw[k] for k in ('x','y','z')]);xyz=xyz[np.isfinite(xyz).all(1)];_,idx=np.unique(np.floor(xyz/.2).astype('i8'),axis=0,return_index=True);xyz=xyz[idx];points.append(xyz);labels.append(np.full(len(xyz),i,dtype='u1'));counts[str(i)]=len(xyz)
 a.output.mkdir(parents=True);np.savez_compressed(a.output/'regions.npz',xyz=np.concatenate(points),labels=np.concatenate(labels))
 m=dict(schema_version=1,map_sha256=sha256(map_file()),index_sha256=sha256(a.output/'regions.npz'),region_points=counts,voxel_m=.2,field_validated=False,method='3d_inverse_distance_label_vote_not_occupancy_truth')
 (a.output/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n');print(json.dumps(m,ensure_ascii=False))
if __name__=='__main__':main()
