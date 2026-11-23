"""Experimental virtual-template shortlist; never masquerades as measured references."""
from pathlib import Path
import json,time
import numpy as np
from scipy.spatial.transform import Rotation
from scan_context_descriptor import encode,config_hash,DEFAULT_CONFIG
from build_keyframe_library import sha256
from keyframe_database import rigid
ROOT=Path(__file__).resolve().parents[1]
DEFAULT_VIRTUAL_CONFIG=ROOT/'config/virtual_initialization_space_10.json'
class VirtualDatabase:
 def __init__(self,map_path,config_path=DEFAULT_VIRTUAL_CONFIG):
  c=json.loads(Path(config_path).read_text());self.search_config=c
  if c['schema_version']!=1 or c['region_id']!=10 or not 1<=c['maximum_candidates']<=64:raise ValueError('Unsupported virtual region/budget')
  self.path=ROOT/c['library'];m=json.loads((self.path/'manifest.json').read_text());self.manifest=m
  self.map_sha=sha256(map_path);self.library_sha=sha256(self.path/'manifest.json')
  if m['map_sha256']!=self.map_sha or m['config_sha256']!=sha256(config_path) or m.get('source')!='virtual_map_raycasts_not_measured_keyframes':raise ValueError('Virtual model/map/config mismatch')
  for name,digest in m['files'].items():
   path=self.path/name
   if not path.resolve().is_relative_to(self.path.resolve()) or sha256(path)!=digest:raise ValueError('Virtual artifact changed: '+name)
  self.config=json.loads((self.path/'descriptor_profile.json').read_text())
  if self.config!=DEFAULT_CONFIG:raise ValueError('Virtual descriptor profile changed')
  self.profile_sha=config_hash(dict(descriptor=self.config,retrieval=c));self.region_id=10
  self.templates=[json.loads(x) for x in (self.path/'index.jsonl').read_text().splitlines()]
  z=np.load(self.path/'descriptors.npz',allow_pickle=False);self.descriptors=z['descriptors'].copy()
  if len(set(x['id'] for x in self.templates))!=len(self.templates) or list(z['ids'])!=[x['id'] for x in self.templates] or self.descriptors.shape!=(len(self.templates),20,60) or not np.isfinite(self.descriptors).all():raise ValueError('Invalid virtual descriptor arrays')
  for x in self.templates:
   if x['region_id']!=10 or not rigid(x['map_T_base']) or not rigid(x['map_T_descriptor']) or not rigid(x['base_T_descriptor']) or not np.allclose(np.array(x['map_T_base'])@np.array(x['base_T_descriptor']),x['map_T_descriptor'],atol=1e-6):raise ValueError('Virtual pose mismatch')
  self.origin=np.array(m['sensor_offset_base']);self.index=list(range(c['maximum_candidates']))
  self.norm=np.linalg.norm(self.descriptors,axis=1);self.packet_reason='virtual_shortlist';self.last_metrics={}
 def retrieve(self,points_base,up_base,odom_T_base):
  if not rigid(odom_T_base):raise ValueError('Invalid virtual query odom')
  start=time.perf_counter();q=encode(points_base,up_base,self.origin);enc=(time.perf_counter()-start)*1000;start=time.perf_counter()
  best=np.full(len(self.templates),np.inf);shifts=np.zeros(len(best),int)
  for shift in range(60):
   v=np.roll(q['descriptor'],shift,axis=1);vn=np.linalg.norm(v,axis=0);ok=(self.norm>1e-12)&(vn[None,:]>1e-12)
   dots=np.einsum('nrs,rs->ns',self.descriptors,v);sim=np.divide(dots,self.norm*vn[None,:],out=np.zeros_like(dots),where=ok);count=ok.sum(1)
   scores=np.where(count>0,np.maximum(0,1-sim.sum(1)/np.maximum(count,1)),np.inf);update=scores<best;best[update]=scores[update];shifts[update]=shift
  selected=[];dedup=0;overflow=False
  for i in np.argsort(best,kind='stable'):
   if not np.isfinite(best[i]):continue
   rz=np.eye(4);rz[:3,:3]=Rotation.from_euler('z',int(shifts[i])*6,degrees=True).as_matrix()
   mb=np.array(self.templates[i]['map_T_descriptor'])@rz@np.linalg.inv(q['base_T_descriptor'])
   if any(np.linalg.norm(mb[:3,3]-x['map_T_base'][:3,3])<self.search_config['dedup_translation_m'] and np.rad2deg(Rotation.from_matrix(mb[:3,:3].T@x['map_T_base'][:3,:3]).magnitude())<self.search_config['dedup_rotation_deg'] for x in selected):dedup+=1;continue
   if len(selected)>=len(self.index):overflow=True;break
   selected.append(dict(id=self.templates[i]['id'],distance=float(best[i]),map_T_base=mb,map_T_odom=mb@np.linalg.inv(odom_T_base)))
  if not selected:raise ValueError('No finite virtual descriptor candidates')
  self.last_metrics=dict(templates=len(self.templates),selected=len(selected),deduplicated=dedup,shortlist_truncated=overflow,policy='descriptor_top24_distinct_poses_not_global_competitor_completeness')
  return selected,enc,(time.perf_counter()-start)*1000
