"""只读区域识别：三维标签近邻、歧义拒绝、按不同位姿时间戳防抖。"""
import numpy as np
from scipy.spatial import cKDTree

class RegionClassifier:
 def __init__(self,xyz,labels,k=96,radius=2.,min_vote=.7,min_gap=.2):
  xyz=np.asarray(xyz,float);labels=np.asarray(labels,int)
  if xyz.ndim!=2 or xyz.shape[1]!=3 or len(xyz)!=len(labels) or not len(xyz) or not np.isfinite(xyz).all() or not np.isin(labels,np.arange(1,14)).all():raise ValueError('区域索引无效')
  self.tree=cKDTree(xyz);self.labels=labels;self.k=min(k,len(labels));self.radius=radius;self.min_vote=min_vote;self.min_gap=min_gap
 def classify(self,position):
  p=np.asarray(position,float)
  if p.shape!=(3,) or not np.isfinite(p).all():return 0,0.,'invalid_pose'
  distance,index=self.tree.query(p,k=self.k,distance_upper_bound=self.radius)
  distance=np.atleast_1d(distance);index=np.atleast_1d(index);ok=np.isfinite(distance)
  if ok.sum()<8:return 0,0.,'outside_labeled_map'
  weights=1/np.maximum(distance[ok],.15)**2
  votes=np.bincount(self.labels[index[ok]],weights=weights,minlength=14)[1:];order=np.argsort(votes);best=order[-1];fraction=votes[best]/votes.sum();gap=(votes[best]-votes[order[-2]])/votes.sum()
  if fraction<self.min_vote or gap<self.min_gap:return 0,float(fraction),'ambiguous_boundary'
  return int(best+1),float(fraction),'geometry_vote'

class RegionState:
 def __init__(self,dwell=2.,min_samples=3):
  self.dwell=dwell;self.min_samples=min_samples;self.current=0;self.last_known=0;self.pending=0;self.start=0.;self.samples=0;self.last_stamp=None
 def clear_pending(self):self.pending=0;self.samples=0
 def update(self,region,stamp,valid=True):
  if self.last_stamp is not None and stamp<self.last_stamp:
   self.current=0;self.last_known=0;self.clear_pending()
  if not valid or region==0:
   self.current=0;self.clear_pending();return None
  if stamp==self.last_stamp:return None
  self.last_stamp=stamp
  if self.current==region:self.clear_pending();return None
  if self.pending!=region:self.pending=region;self.start=stamp;self.samples=1
  else:self.samples+=1
  if self.samples<self.min_samples or stamp-self.start<self.dwell:return None
  old=self.last_known;self.current=region;self.last_known=region;self.clear_pending()
  return dict(kind='initial' if old==0 else ('recovered' if old==region else 'transition'),from_region=old,to_region=region,stamp=stamp)
