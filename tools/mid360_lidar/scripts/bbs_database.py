"""Map-only live candidate source, with bounded competing-peak search."""
import csv
import json
from pathlib import Path
import selectors
import subprocess
import tempfile
import time
import numpy as np
from build_keyframe_library import sha256
from keyframe_database import rigid
from scan_context_descriptor import DEFAULT_CONFIG, encode

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BBS_CONFIG = ROOT/'config/bbs_initialization.json'

def bbs_config_for_region(region):
    region = str(region)
    profiles = {'2': DEFAULT_BBS_CONFIG, 'space_2': DEFAULT_BBS_CONFIG,
                '10': ROOT/'config/bbs_initialization_space_10.json',
                'space_10': ROOT/'config/bbs_initialization_space_10.json'}
    if region not in profiles:
        raise ValueError('BBS model not configured for region: ' + region)
    return profiles[region]

class BbsDatabase:
    def __init__(self, map_path, config_path=DEFAULT_BBS_CONFIG, output=None):
        self.search_config=json.loads(Path(config_path).read_text()); c=self.search_config
        if c['schema_version']!=1 or c['region_id'] not in (2,10):
            raise ValueError('Only configured experimental space_2/space_10 BBS profiles are supported')
        for k in ['timeout_ms','threads','levels','maximum_candidates']:
            if not isinstance(c[k],int) or c[k]<1: raise ValueError('Invalid BBS '+k)
        if not 1<=c['maximum_candidates']<=64 or not 0<c['competition_score_gap']<1 or not 0<c['minimum_occupancy_fraction']<1:
            raise ValueError('Invalid BBS search limits')
        self.path=ROOT/c['model']; self.manifest=json.loads((self.path/'manifest.json').read_text())
        self.library_sha=sha256(self.path/'manifest.json'); self.profile_sha=sha256(config_path)
        self.map_sha=sha256(map_path)
        m=self.manifest
        if m['region_id']!=c['region_id'] or m['map_sha256']!=self.map_sha or m['config_sha256']!=self.profile_sha:
            raise ValueError('BBS model/map/config mismatch; rebuild model')
        for file,key in [('target.bin','target_sha256'),('worker.cfg','worker_config_sha256')]:
            if sha256(self.path/file)!=m[key]: raise ValueError('BBS model changed: '+file)
        self.region_id=c['region_id']; self.origin=np.array(m['lidar_origin_base']); self.config=DEFAULT_CONFIG
        self.basis=np.asarray(m['basis_map_from_gravity']); t=np.eye(4); t[:3,:3]=self.basis
        if not rigid(t): raise ValueError('Invalid BBS gravity basis')
        self.frame=t; self.index=list(range(c['maximum_candidates']))
        binary=ROOT/'build/bbs/bbs_worker'
        for source,digest in json.loads((binary.parent/'source_sha256.json').read_text()).items():
            if sha256(ROOT/source)!=digest: raise ValueError('BBS sources changed; rebuild worker')
        self.worker=subprocess.Popen([str(binary),str(self.path/'target.bin'),str(self.path/'worker.cfg')],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
        self.temp=tempfile.TemporaryDirectory(prefix='mid360_bbs_query_')
        self.selector=selectors.DefaultSelector(); self.selector.register(self.worker.stdout,selectors.EVENT_READ)
        self.file=None; self.count=0; self.last_metrics={}
        try:
            if self.readline(10).strip()!='READY': raise ValueError('BBS worker startup failed')
            if output:
                output=Path(output); output.mkdir(parents=True,exist_ok=True)
                self.file=(output/'bbs_queries.csv').open('x',newline=''); self.writer=csv.DictWriter(self.file,
                    fieldnames=['query','cloud_ns','start_ns','search_ms','total_ms','points','peaks','complete','timed_out','peak_limit','scores'])
                self.writer.writeheader()
        except BaseException:
            self.close(); raise

    def readline(self, timeout):
        if not self.selector.select(timeout): raise TimeoutError('BBS worker response timeout')
        line=self.worker.stdout.readline()
        if not line: raise RuntimeError('BBS worker exited')
        return line

    def retrieve(self, points_base, up_base, odom_T_base):
        if not rigid(odom_T_base): raise ValueError('Invalid BBS query odom pose')
        begin=time.perf_counter(); q=encode(points_base,up_base,self.origin)
        encoding_ms=(time.perf_counter()-begin)*1000
        _,indices=np.unique(np.floor(q['points']/self.search_config['voxel_m']).astype('i8'),axis=0,return_index=True)
        points=q['points'][indices]; path=Path(self.temp.name)/'query.bin'; points.astype('<f4').tofile(path)
        self.worker.stdin.write(str(path)+'\n'); self.worker.stdin.flush()
        result=json.loads(self.readline(self.search_config['timeout_ms']/1000+2))
        self.count+=1
        total=(time.perf_counter()-begin)*1000
        self.last_metrics=dict(query=self.count,cloud_ns=getattr(self,'query_cloud_ns',0),start_ns=getattr(self,'query_start_ns',0),search_ms=result['ms'],total_ms=total,
            points=len(points),peaks=len(result['peaks']),complete=result['complete'],
            timed_out=result['timed_out'],peak_limit=result['peak_limit'],
            scores=json.dumps([x['score'] for x in result['peaks']]))
        if self.file: self.writer.writerow(self.last_metrics); self.file.flush()
        if not result['complete'] or result['timed_out'] or result['peak_limit']:
            raise ValueError('bbs_competitor_search_incomplete')
        if not result['peaks']: raise ValueError('bbs_no_occupancy_candidate')
        hypotheses=[]
        for i,peak in enumerate(result['peaks']):
            mb=self.frame @ np.array(peak['pose']) @ np.linalg.inv(q['base_T_descriptor'])
            if not rigid(mb): raise ValueError('Invalid BBS pose')
            hypotheses.append(dict(id=f'bbs_peak_{i:02d}',distance=1-peak['score'],
                map_T_base=mb,map_T_odom=mb@np.linalg.inv(odom_T_base)))
        return hypotheses,encoding_ms,total-encoding_ms

    def close(self):
        if self.file: self.file.close(); self.file=None
        if getattr(self,'worker',None):
            if self.worker.stdin:
                try: self.worker.stdin.close()
                except (BrokenPipeError, OSError): pass
            try:self.worker.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.worker.terminate(); self.worker.wait(timeout=5)
            self.worker.stdout.close(); self.worker=None
        if getattr(self,'selector',None): self.selector.close()
        if getattr(self,'temp',None): self.temp.cleanup()
