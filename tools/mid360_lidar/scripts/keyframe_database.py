"""Validated, read-only experimental keyframe database and candidate construction."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
from scan_context_descriptor import encode,circular_distance,config_hash,DEFAULT_CONFIG,load_xyz
from build_keyframe_library import sha256


def default_library():
    """One shared reference selection for preflight, launch and retriever."""
    root = Path(__file__).resolve().parents[1]
    selection = json.loads((root/'config/keyframe_library.json').read_text())
    if selection.get('schema_version') != 1 or selection.get('region') != 'space_2':
        raise ValueError('Unsupported active reference-library selection')
    path = Path(selection['library'])
    return (path if path.is_absolute() else root/path).resolve()


def rigid(t):
    t=np.asarray(t,float)
    return t.shape==(4,4) and np.isfinite(t).all() and np.allclose(t[3],[0,0,0,1]) and np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-5) and np.linalg.det(t[:3,:3])>.999


class KeyframeDatabase:
    def __init__(self,library,map_path):
        self.path=Path(library).resolve();m=json.loads((self.path/'manifest.json').read_text());self.manifest=m
        self.map_sha=sha256(Path(map_path));self.library_sha=sha256(self.path/'keyframes.jsonl')
        if self.map_sha!=m['map_sha256'] or self.library_sha!=m['index_sha256']:
            raise ValueError('Keyframe map/index SHA mismatch')
        if (not m.get('operator_session_pose_floor_confirmed') and m.get('automatic_admission_policy')!='space2_auto_v1') or m.get('state')!='geometry_checked_descriptors_ready':
            raise ValueError('Need geometry checked, operator route confirmed reference library')
        self.region_id=int(m['region_claim'].removeprefix('space_'))
        d=self.path/m['descriptor_directory'];self.config=json.loads((d/'profile.json').read_text());self.profile_sha=config_hash(self.config)
        if self.profile_sha!=m['descriptor_profile_sha256'] or self.config!=DEFAULT_CONFIG:
            raise ValueError('Descriptor profile changed or unsupported')
        self.index=[json.loads(x) for x in (d/'index.jsonl').read_text().splitlines()]
        original=[json.loads(x) for x in (self.path/'keyframes.jsonl').read_text().splitlines()]
        summary=json.loads((d/'summary.json').read_text());npz=np.load(d/'descriptors.npz',allow_pickle=False)
        self.descriptors=npz['descriptors'].copy()
        if not 1<=len(original)<=64 or len(original)!=len(self.index) or summary['geometry_passed']!=len(original):raise ValueError('Invalid reference count/geometry')
        if list(npz['ids'])!=[x['id'] for x in original] or str(npz['profile_sha256'])!=self.profile_sha or self.descriptors.shape!=(len(original),20,60) or not np.isfinite(self.descriptors).all():raise ValueError('Invalid descriptor arrays')
        for n,(r,i) in enumerate(zip(original,self.index)):
            if r['id']!=i['id'] or sha256(self.path/r['pcd'])!=r['pcd_sha256'] or sha256(d/i['canonical_pcd'])!=i['canonical_sha256']:raise ValueError('Reference cloud changed')
            if not all(rigid(x) for x in [i['base_T_descriptor'],i['map_T_descriptor'],r['map_T_base']]) or not np.allclose(np.array(r['map_T_base'])@np.array(i['base_T_descriptor']),i['map_T_descriptor'],atol=1e-6):raise ValueError('Reference pose mismatch')
            encoded=encode(load_xyz(self.path/r['pcd']),i['up_base'],r['lidar_origin_base'],self.config)
            if not np.array_equal(encoded['descriptor'],self.descriptors[n]):
                raise ValueError('Reference descriptor content changed')
            if not np.allclose(encoded['base_T_descriptor'],i['base_T_descriptor'],atol=1e-6):raise ValueError('Descriptor basis changed')
        if m.get('automatic_admission_policy')=='space2_auto_v1':
            manual=[x for x in original if not x.get('auto_admission')]
            baseline=None
            if manual:
                baseline=KeyframeDatabase(Path(__file__).resolve().parents[1]/'reference_maps/keyframe_libraries/space_2/merged_20261111',map_path)
                if len(manual)!=40:raise ValueError('Automatic bank must preserve all forty frozen references')
                frozen=[json.loads(x) for x in (baseline.path/'keyframes.jsonl').read_text().splitlines()]
                if any(a['pcd_sha256']!=b['pcd_sha256'] or not np.allclose(a['map_T_base'],b['map_T_base']) for a,b in zip(manual,frozen)):
                    raise ValueError('Frozen manual baseline changed')
            for row in original:
                if 'auto_admission' not in row:continue
                a=row['auto_admission']
                if (a.get('policy')!='space2_auto_v1' or not a.get('three_recent_applied') or
                    a.get('operator_pose_floor_confirmed') is not False or a['geometry_support']<.95 or
                    a['geometry_rmse_m']>.10 or a['region_vote']<.9 or a['region_support']<.9 or
                    row['applied_overlap_percent']<85 or row['applied_age_seconds']>2.5):
                    raise ValueError('Automatic reference admission evidence failed')
        offsets=np.array([x['lidar_origin_base'] for x in original]);self.origin=offsets[0]
        if not np.allclose(offsets,self.origin):raise ValueError('Reference sensor offsets vary')

    def retrieve(self,points_base,up_base,odom_T_base):
        if not rigid(odom_T_base):raise ValueError('Invalid query odom pose')
        import time
        t=time.perf_counter();q=encode(points_base,up_base,self.origin,self.config);encoding_ms=(time.perf_counter()-t)*1000
        t=time.perf_counter();scores=[circular_distance(r,q['descriptor']) for r in self.descriptors];order=sorted(range(len(scores)),key=lambda i:scores[i][0]);result=[]
        for i in order:
            score,shift=scores[i]
            if not np.isfinite(score):raise ValueError('No usable angular descriptor overlap')
            rz=np.eye(4);rz[:3,:3]=Rotation.from_euler('z',shift*6,degrees=True).as_matrix()
            mb=np.array(self.index[i]['map_T_descriptor'])@rz@np.linalg.inv(q['base_T_descriptor'])
            result.append(dict(id=self.index[i]['id'],distance=score,shift_sectors=shift,map_T_base=mb,map_T_odom=mb@np.linalg.inv(odom_T_base)))
        return result,encoding_ms,(time.perf_counter()-t)*1000


def validate_sensor_config(path,database):
    import yaml
    cfg=yaml.safe_load(Path(path).read_text())['/**']['ros__parameters']
    mapping=cfg['mapping'];pre=cfg['preprocess']
    if mapping.get('extrinsic_est_en') or not np.allclose(mapping['extrinsic_T'],database.origin) or not np.allclose(np.array(mapping['extrinsic_R']).reshape(3,3),np.eye(3)):
        raise ValueError('Keyframe trial requires the library fixed LiDAR/IMU extrinsics')
    if not np.isclose(mapping['det_range'],database.config['radius_m']) or not np.isclose(pre['blind'],database.config['blind_m']):
        raise ValueError('Keyframe trial requires matching 5m range / 0.5m blind profile')
