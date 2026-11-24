#!/usr/bin/python3
"""离线生成13区候选资料，保留既有模型；失败区域不伪造就绪状态。"""
import argparse,json,subprocess,sys,time
from pathlib import Path
from build_keyframe_library import sha256
from workspace_paths import map_file
from region_profiles import ROOT,virtual_config_for_region
from bbs_database import bbs_config_for_region

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--map-cache',type=Path,required=True);p.add_argument('--report',type=Path,required=True);a=p.parse_args()
 a.report.parent.mkdir(parents=True,exist_ok=True);report={'map_sha256':sha256(map_file()),'regions':[],'field_validated':False}
 for i in range(1,14):
  row={'region':i};report['regions'].append(row)
  for kind,config in [('bbs',bbs_config_for_region(i)),('virtual',virtual_config_for_region(i))]:
   c=json.loads(config.read_text());out=ROOT/c['model' if kind=='bbs' else 'library'];begin=time.monotonic()
   if (out/'manifest.json').exists():row[kind]={'status':'preserved','path':str(out)};continue
   try:
    if out.exists():raise RuntimeError('目标目录未封口，需人工检查后重建，不覆盖已有资料：'+str(out))
    if kind=='bbs':
     cmd=[sys.executable,str(ROOT/'scripts/prepare_bbs_model.py'),'--config',str(config),'--output',str(out)]
    else:
     cmd=[sys.executable,str(ROOT/'scripts/build_virtual_map_library.py'),'--map-cache',str(a.map_cache),'--region',str(i),'--output',str(out),'--heights','.5','--spacing','1','--up-base','-0.3420201433','0','0.9396926208']
    with (a.report.parent/f'{kind}_space_{i}.log').open('w') as log:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
    if kind=='virtual':
     summary=json.loads((out/'build_summary.json').read_text());files={str(q.relative_to(out)):sha256(q) for q in sorted(out.iterdir()) if q.name in ['index.jsonl','descriptors.npz','descriptor_profile.json','sampling_summary.json','stations.jsonl','build_summary.json']}
     files.update({str(q.relative_to(out)):sha256(q) for q in sorted((out/'clouds').glob('*.pcd'))})
     m=dict(schema_version=1,source='virtual_map_raycasts_not_measured_keyframes',region_id=i,map_sha256=report['map_sha256'],config_sha256=sha256(config),sensor_offset_base=summary['sensor_offset_base'],stations=summary['stations'],templates=summary['usable_virtual_templates'],files=files,field_validated=False,free_space_certified=False,reported_height_m=.5,reported_pitch_down_deg=20,support_selection='automatic_geometry_no_manual_floor_certification',generator_sha256=sha256(ROOT/'scripts/build_virtual_map_library.py'))
     (out/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n')
    m=json.loads((out/'manifest.json').read_text());row[kind]=dict(status='built',path=str(out),seconds=round(time.monotonic()-begin,2),stations=m.get('stations'),templates=m.get('templates'),target_points=m.get('target_points'))
   except Exception as e:row[kind]={'status':'failed','reason':str(e)}
   a.report.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(i,kind,row[kind],flush=True)
 report['complete']=all(all(x.get(k,{}).get('status') in ('built','preserved') for k in ('bbs','virtual')) for x in report['regions']);a.report.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
 return 0 if report['complete'] else 1
if __name__=='__main__':raise SystemExit(main())
