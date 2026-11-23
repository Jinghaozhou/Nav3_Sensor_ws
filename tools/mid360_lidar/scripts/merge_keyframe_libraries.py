#!/usr/bin/python3
"""Merge reviewed same-map references without modifying either source library.

This creates experimental reference data, not independent field validation.
Duplicate means nearby pose AND similar orientation AND similar descriptor.
"""
import argparse
import copy
import csv
from datetime import datetime
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial.transform import Rotation

from build_keyframe_library import sha256
from keyframe_database import KeyframeDatabase
from scan_context_descriptor import circular_distance


def merge(base, addition, map_path, output, automatic=False):
    base, addition, output = Path(base).resolve(), Path(addition).resolve(), Path(output).absolute()
    databases = [KeyframeDatabase(p, map_path) for p in (base, addition)]
    if databases[0].region_id != databases[1].region_id or databases[0].profile_sha != databases[1].profile_sha:
        raise ValueError('Map, region and descriptor profile must agree')
    if not np.allclose(databases[0].origin, databases[1].origin):
        raise ValueError('Sensor origin changed')
    selected, report = [], []
    for source_no, (path, db) in enumerate(zip((base, addition), databases)):
        originals = [json.loads(x) for x in (path/'keyframes.jsonl').read_text().splitlines()]
        for i, row in enumerate(originals):
            if not db.index[i]['geometry_check_passed']:
                raise ValueError('Unreviewed geometry')
            duplicate = None
            if source_no:
                for previous in selected:
                    a, b = np.array(row['map_T_base']), np.array(previous['row']['map_T_base'])
                    distance = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
                    angle = float(np.rad2deg(Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).magnitude()))
                    if distance <= .35 and angle <= 15:
                        score, _ = circular_distance(previous['descriptor'], db.descriptors[i])
                        if score <= .05:
                            duplicate = dict(kept_id=previous['id'], distance_m=distance,
                                             rotation_deg=angle, descriptor_distance=float(score))
                            break
            report.append(dict(source_library=str(path), source_id=row['id'],
                               included=duplicate is None, duplicate=duplicate))
            if duplicate is not None:
                continue
            ident = row['id'] if not source_no else f'kf_{len(selected):05d}'
            selected.append(dict(id=ident, row=row, index=db.index[i], descriptor=db.descriptors[i],
                                 source=path, database=db))
    if automatic:
        frozen=[x for x in selected if not x['row'].get('auto_admission')]
        pool=[x for x in selected if x['row'].get('auto_admission')]
        if len(frozen)!=40:raise ValueError('Automatic bank needs the original forty frozen references')
        # Bounded online bank: preserve forty originals, select 24 diverse automatic views.
        chosen=[]
        while pool and len(chosen)<24:
            def gain(item):
                t=np.array(item['row']['map_T_base']);anchors=chosen if chosen else frozen
                return min(np.linalg.norm(t[:3,3]-np.array(v['row']['map_T_base'])[:3,3])+
                    .5*Rotation.from_matrix(t[:3,:3].T@np.array(v['row']['map_T_base'])[:3,:3]).magnitude() for v in anchors)
            best=max(range(len(pool)),key=lambda i:(gain(pool[i]),pool[i]['row']['stamp_ns']));chosen.append(pool.pop(best))
        selected=frozen+chosen
        for n,item in enumerate(selected):item['id']=f'kf_{n:05d}'
    if not automatic and not len(databases[0].index) < len(selected) <= 64:
        raise ValueError('Need genuinely new frames and at most 64; never silently truncate')
    output.mkdir(parents=True, exist_ok=False)
    (output/'clouds').mkdir()
    d = output/'descriptors_v1'; (d/'canonical_clouds').mkdir(parents=True)
    rows, index, provenance = [], [], []
    for item in selected:
        path, db, ident = item['source'], item['database'], item['id']
        row, entry = copy.deepcopy(item['row']), copy.deepcopy(item['index'])
        original_id = row['id']
        source_session = row.get('source_session') or db.manifest.get('source_session')
        if not source_session:
            raise ValueError('Reference frame has no source-session provenance')
        new_pcd, new_canonical = f'clouds/{ident}.pcd', f'canonical_clouds/{ident}.pcd'
        shutil.copy2(path/row['pcd'], output/new_pcd)
        shutil.copy2(path/db.manifest['descriptor_directory']/entry['canonical_pcd'], d/new_canonical)
        row.update(id=ident, pcd=new_pcd, source_library=str(path), source_id=original_id,
                   source_session=source_session)
        entry.update(id=ident, source_pcd=new_pcd, canonical_pcd=new_canonical)
        rows.append(row); index.append(entry)
        provenance.append(dict(id=ident, source_library=str(path), source_id=original_id,
                               source_session=source_session, stamp_ns=row['stamp_ns']))
    # Preserve descriptor values and source pose labels bit for bit.
    desc = np.stack([x['descriptor'] for x in selected])
    np.savez_compressed(d/'descriptors.npz', ids=np.array([x['id'] for x in rows]),
                        descriptors=desc, occupied=desc != 0, ring_keys=desc.mean(axis=2),
                        sector_keys=desc.mean(axis=1), profile_sha256=np.array(databases[0].profile_sha))
    (output/'keyframes.jsonl').write_text(''.join(json.dumps(x, ensure_ascii=False)+'\n' for x in rows))
    (d/'index.jsonl').write_text(''.join(json.dumps(x, ensure_ascii=False)+'\n' for x in index))
    shutil.copy2(base/databases[0].manifest['descriptor_directory']/'profile.json', d/'profile.json')
    audits = []
    for item in selected:
        p = item['source']/item['database'].manifest['descriptor_directory']/'geometry_review.csv'
        with p.open() as f:
            a = next(x for x in csv.DictReader(f) if x['id'] == item['row']['id'])
        a['id'] = item['id']; audits.append(a)
    with (d/'geometry_review.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(audits[0])); writer.writeheader(); writer.writerows(audits)
    matrix = np.array([[circular_distance(a, b)[0] for b in desc] for a in desc])
    np.save(d/'pairwise_distance.npy', matrix, allow_pickle=False)
    summary = dict(state='geometry_checked_descriptors_ready', keyframes=len(rows), geometry_passed=len(rows),
                   map_sha256=databases[0].map_sha, profile_sha256=databases[0].profile_sha,
                   source_index_sha256=sha256(output/'keyframes.jsonl'),
                   min_map_30cm_fraction=min(float(x['nearest_map_30cm_fraction']) for x in audits),
                   max_map_inlier_rmse_m=max(float(x['nearest_map_inlier_rmse_m']) for x in audits),
                   online_enabled=False, independent_retrieval_test_passed=False,
                   note='Training-source runs are regression data after merging, not independent accuracy tests.')
    manifest = copy.deepcopy(databases[0].manifest)
    manifest.update(created=datetime.now().astimezone().isoformat(), keyframes=len(rows),
                    source_sessions=sorted({x['source_session'] for x in provenance}),
                    source_libraries=[dict(path=str(p), index_sha256=db.library_sha,
                         manifest_sha256=sha256(p/'manifest.json')) for p, db in zip((base, addition), databases)],
                    index_sha256=summary['source_index_sha256'], state=summary['state'],
                    descriptor_directory='descriptors_v1', descriptor_built=True, online_enabled=False,
                    location_note='space_2：原11月9日28帧与11月10日经筛选的新视角合并；位置非测量真值',
                    provenance_file='frame_provenance.json', region_selection_file='region_selection.json')
    if automatic:
        manifest.update(operator_session_pose_floor_confirmed=False,automatic_admission_policy='space2_auto_v1',
            location_note='forty frozen manually confirmed references plus automatically checked pseudo-labels; new floors not operator-confirmed',
            pose_status='mixed_manual_and_automatic_not_ground_truth')
    # Single-run parameters and origin must not describe a composite as though
    # all frames were acquired in the original run. See source manifests instead.
    for key in ('source_session', 'source_metadata_sha256', 'source_config_sha256',
                'cloud_time_origin_ns', 'parameters', 'skipped'):
        manifest.pop(key, None)
    selections = []
    for p, db in zip((base, addition), databases):
        selections.append(dict(source_library=str(p), selection=json.loads((p/db.manifest['region_selection_file']).read_text())))
    (output/'region_selection.json').write_text(json.dumps(dict(sources=selections), ensure_ascii=False, indent=2)+'\n')
    manifest['region_selection_sha256'] = sha256(output/'region_selection.json')
    (output/'frame_provenance.json').write_text(json.dumps(provenance, ensure_ascii=False, indent=2)+'\n')
    (d/'route_annotations.json').write_text(json.dumps(dict(source_index_sha256=manifest['index_sha256'],
        frames={r['id']: r['route'] for r in index}), ensure_ascii=False, indent=2)+'\n')
    for path, value in [(d/'summary.json', summary), (output/'manifest.json', manifest),
                        (output/'merge_review.json', dict(rules=dict(max_position_m=.35,
                         max_rotation_deg=15, max_descriptor_distance=.05), frames=report,
                         base_count=len(databases[0].index), addition_count=len(databases[1].index),
                         merged_count=len(rows), retained_new=len(rows)-len(databases[0].index),
                         selected_addition=sum(x['source']==addition for x in selected)))]:
        path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    KeyframeDatabase(output, map_path)  # Content hashes, re-encoding and pose/profile agreement.
    return json.loads((output/'merge_review.json').read_text())


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for key in ('base', 'addition', 'map', 'output'):
        p.add_argument('--'+key, type=Path, required=True)
    a = p.parse_args()
    result = merge(a.base, a.addition, a.map, a.output)
    print(json.dumps({k: v for k, v in result.items() if k != 'frames'}, ensure_ascii=False, indent=2))
