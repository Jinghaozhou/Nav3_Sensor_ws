#!/usr/bin/python3
"""Build an inactive aligned xyz/rgb/region_id binary PCD, with streamed QA.

Only numpy/scipy are required. Inputs are read-only. Existing outputs are refused.
10cm occupied cells are represented by their observed surface centroid, not filled
boxes. A point is region 1 iff its distance to one such centroid is <= tolerance.
Normals/intensity are intentionally dropped; RGB's four bytes are copied unchanged.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial import cKDTree


OUT_DTYPE = np.dtype([(k, '<f4') for k in ('x', 'y', 'z', 'rgb', 'region_id')])


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read_pcd(path):
    """Read binary PCD metadata; honor field COUNT, field order and trailing bytes."""
    meta = {}
    with open(path, 'rb') as f:
        for _ in range(256):
            line = f.readline()
            if not line:
                raise ValueError('Missing DATA header: ' + str(path))
            words = line.decode('ascii').strip().split()
            if words and not words[0].startswith('#'):
                meta[words[0].upper()] = words[1:]
            if words and words[0].upper() == 'DATA':
                offset = f.tell()
                break
        else:
            raise ValueError('Header too long')
    if meta['DATA'] != ['binary']:
        raise ValueError('Only DATA binary is supported')
    fields = meta['FIELDS']
    sizes = list(map(int, meta['SIZE']))
    types = meta['TYPE']
    counts = list(map(int, meta.get('COUNT', ['1'] * len(fields))))
    if not len(fields) == len(sizes) == len(types) == len(counts):
        raise ValueError('Inconsistent field metadata')
    dtype_fields = []
    seen = set()
    for i, (field, size, typ, count) in enumerate(zip(fields, sizes, types, counts)):
        name = field if field not in seen else field + '_' + str(i)
        seen.add(name)
        kind = {'F': 'f', 'I': 'i', 'U': 'u'}[typ]
        dt = np.dtype('<' + kind + str(size))
        dtype_fields.append((name, dt) if count == 1 else (name, dt, (count,)))
    dtype = np.dtype(dtype_fields)
    n = int(meta['POINTS'][0])
    if n != int(meta['WIDTH'][0]) * int(meta['HEIGHT'][0]):
        raise ValueError('POINTS differs from WIDTH * HEIGHT')
    payload = n * dtype.itemsize
    trailing = Path(path).stat().st_size - offset - payload
    if trailing < 0:
        raise ValueError('Truncated PCD')
    for key in ('x', 'y', 'z'):
        if key not in dtype.names or dtype[key].shape:
            raise ValueError('Missing scalar coordinate ' + key)
    info = dict(header=meta, header_bytes=offset, point_count=n,
                point_step=dtype.itemsize, payload_bytes=payload,
                trailing_bytes=trailing, file_bytes=Path(path).stat().st_size)
    return np.memmap(path, dtype=dtype, mode='r', offset=offset, shape=(n,)), info


def xyz(a):
    return np.column_stack([a[k] for k in ('x', 'y', 'z')]).astype(np.float64)


def cells(points, voxel):
    return np.floor(points / voxel).astype(np.int64)


def aggregate(points, voxel):
    keys, inv, counts = np.unique(cells(points, voxel), axis=0,
                                  return_inverse=True, return_counts=True)
    sums = np.column_stack([np.bincount(inv, weights=points[:, j]) for j in range(3)])
    return keys, sums, counts


def merge_aggregates(parts):
    keys, inv = np.unique(np.concatenate([p[0] for p in parts]), axis=0,
                          return_inverse=True)
    sums = np.concatenate([p[1] for p in parts])
    counts = np.concatenate([p[2] for p in parts])
    total = np.bincount(inv, weights=counts)
    centers = np.column_stack([np.bincount(inv, weights=sums[:, j]) / total
                               for j in range(3)])
    return keys, centers, total


def bbox_update(bounds, points):
    if len(points):
        bounds[0] = np.minimum(bounds[0], points.min(axis=0))
        bounds[1] = np.maximum(bounds[1], points.max(axis=0))


def bbox_new():
    return np.array([[np.inf]*3, [-np.inf]*3])


def distance_summary(distances, thresholds):
    return dict(count=len(distances), distance_m_quantiles=dict(zip(
        ['min', 'median', 'p95', 'p99', 'max'],
        np.quantile(distances, [0, .5, .95, .99, 1]).tolist())),
        within={str(t): dict(count=int(np.count_nonzero(distances <= t)),
                              fraction=float(np.mean(distances <= t))) for t in thresholds})


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--old-map', type=Path, required=True)
    p.add_argument('--transform', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--record-dir', type=Path, required=True)
    p.add_argument('--voxel', type=float, default=.1)
    p.add_argument('--surface-tolerance', type=float, default=.15)
    p.add_argument('--chunk-points', type=int, default=500000)
    p.add_argument('--geometry-sample', type=int, default=200000)
    a = p.parse_args()
    if a.voxel <= 0 or a.surface_tolerance <= 0 or a.chunk_points <= 0 or a.geometry_sample <= 0:
        p.error('Sizes and counts must be positive')
    inputs = [a.source, a.old_map, a.transform, Path(__file__)]
    if a.output.exists() or a.output.is_symlink():
        p.error('Refusing to replace existing output')
    if a.output.resolve() in [x.resolve() for x in inputs]:
        p.error('Output aliases an input')
    a.record_dir.mkdir(parents=True, exist_ok=True)
    report_path = a.record_dir / 'report.json'
    if report_path.exists():
        p.error('Refusing to replace existing report')
    started = time.time()
    before = {str(x.resolve()): sha256(x) for x in inputs}
    T = np.loadtxt(a.transform)
    if T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1]):
        raise ValueError('Invalid transform')
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(R), 1, atol=1e-5):
        raise ValueError('Transform rotation is not rigid')
    source, sinfo = read_pcd(a.source)
    old, oinfo = read_pcd(a.old_map)
    if source.dtype['rgb'].itemsize != 4 or source.dtype['rgb'].shape:
        raise ValueError('RGB must occupy four scalar bytes')
    if old.dtype['region_id'].shape:
        raise ValueError('region_id must be scalar')
    print('Headers parsed; building old occupied surface cells', flush=True)
    region_parts, full_parts = [], []
    old_counts = {0: 0, 1: 0}
    old_bbox = {i: bbox_new() for i in (0, 1)}
    old_nonfinite = 0
    for start in range(0, len(old), a.chunk_points):
        block = old[start:start+a.chunk_points]
        points = xyz(block)
        finite = np.isfinite(points).all(axis=1)
        old_nonfinite += int(np.count_nonzero(~finite))
        ids = block['region_id']
        if not np.isfinite(ids).all() or not np.isin(ids, [0, 1]).all():
            raise ValueError('Old map contains unexpected region IDs')
        full_parts.append(np.unique(cells(points[finite], a.voxel), axis=0))
        for rid in (0, 1):
            mask = finite & (ids == rid)
            old_counts[rid] += int(mask.sum())
            bbox_update(old_bbox[rid], points[mask])
            if rid == 1 and mask.any():
                region_parts.append(aggregate(points[mask], a.voxel))
    if old_nonfinite or not region_parts:
        raise ValueError('Old map nonfinite or no ID1 points')
    rkeys, centers, weights = merge_aggregates(region_parts)
    del region_parts
    fullkeys = np.unique(np.concatenate(full_parts), axis=0)
    del full_parts
    region_tree = cKDTree(centers)
    old_tree = cKDTree((fullkeys.astype(float) + .5) * a.voxel)
    del fullkeys
    lo = centers.min(axis=0) - a.surface_tolerance
    hi = centers.max(axis=0) + a.surface_tolerance
    print('Region occupied cells:', len(centers), 'old region counts:', old_counts, flush=True)
    header = ('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n'
              'FIELDS x y z rgb region_id\nSIZE 4 4 4 4 4\nTYPE F F F F F\n'
              'COUNT 1 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              'POINTS %d\nDATA binary\n' % (len(source), len(source))).encode('ascii')
    sample_indices = np.unique(np.linspace(0, len(source)-1,
                                           min(a.geometry_sample, len(source)), dtype=np.int64))
    samples = []
    output_counts = {0: 0, 1: 0}
    output_bbox = {i: bbox_new() for i in (0, 1)}
    with a.output.open('xb') as f:
        f.write(header)
        for start in range(0, len(source), a.chunk_points):
            block = source[start:start+a.chunk_points]
            points = xyz(block) @ R.T + T[:3, 3]
            if not np.isfinite(points).all():
                raise ValueError('Nonfinite source/transformed XYZ')
            # Classify the exact float32 coordinates that will be written.
            out = np.empty(len(block), dtype=OUT_DTYPE)
            for j, key in enumerate(('x', 'y', 'z')):
                out[key] = points[:, j]
            points = xyz(out)
            near = np.all((points >= lo) & (points <= hi), axis=1)
            labels = np.zeros(len(block), dtype=bool)
            labels[near] = region_tree.query(points[near], workers=1)[0] <= a.surface_tolerance
            out['region_id'] = labels
            # uint32 view preserves packed RGB, including any NaN float bit pattern.
            out['rgb'].view('<u4')[:] = block['rgb'].view('<u4')
            out.tofile(f)
            for rid in (0, 1):
                mask = labels == rid
                output_counts[rid] += int(mask.sum())
                bbox_update(output_bbox[rid], points[mask])
            ix = sample_indices[(sample_indices >= start) & (sample_indices < start+len(block))] - start
            samples.append(points[ix])
            print('Written %d / %d' % (start + len(block), len(source)), flush=True)
        f.flush()
        os.fsync(f.fileno())
    print('Verifying persisted output, RGB bits, transform, labels and region coverage', flush=True)
    output, outinfo = read_pcd(a.output)
    expected_size = len(header) + len(source) * OUT_DTYPE.itemsize
    if len(output) != len(source) or outinfo['file_bytes'] != expected_size or outinfo['trailing_bytes']:
        raise AssertionError('Output point count/size mismatch')
    candidate_parts = []
    max_coordinate_error = 0.
    rgb_mismatches = label_mismatches = nonfinite_xyz = 0
    verified_counts = {0: 0, 1: 0}
    for start in range(0, len(output), a.chunk_points):
        block = output[start:start+a.chunk_points]
        original = source[start:start+len(block)]
        points = xyz(block)
        expected = xyz(original) @ R.T + T[:3, 3]
        max_coordinate_error = max(max_coordinate_error, float(np.max(np.abs(points-expected))))
        nonfinite_xyz += int(np.count_nonzero(~np.isfinite(points).all(axis=1)))
        rgb_mismatches += int(np.count_nonzero(block['rgb'].view('<u4') != original['rgb'].view('<u4')))
        ids = block['region_id']
        if not np.isfinite(ids).all() or not np.isin(ids, [0, 1]).all():
            raise AssertionError('Invalid output region ID')
        near = np.all((points >= lo) & (points <= hi), axis=1)
        expected_labels = np.zeros(len(block), dtype=bool)
        expected_labels[near] = region_tree.query(points[near], workers=1)[0] <= a.surface_tolerance
        label_mismatches += int(np.count_nonzero(ids != expected_labels))
        for rid in (0, 1):
            verified_counts[rid] += int(np.count_nonzero(ids == rid))
        if (ids == 1).any():
            candidate_parts.append(aggregate(points[ids == 1], a.voxel))
    if nonfinite_xyz or rgb_mismatches or label_mismatches or verified_counts != output_counts:
        raise AssertionError('Persisted output validation failed')
    candidate_keys, candidate_centers, _ = merge_aggregates(candidate_parts)
    del candidate_parts
    candidate_tree = cKDTree(candidate_centers)
    region_dist = candidate_tree.query(centers, workers=1)[0]
    # Full original region raw-point coverage, not inferred from its bounding box.
    raw_coverage = {str(t): 0 for t in (.1, .15, .2, .3)}
    raw_count = 0
    old0_in_region_cells = 0
    rkey_set = set(map(tuple, rkeys.tolist()))
    for start in range(0, len(old), a.chunk_points):
        block = old[start:start+a.chunk_points]
        for rid in (0, 1):
            points = xyz(block[block['region_id'] == rid])
            if rid == 1:
                d = candidate_tree.query(points, workers=1)[0]
                raw_count += len(d)
                for threshold in raw_coverage:
                    raw_coverage[threshold] += int(np.count_nonzero(d <= float(threshold)))
            else:
                keys, count = np.unique(cells(points, a.voxel), axis=0, return_counts=True)
                old0_in_region_cells += sum(int(n) for k, n in zip(keys, count) if tuple(k) in rkey_set)
    geometry_dist = old_tree.query(np.concatenate(samples), workers=1)[0]
    after = {str(x.resolve()): sha256(x) for x in inputs}
    if before != after:
        raise AssertionError('An input changed during build')
    import resource
    report = dict(arguments={k: str(v) if isinstance(v, Path) else v for k, v in vars(a).items()},
        command=[sys.executable, *sys.argv], transform=T.tolist(),
        inputs_sha256_before=before, inputs_sha256_after=after,
        inputs_unchanged=True, source=sinfo, old_map=oinfo, output=outinfo,
        output_sha256=sha256(a.output), output_region_counts=output_counts,
        output_region_bbox={i: b.tolist() for i, b in output_bbox.items()},
        old_region_counts=old_counts, old_region_bbox={i: b.tolist() for i, b in old_bbox.items()},
        old_id1_occupied_cells=len(centers), candidate_id1_occupied_cells=len(candidate_keys),
        old_id0_points_in_id1_occupied_cells=old0_in_region_cells,
        label_rule='floor(XYZ/voxel) old-ID1 occupied cells, observed surface centroids; nearest centroid Euclidean distance <= tolerance; each source row appears once with exactly one ID; old ID0 is not used as a negative class because it duplicates the region',
        old_id1_cell_coverage=distance_summary(region_dist, [.1, .15, .2, .3]),
        old_id1_raw_point_coverage={t: dict(covered=n, total=raw_count, fraction=n/raw_count,
            missed_fraction=1-n/raw_count) for t, n in raw_coverage.items()},
        transformed_source_to_old_geometry=distance_summary(geometry_dist, [.1, .15, .2, .3]),
        geometry_metric='deterministic uniform source-row sample to old full-map occupied 10cm cell centers; not registration or a field localization test',
        validation=dict(nonfinite_xyz=nonfinite_xyz, nonfinite_region_id=0,
                        rgb_bit_mismatches=rgb_mismatches, label_mismatches=label_mismatches,
                        max_transform_float32_error_m=max_coordinate_error,
                        exact_file_size=True, point_count_preserved=True),
        note='RGB is packed bits, not a numeric float; potential RGB NaN bits are preserved. Normals and intensity omitted. Candidate is not activated.',
        elapsed_seconds=time.time()-started,
        peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024)
    with report_path.open('x') as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
        f.write('\n')
    print(json.dumps(dict(output=str(a.output), counts=output_counts,
                          report=str(report_path), seconds=report['elapsed_seconds']), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
