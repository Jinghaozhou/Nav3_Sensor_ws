#!/usr/bin/env python3
"""Summarize diagnostics from one experiment without changing its original CSV."""
import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path


def summarize(rows):
    def values(key, selected):
        return [float(row[key]) for row in selected if row.get(key, '') != '']

    def distribution(numbers):
        if not numbers:
            return None
        ordered = sorted(numbers)
        return {'count': len(numbers), 'p50': ordered[math.ceil(.5 * len(ordered)) - 1],
                'p95': ordered[math.ceil(.95 * len(ordered)) - 1], 'max': max(numbers)}

    def group(selected):
        count = len(selected)
        matched = sum(row['matched'] == '1' for row in selected)
        applied = sum(row['applied'] == '1' for row in selected)
        ran = [row for row in selected if row['coarse_ran'] == '1']
        fine = [row for row in selected if row['fine_ran'] == '1']
        return {
            'completed_attempts': count, 'matched': matched, 'applied': applied,
            'applied_rate_per_completed_attempt_percent': 100 * applied / count if count else None,
            'reasons': dict(Counter(row['reason'] for row in selected)),
            'registration_ms_excluding_not_run': distribution(
                [x for x in values('registration_ms', selected) if x > 0]),
            'input_age_at_result_sec': distribution(values('input_age_at_result_sec', selected)),
            'source_features_when_coarse_ran': distribution(values('source_features', ran)),
            'coarse_inliers_when_coarse_ran': distribution(values('coarse_inliers', ran)),
            'overlap_percent_when_fine_ran': distribution(values('overlap_percent', fine)),
        }

    return {'overall': group(rows),
            'by_mode': {mode: group([r for r in rows if r['mode'] == mode])
                        for mode in sorted({row['mode'] for row in rows})},
            'note': 'Matched/applied rates are acceptance statistics, not verified localization '
                    'accuracy. Compare the same route, map, parameters and actual window coverage. '
                    'Only completed attempts are present; interrupted computations are not counted.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path, help='Experiment folder or matching_metrics.csv')
    parser.add_argument('--no-plot', action='store_true', help='Only write statistics, without images')
    parser.add_argument('--map-pcd', type=Path, help='Optional explicit reference-map override')
    parser.add_argument('--until', help='ISO cutoff with timezone; later data excluded from statistics and images')
    args = parser.parse_args()
    path = args.session / 'matching_metrics.csv' if args.session.is_dir() else args.session
    with path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    from render_localization import cutoff_ns
    try:
        until = cutoff_ns(args.until)
    except ValueError as error:
        parser.error(str(error))
    if until is not None:
        rows = [row for row in rows if int(row['completed_ns']) <= until and int(row['cloud_ns']) <= until]
    result = summarize(rows)
    if until is None:
        output = path.with_name('matching_summary.json')
    else:
        from datetime import datetime
        from render_localization import TZ
        suffix = datetime.fromtimestamp(until/1e9, TZ).strftime('%Y%m%d_%H%M%S')
        output = path.with_name('matching_summary_until_' + suffix + '.json')
        result['cutoff'] = args.until
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    total = result['overall']
    print(f"已完成 {total['completed_attempts']} 次，匹配通过 {total['matched']} 次，实际应用 {total['applied']} 次")
    for mode, metrics in result['by_mode'].items():
        duration = metrics['registration_ms_excluding_not_run']
        print(f"{mode}: {metrics['completed_attempts']} 次，应用 {metrics['applied']} 次，"
              f"配准耗时P50/P95(ms)={duration['p50'] if duration else 'N/A'}/"
              f"{duration['p95'] if duration else 'N/A'}")
    print(f'详细统计：{output}')
    print('接受率不等于真实定位准确率；未完成的计算不计入此统计。')
    if not args.no_plot:
        from render_localization import render_session
        render_session(path.parent, args.map_pcd, until)


if __name__ == '__main__':
    main()
