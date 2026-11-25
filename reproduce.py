"""Reproduce the numerical vehicle-drone blood-delivery experiments offline."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import uuid
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parent
sha = lambda raw: hashlib.sha256(raw).hexdigest()


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def rows(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def verify():
    manifest = read_json(ROOT/'MANIFEST.json')
    for rel, record in manifest.items():
        path = ROOT/rel
        assert path.is_file(), 'Missing: ' + rel
        raw = path.read_bytes()
        assert len(raw) == record['bytes'] and sha(raw) == record['sha256'], 'Changed: ' + rel
    with ZipFile(ROOT/'data/inputs.zip') as archive:
        assert archive.testzip() is None
    return {'files_verified': len(manifest), 'input_archive_verified': True}


def prepare(requested):
    work = Path(requested).expanduser().resolve() if requested else ROOT/'runs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8])
    if work.exists():
        raise ValueError('--work-dir must name a new directory')
    work.mkdir(parents=True)
    shutil.copytree(ROOT/'src', work/'experiments')
    (work/'logs').mkdir()
    with ZipFile(ROOT/'data/inputs.zip') as archive:
        for name in archive.namelist():
            rel = Path(name)
            assert not rel.is_absolute() and '..' not in rel.parts
            path = work/rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(archive.read(name))
    return work


def run(work, rel):
    start = time.perf_counter()
    path = work/'experiments'/rel
    log = work/'logs'/(rel.replace('/', '_')+'.log')
    print('Running ' + rel, flush=True)
    env = os.environ.copy()
    env.update(PYTHONDONTWRITEBYTECODE='1', PYTHONUTF8='1')
    with log.open('w', encoding='utf-8') as stream:
        result = subprocess.run([sys.executable, str(path)], cwd=work, env=env,
                                stdout=stream, stderr=subprocess.STDOUT)
    if result.returncode:
        print(log.read_text(encoding='utf-8')[-5000:], flush=True)
        raise RuntimeError('Failed: ' + rel)
    return {'script': rel, 'seconds': round(time.perf_counter()-start, 3), 'exit_code': 0}


def compare_inputs(work):
    targets = read_json(ROOT/'expected/input_checks.json')
    for rel, expected in targets.items():
        path = work/rel
        value = read_json(path) if path.suffix == '.json' else rows(path)
        if isinstance(value, dict):
            value = {k:v for k,v in value.items() if k != 'id'}
        actual = sha(json.dumps(value, sort_keys=True, ensure_ascii=False).encode())
        assert actual == expected, 'Regenerated input differs: ' + rel
    return len(targets)


def compare_csv(work, name, rel, keys=None, fields=None):
    expected = rows(ROOT/'expected'/name)
    actual = rows(work/'experiments'/rel)
    if keys:
        identify = lambda row: tuple(row[k] for k in keys)
        left = {identify(r):r for r in expected}; right = {identify(r):r for r in actual}
        assert len(left) == len(expected) == len(actual) == len(right)
        assert set(left) == set(right), name
        pairs = [(left[k], right[k]) for k in left]
    else:
        assert len(expected) == len(actual), name
        pairs = zip(expected, actual)
    for old, new in pairs:
        for key in fields or old.keys():
            assert old[key] == new[key], (name, key, old[key], new[key])
    return len(actual)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', nargs='?', default='verify', choices=['verify','check','recompute','resolve-roads'])
    parser.add_argument('--work-dir', help='New directory for generated data, results and logs')
    args = parser.parse_args()
    if sys.flags.optimize:
        raise RuntimeError('Run without -O: numerical checks require assertions.')
    report = {'command': args.command, 'integrity': verify(), 'commands': [], 'comparisons': {}}
    if args.command == 'verify':
        print(json.dumps(report, indent=2)); return
    work = prepare(args.work_dir)
    print('Output directory: ' + str(work), flush=True)
    report['status'] = 'running'
    try:
        for script in ['geography/rebuild_geography.py', 'demand/build_inputs.py', 'linked/build_roads.py']:
            report['commands'].append(run(work, script))
        report['comparisons']['regenerated_input_files'] = compare_inputs(work)
        if args.command == 'resolve-roads':
            for script in ['linked/solve_roads.py', 'linked/run_linked.py', 'linked/run_road_stress.py', 'linked/check_closure.py']:
                report['commands'].append(run(work, script))
            report['scope'] = 'New time-limited road optimization; selected paths, bounds and downstream results may differ.'
        else:
            for script in ['geography/verify_geography.py','demand/verify_inputs.py','linked/verify_link.py',
                           'linked/check_closure.py','ground/benchmark.py','load/run_load_preparation.py']:
                report['commands'].append(run(work, script))
            for name, rel in [('ground.csv','ground/results/Matched_Ground_Results.csv'),
                              ('ground_preparation.csv','ground/results/Matched_Preparation_Results.csv'),
                              ('load.csv','load/results/Load_Preparation_Replays.csv')]:
                report['comparisons'][name] = compare_csv(work, name, rel)
            if args.command == 'recompute':
                for script in ['drone/run_experiments.py', 'linked/run_linked.py', 'linked/audit_results.py',
                               'resource/run_resource_grid.py', 'preparation/run_preparation_grid.py']:
                    report['commands'].append(run(work, script))
                specs = [
                    ('drone.csv','drone/results/Deney_Sonuclari.csv',['case_id','policy'],
                     ['status','total_units','last_delivery_sec','drones_used','total_airborne_sec']),
                    ('linked.csv','linked/results/Bagli_Sonuclar.csv',['case_id','realization'],
                     ['assigned_units','hybrid_status','hybrid_last_delivery_sec','drones_used','ground_status','ground_last_delivery_sec','hybrid_reason','ground_reason']),
                    ('resource.csv','resource/results/Resource_Grid_Upper.csv',['box_positions_per_drone','preparation_crews_per_root'],
                     ['status','last_delivery_sec','drones_used','total_airborne_sec','ground_last_delivery_sec']),
                    ('preparation.csv','preparation/results/Preparation_Grid_Upper.csv',['preparation_sec','box_positions','crews_per_root'],
                     ['status','last_delivery_sec','drones_used','total_airborne_sec','ground_last_delivery_sec']),
                ]
                for name, rel, keys, fields in specs:
                    report['comparisons'][name] = compare_csv(work, name, rel, keys, fields)
            report['scope'] = 'Retained road decisions; ground and load-dependent checks replay the supplied flight plans. Recompute also re-solves drone allocation and uniform-preparation grids.'
        report['status'] = 'passed'
    except Exception as exc:
        report.update(status='failed', error=str(exc))
        raise
    finally:
        (work/'RUN_REPORT.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        print('Report: ' + str(work/'RUN_REPORT.json'), flush=True)
    print(json.dumps({'status':report['status'], 'comparisons':report['comparisons']}), flush=True)


if __name__ == '__main__':
    main()
