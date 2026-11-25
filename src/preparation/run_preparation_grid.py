"""Reproduce the 3 x 4 x 3 preparation/resource experiment, entirely offline.

All 36 configurations are reoptimized at upper road times. Their fixed plans
are replayed at lower and midpoint road times. No road path is reoptimized.
The solver and packaging modules use the reported numerical formulations.
"""
from collections import Counter
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
import csv
import hashlib
import json
import platform
import sys
import time

import numpy as np
import scipy

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'linked'))
from linked_drone import solve
from demand_payload import Packaging
from independent_checks import evaluate

PREPARATION = [60, 180, 300]
BOXES = [1, 2, 3, 4]
CREWS = [1, 2, 3]
LIMIT = 30
GOALS = ['last_delivery_sec', 'drones_used', 'total_airborne_sec']


def load(p):
    return json.loads(p.read_text())


def save(p, x):
    p.write_text(json.dumps(x, ensure_ascii=False, indent=2) + '\n')


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def write_csv(p, rows):
    with p.open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def solver_instance(raw):
    inst = deepcopy(raw)
    inst['arcs'] = {(x['from_node'], x['to_node']): x['flight_sec'] for x in raw['arcs']}
    return inst


def main():
    for row in load(HERE / 'inputs/Input_Manifest.json'):
        assert sha(HERE / row['snapshot']) == row['sha256'], row['snapshot']
    out = HERE / 'results'
    out.mkdir(exist_ok=True)
    baseline = load(HERE / 'inputs/Referans_Girdisi_Ust.json')
    road = {s: load(HERE / f'inputs/reference__{s}.json') for s in ['lower', 'midpoint', 'upper']}
    anchors = load(HERE / 'inputs/Archived_Objectives.json')
    assert sum(r['root_demand'] + sum(r['demands'].values()) for r in baseline['roots']) == 351
    assert sum(sum(r['demands'].values()) for r in baseline['roots']) == 281
    ground = road['upper']['ground']['last_delivery_sec']
    assert ground == 4188
    protocol = {
        'study_date': '2026-11-22', 'preparation_seconds_per_sortie': PREPARATION,
        'box_positions_per_drone': BOXES, 'preparation_crews_per_root': CREWS,
        'factorial_configurations': 36, 'all_configurations_freshly_reoptimized': True,
        'fixed_assigned_units': 351, 'fixed_air_delivered_units': 281,
        'fixed_mission_payload_kg': 50, 'fleet_by_root': {r['id']: r['fleet'] for r in baseline['roots']},
        'optimize_at': 'upper road times', 'replay_at': ['lower', 'midpoint', 'upper'],
        'plan_fixed_between_replays': ['quantities', 'boxes', 'route_orders', 'crew_slots'],
        'source_solver_unchanged': True, 'road_paths_unchanged': True,
        'preparation_model': 'Constant duration per sortie within each scenario; independent of load.',
        'ground_information': 'Archived realization-specific ground optimum; information asymmetry remains.',
        'objectives_lexicographic': GOALS, 'seconds_per_stage': LIMIT, 'target_relative_gap': 0,
        'interpretation': 'Designed conditional scenarios, not measured preparation times or probabilities.',
    }
    save(HERE / 'Experiment_Protocol.json', protocol)
    started = time.perf_counter()
    solutions, rows, replays, anchor_checks = {}, [], [], []
    event_checks = 0
    for prep in PREPARATION:
        for boxes in BOXES:
            for crews in CREWS:
                key = (prep, boxes, crews)
                case = f'prep_{prep:03d}_boxes_{boxes}_crews_{crews}'
                raw = deepcopy(baseline)
                raw['preparation_sec'] = prep
                raw['packaging']['available_box_slots'] = boxes
                for root in raw['roots']:
                    root['preparation_crews'] = crews
                normalized = deepcopy(raw)
                normalized['preparation_sec'] = baseline['preparation_sec']
                normalized['packaging']['available_box_slots'] = baseline['packaging']['available_box_slots']
                for root, original in zip(normalized['roots'], baseline['roots']):
                    root['preparation_crews'] = original['preparation_crews']
                assert normalized == baseline
                result = solve(solver_instance(raw), 'all', LIMIT)
                metric = result['metrics']
                if metric is not None:
                    checked, flights = evaluate(raw, result['plan'])
                    for field, value in checked.items():
                        assert metric[field] == value, (case, field)
                    assert result['status'] == 'lexicographic_optimal'
                    assert all(s['status'] == 0 and s['mip_gap'] == 0 for s in result['stages'])
                    event_checks += 1
                else:
                    assert boxes == 1 and result['status'] == 'proven_infeasible_precheck'
                    assert sum(r['fleet'] * 12 for r in raw['roots']) < 281
                for anchor in anchors:
                    if [prep, boxes, crews] != anchor['configuration']:
                        continue
                    assert result['status'] == anchor['status']
                    if metric is not None:
                        assert [metric[g] for g in GOALS] == anchor['objectives']
                    anchor_checks.append({'configuration': list(key), 'source': anchor['source'], 'passed': True})
                save(out / (case + '.json'), {'case_id': case, 'input': raw, 'result': result})
                solutions[key] = result
                capacity = Packaging(**raw['packaging']).one_recipient_capacity()
                rows.append({
                    'preparation_sec': prep, 'preparation_min': prep / 60,
                    'box_positions': boxes, 'crews_per_root': crews,
                    'single_recipient_capacity_units': capacity, 'status': result['status'],
                    'last_delivery_sec': None if metric is None else metric['last_delivery_sec'],
                    'last_delivery_min': None if metric is None else metric['last_delivery_sec'] / 60,
                    'drones_used': None if metric is None else metric['drones_used'],
                    'total_airborne_sec': None if metric is None else metric['total_airborne_sec'],
                    'boxes_by_air': None if metric is None else metric['boxes_by_air'],
                    'last_drone_return_sec': None if metric is None else metric['last_drone_return_sec'],
                    'ground_last_delivery_sec': ground,
                    'gain_vs_ground_sec': None if metric is None else ground - metric['last_delivery_sec'],
                    'variables': result.get('model_size', {}).get('variables'),
                    'base_constraints': result.get('model_size', {}).get('base_constraints'),
                    'max_stage_gap': max((s['mip_gap'] for s in result['stages']), default=None),
                    'solver_wall_sec': result['total_wall_sec'], 'reason': result.get('reason', ''),
                })
                for realization, record in road.items():
                    replay = deepcopy(raw)
                    release = {r['root']: r for r in record['truck_metrics']}
                    for root in replay['roots']:
                        root['arrival_sec'] = release[root['id']]['root_release_sec']
                        root['age_at_arrival_sec'] = release[root['id']]['age_at_root_sec']
                    checked = None
                    if metric is not None:
                        checked, _ = evaluate(replay, result['plan'])
                        assert checked['vehicle_loads'] == metric['vehicle_loads']
                        assert checked['last_delivery_sec'] <= metric['last_delivery_sec']
                        event_checks += 1
                    replays.append({
                        'preparation_sec': prep, 'box_positions': boxes, 'crews_per_root': crews,
                        'realization': realization, 'upper_plan_status': result['status'],
                        'replay_status': 'feasible_fixed_plan' if checked else 'no_upper_plan',
                        'last_delivery_sec': None if checked is None else checked['last_delivery_sec'],
                        'last_delivery_min': None if checked is None else checked['last_delivery_sec'] / 60,
                        'drones_used': None if checked is None else checked['drones_used'],
                        'ground_last_delivery_sec': record['ground']['last_delivery_sec'],
                        'gain_vs_ground_sec': None if checked is None else record['ground']['last_delivery_sec'] - checked['last_delivery_sec'],
                    })
                print(f'{case}: {result["status"]}; T={None if metric is None else round(metric["last_delivery_sec"]/60, 4)}; drones={None if metric is None else metric["drones_used"]}', flush=True)

    # Resource additions enlarge the feasible set; slower preparation shrinks it.
    monotone = []
    for key, result in solutions.items():
        if result['metrics'] is None:
            continue
        p, b, c = key
        for neighbor in [(p, b+1, c), (p, b, c+1)]:
            if neighbor in solutions:
                assert solutions[neighbor]['metrics']['last_delivery_sec'] <= result['metrics']['last_delivery_sec']
                monotone.append([list(key), list(neighbor), 'nonincreasing'])
        if p + 120 in PREPARATION:
            neighbor = (p+120, b, c)
            assert solutions[neighbor]['metrics']['last_delivery_sec'] >= result['metrics']['last_delivery_sec']
            monotone.append([list(key), list(neighbor), 'nondecreasing'])
    summaries = []
    for p in PREPARATION:
        T = lambda b, c: solutions[p,b,c]['metrics']['last_delivery_sec']
        minimum = min(T(b,c) for b in [2,3,4] for c in CREWS)
        best = [[b,c,solutions[p,b,c]['metrics']['drones_used']] for b in [2,3,4] for c in CREWS if T(b,c) == minimum]
        summaries.append({
            'preparation_sec': p, 'reference_sec': T(2,1), 'best_delivery_sec': minimum,
            'best_time_configurations_box_crew_drones': best,
            'fourth_box_gain_by_crews_sec': {str(c): T(3,c)-T(4,c) for c in CREWS},
            'crew_2_to_3_gain_by_boxes_sec': {str(b): T(b,2)-T(b,3) for b in [2,3,4]},
            'box_only_gain_sec': T(2,1)-T(4,1), 'crew_only_gain_sec': T(2,1)-T(2,3),
            'joint_gain_sec': T(2,1)-T(4,3),
            'separate_gains_overcount_sec': T(2,1)-T(4,1)-T(2,3)+T(4,3),
            'faster_than_ground_configurations': [[b,c] for b in [2,3,4] for c in CREWS if T(b,c)<ground],
        })
    assert len(rows) == 36 and len(replays) == 108
    assert Counter(r['status'] for r in rows) == {'lexicographic_optimal': 27, 'proven_infeasible_precheck': 9}
    assert len(anchor_checks) == len(anchors)
    report = {
        'executed_utc': datetime.now(timezone.utc).isoformat(),
        'configurations': 36, 'optimal_allocations': 27, 'capacity_infeasible': 9,
        'optimization_stages': sum(len(r['stages']) for r in solutions.values()),
        'all_stage_gaps_zero': True, 'road_replay_records': len(replays),
        'feasible_replays': sum(r['replay_status']=='feasible_fixed_plan' for r in replays),
        'independent_event_checks': event_checks,
        'archived_anchor_checks': anchor_checks, 'monotonicity_checks': monotone,
        'python': platform.python_version(), 'numpy': np.__version__, 'scipy': scipy.__version__,
        'platform': platform.platform(), 'elapsed_seconds': time.perf_counter()-started,
        'scope': 'Allocation optima conditional on fixed roads and scenario assumptions; no load-dependent preparation or matched-information ground comparison added.',
    }
    write_csv(out / 'Preparation_Grid_Upper.csv', rows)
    write_csv(out / 'Preparation_Grid_Replays.csv', replays)
    save(out / 'Interaction_and_Robustness.json', summaries)
    save(out / 'Computation_and_Checks.json', report)
    print(json.dumps({'summaries': summaries, 'checks': {k:v for k,v in report.items() if k not in ['archived_anchor_checks','monotonicity_checks']}}, indent=2), flush=True)


if __name__ == '__main__':
    main()
