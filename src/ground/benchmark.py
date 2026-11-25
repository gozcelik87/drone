"""Precommit ground tours at upper road times, then replay without rerouting."""
from collections import Counter, defaultdict
from copy import deepcopy
from decimal import Decimal
from pathlib import Path
import csv
import hashlib
import json
import platform
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / 'code'))
sys.path.insert(0, str(HERE.parent / 'linked'))
sys.path.insert(0, str(HERE.parent / 'preparation'))
from ground_model import Ground, DESIGN
from independent_checks import evaluate as evaluate_hybrid

REALIZATIONS = ['lower', 'midpoint', 'upper']
load = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def write_csv(path, rows):
    with path.open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def decisions(plan):
    return [{'cluster_root': p['cluster_root'], 'order': p['order'],
             'paths': [e['path'] for e in p['legs']] + [p['return_leg']['path']],
             'loads': [(e['node'], e['units'], e['boxes']) for e in p['handovers']]}
            for p in plan.get('plans', [])]


def replay(ground, upper, raw, realization, changes):
    if upper['status'] != 'optimal_for_realization':
        return {'status': 'no_upper_plan', 'reason': upper['reason'],
                'interpretation': 'No feasible precommitted upper-time plan; not an independent infeasibility claim for this realization.'}
    out = deepcopy(upper)
    out['status'] = 'feasible_fixed_upper_plan'
    out['scope'] = 'Upper-time receiver order, all outbound paths and return path frozen; realized clocks only.'
    costs = ground.costs[realization]

    def update_leg(leg, source, target):
        ground.roads.validate(leg['path'], source, target)
        leg['road_sec'] = int(ground.roads.cost(leg['path'], costs))
        leg['transfer_sec'] = (leg['road_sec'] + ground.roads.sites[source]['access_sec']
                               + ground.roads.sites[target]['access_sec'])

    for plan in out['plans']:
        clock = changes.get('depot_loading_sec', DESIGN['depot_loading_sec'])
        age = changes.get('initial_component_age_sec', DESIGN['initial_component_age_sec'])
        for leg, event in zip(plan['legs'], plan['handovers']):
            update_leg(leg, leg['from_node'], leg['to_node'])
            clock += leg['transfer_sec'] + raw['stop_fixed_sec'] + raw['handover_per_box_sec'] * event['boxes']
            event['delivery_sec'] = clock
            event['thermal_age_sec'] = age + clock
            assert event['thermal_age_sec'] <= raw['thermal_limit_sec']
        plan['last_delivery_sec'] = clock
        update_leg(plan['return_leg'], plan['order'][-2], 'D01')
        plan['return_sec'] = clock + plan['return_leg']['transfer_sec']
        assert plan['return_sec'] <= DESIGN['vehicle_return_horizon_sec']
    out['last_delivery_sec'] = max(p['last_delivery_sec'] for p in out['plans'])
    out['vehicle_return_sec'] = max(p['return_sec'] for p in out['plans'])
    assert decisions(out) == decisions(upper)
    return out


class IndependentAudit:
    """Recompute path legality and events directly from graph records, not Roads."""
    def __init__(self, graph):
        self.graph = graph
        self.edges = {e['id']: e for e in graph['edges']}
        self.sites = {s['id']: s for s in graph['sites']}
        self.rules = defaultdict(list)
        for rule in graph['restrictions']:
            self.rules[rule['via'], rule['from_way']].append(rule)
        self.paths_checked = self.events_checked = self.plans_checked = 0

    def leg(self, path, source, target, realization):
        edges = [self.edges[i] for i in path]
        assert len(set(path)) == len(path)
        if not edges:
            assert self.sites[source]['osm_node'] == self.sites[target]['osm_node']
        else:
            assert edges[0]['u'] == self.sites[source]['osm_node']
            assert edges[-1]['v'] == self.sites[target]['osm_node']
        total = 0
        for edge in edges:
            total += {'lower': edge['lower_sec'], 'upper': edge['upper_sec'],
                      'midpoint': (edge['lower_sec'] + edge['upper_sec'] + 1) // 2}[realization]
        for a, b in zip(edges, edges[1:]):
            assert a['v'] == b['u'] and a['u'] != b['v']
            for rule in self.rules[a['v'], a['way']]:
                restriction = rule['restriction']
                if restriction.startswith('only_'):
                    assert b['way'] == rule['to_way']
                if restriction.startswith('no_') and b['way'] == rule['to_way']:
                    assert restriction == 'no_u_turn' and a['way'] == b['way'] and b['v'] != a['u']
            if a['way'] != b['way']:
                total += self.graph['turn_delay_sec']
        self.paths_checked += 1
        return total, total + self.sites[source]['access_sec'] + self.sites[target]['access_sec']

    def check(self, raw, plan, realization, changes):
        assert len(plan['plans']) == len(raw['roots'])
        packaging = raw['packaging']
        box_size = packaging['units_per_box']
        roots = {r['id']: r for r in raw['roots']}
        all_finish, all_return = [], []
        for tour in plan['plans']:
            root = roots[tour['cluster_root']]
            demands = dict(root['demands'])
            if root['root_demand']:
                demands[root['id']] = root['root_demand']
            nodes = [e['node'] for e in tour['handovers']]
            assert len(nodes) == len(set(nodes)) and set(nodes) == set(demands)
            assert tour['order'] == ['D01'] + nodes + ['D01']
            assert len(tour['legs']) == len(nodes)
            boxes = {node: (quantity + box_size - 1) // box_size for node, quantity in demands.items()}
            mass = Decimal(str(packaging['filled_bag_kg'])) * sum(demands.values()) + Decimal(str(packaging['box_and_coolant_kg'])) * sum(boxes.values())
            assert tour['boxes'] == sum(boxes.values()) <= root['vehicle_box_slots']
            assert mass == Decimal(str(tour['mass_kg'])) <= Decimal(str(root['vehicle_component_payload_kg']))
            clock = changes.get('depot_loading_sec', DESIGN['depot_loading_sec'])
            age = changes.get('initial_component_age_sec', DESIGN['initial_component_age_sec'])
            previous = 'D01'
            for leg, event in zip(tour['legs'], tour['handovers']):
                node = event['node']
                assert leg['from_node'] == previous and leg['to_node'] == node
                road, transfer = self.leg(leg['path'], previous, node, realization)
                assert (leg['road_sec'], leg['transfer_sec']) == (road, transfer)
                assert event['units'] == demands[node] and event['boxes'] == boxes[node]
                clock += transfer + raw['stop_fixed_sec'] + raw['handover_per_box_sec'] * boxes[node]
                assert event['delivery_sec'] == clock
                assert event['thermal_age_sec'] == age + clock <= raw['thermal_limit_sec']
                self.events_checked += 1
                previous = node
            assert tour['last_delivery_sec'] == clock
            road, transfer = self.leg(tour['return_leg']['path'], previous, 'D01', realization)
            assert (tour['return_leg']['road_sec'], tour['return_leg']['transfer_sec']) == (road, transfer)
            assert tour['return_sec'] == clock + transfer <= DESIGN['vehicle_return_horizon_sec']
            all_finish.append(clock)
            all_return.append(clock + transfer)
        assert plan['last_delivery_sec'] == max(all_finish)
        assert plan['vehicle_return_sec'] == max(all_return)
        self.plans_checked += 1


def synthetic_check():
    # A test where adaptive routing DOES improve delivery catches accidental
    # rerouting in replay; zero effects in the real instance alone cannot do so.
    graph = {'edges': [dict(id=i, u=a, v=b, way=i+1, lower_sec=lo, upper_sec=hi)
                       for i, (a, b, lo, hi) in enumerate([(1,2,10,60), (1,3,15,20), (3,2,15,20), (2,1,20,30)])],
             'sites': [dict(id=name, osm_node=node, access_sec=0) for name, node in [('D01',1), ('A',2)]],
             'restrictions': [], 'turn_delay_sec': 0}
    raw = {'packaging': {'units_per_box':12, 'filled_bag_kg':'0.35', 'box_and_coolant_kg':'8.45'},
           'roots':[dict(id='A',demands={},root_demand=1,available_units=1,vehicle_box_slots=10,vehicle_component_payload_kg='100')],
           'stop_fixed_sec':60,'handover_per_box_sec':30,'thermal_limit_sec':10000}
    model = Ground(graph)
    upper = model.ground_only(raw, 'upper', {})
    adaptive = model.ground_only(raw, 'lower', {})
    fixed = replay(model, upper, raw, 'lower', {})
    assert upper['plans'][0]['legs'][0]['path'] == [1,2]
    assert adaptive['plans'][0]['legs'][0]['path'] == [0]
    assert fixed['plans'][0]['legs'][0]['path'] == [1,2]
    assert fixed['last_delivery_sec'] - adaptive['last_delivery_sec'] == 20
    IndependentAudit(graph).check(raw, fixed, 'lower', {})
    thermal = deepcopy(raw); thermal['thermal_limit_sec'] = 410
    unavailable = model.ground_only(thermal, 'upper', {})
    assert unavailable['status'] == 'infeasible'
    assert model.ground_only(thermal, 'lower', {})['status'] == 'optimal_for_realization'
    assert replay(model, unavailable, thermal, 'lower', {})['status'] == 'no_upper_plan'
    # An independent checker must reject a changed event clock.
    corrupted = deepcopy(fixed); corrupted['plans'][0]['handovers'][0]['delivery_sec'] += 1
    try:
        IndependentAudit(graph).check(raw, corrupted, 'lower', {})
    except AssertionError:
        pass
    else:
        raise AssertionError('The independent checker accepted a corrupted event clock')
    return {'rerouting_sensitive_case_passed':True,'no_upper_plan_semantics_passed':True,'corrupted_event_rejected':True}


def main():
    started = time.perf_counter()
    for row in load(HERE / 'inputs/Input_Manifest.json'):
        assert sha(HERE / row['snapshot']) == row['sha256'], row['snapshot']
    synthetic = synthetic_check()
    graph = load(HERE.parent / 'linked/Road_Graph.json')
    model, audit = Ground(graph), IndependentAudit(graph)
    cases = load(HERE / 'inputs/Linked_Cases.json')
    out = HERE / 'results'; out.mkdir(exist_ok=True)
    rows, detail, no_upper = [], [], []
    hybrid_checks = 0
    for index, case in enumerate(cases, 1):
        upper_raw = case['realizations']['upper']['input']
        upper = model.ground_only(upper_raw, 'upper', case['changes'])
        assert upper == case['realizations']['upper']['ground'], case['case_id']
        if upper['status'] != 'optimal_for_realization':
            no_upper.append({'case_id':case['case_id'],'reason':upper['reason']})
        for realization in REALIZATIONS:
            evidence = case['realizations'][realization]
            raw = evidence['input']
            adaptive = model.ground_only(raw, realization, case['changes'])
            assert adaptive == evidence['ground'], (case['case_id'],realization)
            fixed = replay(model, upper, raw, realization, case['changes'])
            feasible = fixed['status'] == 'feasible_fixed_upper_plan'
            if adaptive['status'] == 'optimal_for_realization':
                audit.check(raw, adaptive, realization, case['changes'])
            if feasible:
                audit.check(raw, fixed, realization, case['changes'])
                assert fixed['last_delivery_sec'] >= adaptive['last_delivery_sec']
                if fixed['last_delivery_sec'] == adaptive['last_delivery_sec']:
                    assert fixed['vehicle_return_sec'] >= adaptive['vehicle_return_sec']
            hybrid = evidence['hybrid_metrics']
            if hybrid is not None:
                computed, events = evaluate_hybrid(raw, case['hybrid_plan'])
                assert computed == {k:v for k,v in hybrid.items() if k != 'flights'}, (case['case_id'],realization,'hybrid replay')
                hybrid_checks += 1
            plans = list(zip(fixed.get('plans',[]),adaptive.get('plans',[]))) if feasible else []
            outbound_different = any([e['path'] for e in a['legs']] != [e['path'] for e in b['legs']] for a,b in plans)
            return_different = any(a['return_leg']['path'] != b['return_leg']['path'] for a,b in plans)
            row = {'case_id':case['case_id'],'realization':realization,
                   'assigned_units':sum(r['available_units'] for r in raw['roots']),
                   'fixed_ground_status':fixed['status'],'fixed_ground_reason':fixed.get('reason',''),
                   'fixed_ground_delivery_sec':fixed.get('last_delivery_sec'),
                   'adaptive_ground_status':adaptive['status'],'adaptive_ground_delivery_sec':adaptive.get('last_delivery_sec'),
                   'delivery_difference_sec':fixed['last_delivery_sec']-adaptive['last_delivery_sec'] if feasible else None,
                   'fixed_ground_return_sec':fixed.get('vehicle_return_sec'),
                   'adaptive_ground_return_sec':adaptive.get('vehicle_return_sec'),
                   'return_difference_sec':fixed['vehicle_return_sec']-adaptive['vehicle_return_sec'] if feasible else None,
                   'visit_order_changed':any(a['order']!=b['order'] for a,b in plans) if feasible else None,
                   'outbound_paths_changed':outbound_different if feasible else None,
                   'return_paths_changed':return_different if feasible else None,
                   'hybrid_upper_status':case['hybrid_upper_status'],
                   'hybrid_delivery_sec':None if hybrid is None else hybrid['last_delivery_sec'],
                   'fixed_ground_minus_hybrid_sec':fixed['last_delivery_sec']-hybrid['last_delivery_sec'] if feasible and hybrid else None,
                   'adaptive_ground_minus_hybrid_sec':adaptive['last_delivery_sec']-hybrid['last_delivery_sec'] if adaptive['status']=='optimal_for_realization' and hybrid else None}
            rows.append(row)
            detail.append({'case_id':case['case_id'],'realization':realization,'fixed_ground':fixed,
                           'decision_sha256':hashlib.sha256(json.dumps(decisions(fixed),sort_keys=True).encode()).hexdigest() if feasible else None})
        if index % 20 == 0 or index == len(cases):
            print(json.dumps({'validated_linked_cases':index,'total':len(cases)}),flush=True)
    write_csv(out / 'Matched_Ground_Results.csv',rows)
    save(out / 'Fixed_Ground_Plans.json',detail)
    refcase = next(c for c in cases if c['case_id']=='reference')
    refrows = {r['realization']:r for r in rows if r['case_id']=='reference'}
    prep = []
    with (HERE / 'inputs/Archived_Preparation_Replays.csv').open(encoding='utf-8-sig',newline='') as stream:
        archived_prep = {(int(r['preparation_sec']),int(r['box_positions']),int(r['crews_per_root']),r['realization']):r for r in csv.DictReader(stream)}
    for case in load(HERE / 'inputs/Preparation_Cases.json'):
        for realization in REALIZATIONS:
            raw = deepcopy(case['input'])
            for root, reference_root in zip(raw['roots'],refcase['realizations'][realization]['input']['roots']):
                root['arrival_sec'] = reference_root['arrival_sec']
                root['age_at_arrival_sec'] = reference_root['age_at_arrival_sec']
            result = None
            if case['metrics'] is not None:
                result, events = evaluate_hybrid(raw,case['plan'])
                hybrid_checks += 1
            prep_sec = raw['preparation_sec']; boxes = raw['packaging']['available_box_slots']; crews = raw['roots'][0]['preparation_crews']
            old = archived_prep[prep_sec,boxes,crews,realization]
            assert (None if not old['last_delivery_sec'] else int(old['last_delivery_sec'])) == (None if result is None else result['last_delivery_sec'])
            fixed_ground = refrows[realization]['fixed_ground_delivery_sec']
            adaptive_ground = refrows[realization]['adaptive_ground_delivery_sec']
            prep.append({'case_id':case['case_id'],'preparation_sec':prep_sec,'box_positions':boxes,'crews_per_root':crews,
                         'realization':realization,'hybrid_upper_status':case['status'],
                         'hybrid_delivery_sec':None if result is None else result['last_delivery_sec'],
                         'fixed_ground_delivery_sec':fixed_ground,'adaptive_ground_delivery_sec':adaptive_ground,
                         'fixed_ground_minus_hybrid_sec':None if result is None else fixed_ground-result['last_delivery_sec'],
                         'adaptive_ground_minus_hybrid_sec':None if result is None else adaptive_ground-result['last_delivery_sec']})
    write_csv(out / 'Matched_Preparation_Results.csv',prep)
    feasible = [r for r in rows if r['fixed_ground_status']=='feasible_fixed_upper_plan']
    joint = [r for r in feasible if r['hybrid_delivery_sec'] is not None]
    faster_counts = {str(seconds):{real:sum(r['preparation_sec']==seconds and r['realization']==real and r['fixed_ground_minus_hybrid_sec'] is not None and r['fixed_ground_minus_hybrid_sec']>0 for r in prep) for real in REALIZATIONS} for seconds in [60,180,300]}
    report = {'protocol':'Plan ground orders, all delivery paths and return paths at upper road times; freeze all decisions across lower/rounded-midpoint/upper replays.',
              'same_information_timing':True,'same_resources_or_policy_objectives':False,
              'ground_methods_identical_to_archive':True,'adaptive_ground_results_reproduced':len(rows),
              'linked_designs':len(cases),'linked_records':len(rows),'upper_ground_feasible_designs':len(feasible)//3,
              'fixed_ground_feasible_replays':len(feasible),'ground_delivery_equal_records':sum(r['delivery_difference_sec']==0 for r in feasible),
              'joint_feasible_records':len(joint),'joint_delivery_comparison_changed':sum(r['fixed_ground_minus_hybrid_sec']!=r['adaptive_ground_minus_hybrid_sec'] for r in joint),
              'visit_order_changed_records':sum(r['visit_order_changed'] for r in feasible),
              'outbound_path_changed_records':sum(r['outbound_paths_changed'] for r in feasible),
              'return_path_changed_records':sum(r['return_paths_changed'] for r in feasible),
              'return_differences_sec':dict(Counter(r['return_difference_sec'] for r in feasible)),
              'no_upper_plan_cases':no_upper,
              'no_upper_plan_but_adaptive_feasible':[r for r in rows if r['fixed_ground_status']=='no_upper_plan' and r['adaptive_ground_status']=='optimal_for_realization'],
              'reference':[refrows[s] for s in REALIZATIONS],
              'preparation_configurations':len(prep)//3,'preparation_records':len(prep),
              'preparation_faster_counts':faster_counts,
              'preparation_comparison_changed':sum(r['fixed_ground_minus_hybrid_sec']!=r['adaptive_ground_minus_hybrid_sec'] for r in prep),
              'independent_ground_plan_checks':audit.plans_checked,'independent_path_checks':audit.paths_checked,
              'independent_handover_checks':audit.events_checked,'independent_hybrid_replays':hybrid_checks,
              'synthetic_checks':synthetic,'wall_sec':time.perf_counter()-started,'python':platform.python_version(),
              'limits':'Results concern the archived network, fixed assignments and three deterministic road-time realizations. Equal delivery times do not establish that information has no value in general. Upper-plan unavailability is not independent lower-realization infeasibility. Costs and resource provision remain unmatched.'}
    save(out / 'Summary_and_Checks.json',report)
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)


if __name__ == '__main__':
    main()
