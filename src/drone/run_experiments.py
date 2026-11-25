"""Same-instance comparisons; preserve infeasibility and solver limitations."""
import argparse
from copy import deepcopy
import csv
from datetime import datetime, timezone
from decimal import Decimal, ROUND_CEILING
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import time

import numpy as np
import scipy
from route_milp import solve, check_plan

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent.parent
DESIGN=json.loads((HERE/'experiment_design.json').read_text())


def read_csv(path):
    with path.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))


def write_csv(path,rows):
    with path.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def json_write(path,obj):path.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n')


def build_instance(scenario_id='S040',overrides=None):
    design=deepcopy(DESIGN);overrides=overrides or {}
    for k,v in overrides.items():
        if k=='packaging':design[k].update(v)
        else:design[k]=v
    ds=read_csv(ROOT/'experiments/demand/Tesis_Talep_Senaryolari.csv')
    demands={r['node_id']:int(r['net_transport_units']) for r in ds if r['scenario_id']==scenario_id}
    if len(demands)!=16:raise ValueError('Unknown demand scenario')
    memberships={r['id']:r['cluster_id'] for r in read_csv(ROOT/'experiments/geography/Kume_Uyelikleri.csv')}
    matrix=read_csv(ROOT/'experiments/geography/Mesafe_Matrisi_km.csv')
    distances={(r['from_node'],j):v for r in matrix for j,v in r.items() if j!='from_node' and j!=r['from_node']}
    arcs={}
    for edge,d in distances.items():
        seconds=Decimal(d)*3600*Decimal(str(design['distance_multiplier']))/Decimal(str(design['flight_speed_kmh']))
        arcs[edge]=int((seconds+design['maneuver_sec_per_leg']).to_integral_value(rounding=ROUND_CEILING))
    for a,b in design.get('removed_arcs',[]):arcs.pop((a,b),None)
    roots=[]
    for k,r in enumerate(design['roots'],1):
        subset={j:d for j,d in demands.items() if memberships[j]=='C'+str(k) and d>0}
        ground=subset.pop(r['id'],0)
        roots.append(dict(id=r['id'],fleet=r['fleet'],preparation_crews=r['preparation_crews'],
                          root_demand=ground,demands=subset,arrival_sec=design['root_arrival_sec'],
                          age_at_arrival_sec=design['age_at_vehicle_arrival_sec'],available_units=sum(subset.values())+ground))
    instance={k:design[k] for k in ['packaging','airborne_budget_sec','sortie_elapsed_budget_sec',
                                   'thermal_limit_sec','preparation_sec','stop_fixed_sec','handover_per_box_sec']}
    instance.update(roots=roots,arcs=arcs)
    return instance


def serial_instance(i):
    return {**{k:v for k,v in i.items() if k!='arcs'},
            'arcs':[dict(from_node=a,to_node=b,flight_sec=t) for (a,b),t in sorted(i['arcs'].items())]}


def sensitivity_cases():
    cases=[]
    for slots in [1,4,8]:cases.append((f'box_slots_{slots}',{'packaging':{'available_box_slots':slots}}))
    for kg in ['25','100','150']:cases.append((f'payload_{kg}kg',{'packaging':{'mission_payload_kg':kg}}))
    for seconds in [300,360,480,1200]:cases.append((f'air_budget_{seconds}s',{'airborne_budget_sec':seconds}))
    for speed in [30,60]:cases.append((f'speed_{speed}',{'flight_speed_kmh':speed}))
    for seconds in [60,300]:cases.append((f'prep_{seconds}s',{'preparation_sec':seconds}))
    for crews in [3,9]:
        rs=deepcopy(DESIGN['roots'])
        for r in rs:r['preparation_crews']=min(crews,r['fleet'])
        cases.append((f'crews_up_to_{crews}',{'roots':rs}))
    for sec in [2400,3600]:cases.append((f'thermal_limit_{sec}s',{'thermal_limit_sec':sec}))
    cases.append(('distance_factor_1p5',{'distance_multiplier':1.5}))
    cases.append(('one_air_leg_closed',{'removed_arcs':[['N10','N06']]}))
    rs=deepcopy(DESIGN['roots']);rs[0]['id']='N05'
    cases.append(('northern_root_moved_to_N05',{'roots':rs}))
    return cases


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--base-only',action='store_true')
    args=parser.parse_args();started=time.perf_counter()
    jobs=[('reference','S040',{})]
    if not args.base_only:
        scenarios=read_csv(ROOT/'experiments/demand/Talep_Senaryolari.csv')
        jobs += [('demand_'+r['scenario_id'],r['scenario_id'],{}) for r in scenarios if r['scenario_id']!='S040']
        jobs += [(name,'S040',overrides) for name,overrides in sensitivity_cases()]
    folder=HERE/('base_results' if args.base_only else 'results');folder.mkdir(exist_ok=True)
    records=[];cache={};instances={};policy_results={}
    for number,(case,scenario,overrides) in enumerate(jobs,1):
        instance=build_instance(scenario,overrides);serial=serial_instance(instance)
        fingerprint=hashlib.sha256(json.dumps(serial,sort_keys=True).encode()).hexdigest()
        instances[case]=dict(demand_scenario=scenario,overrides=overrides,input_fingerprint=fingerprint)
        policy_results[case]={}
        for policy in DESIGN['policies']:
            key=(fingerprint,policy);reused=key in cache
            if not reused:cache[key]=solve(instance,policy,DESIGN['solver_time_limit_per_stage_sec'])
            result=deepcopy(cache[key]);result.update(case_id=case,demand_scenario=scenario,
                input_fingerprint=fingerprint,cached_duplicate=reused)
            if result['metrics'] is not None:
                assert check_plan(instance,result['plan'])==result['metrics']
            policy_results[case][policy]=result
            json_write(folder/(case+'__'+policy+'.json'),result)
            metric=result['metrics'] or {}
            records.append(dict(case_id=case,demand_scenario=scenario,policy=policy,status=result['status'],
                total_units=sum(r['available_units'] for r in instance['roots']),
                ground_units=sum(r['root_demand'] for r in instance['roots']),
                remote_units=sum(sum(r['demands'].values()) for r in instance['roots']),
                last_delivery_sec=metric.get('last_delivery_sec'),last_drone_return_sec=metric.get('last_drone_return_sec'),
                drones_used=metric.get('drones_used'),total_airborne_sec=metric.get('total_airborne_sec'),
                boxes_by_air=metric.get('boxes_by_air'),wall_seconds=result['total_wall_sec'],cached_duplicate=reused,
                maximum_reported_mip_gap=max([x['mip_gap'] for x in result['stages'] if x['mip_gap'] is not None],default=None),
                route_candidates=sum(result.get('route_counts',{}).values()),
                variables=result.get('model_size',{}).get('variables'),
                reason=result.get('reason','')))
        if number==1 or number%10==0 or number==len(jobs):
            print(json.dumps({'case':number,'of':len(jobs),'id':case,'statuses':{p:r['status'] for p,r in policy_results[case].items()}},ensure_ascii=False),flush=True)
    comparisons=[]
    for case,by_policy in policy_results.items():
        allr=by_policy['all']
        for policy in ['nearest_neighbor','direct']:
            other=by_policy[policy]
            row=dict(case_id=case,baseline=policy,optimized_status=allr['status'],baseline_status=other['status'],
                     delivery_difference_sec=None,drone_difference=None,airborne_difference_sec=None)
            if allr['status']==other['status']=='lexicographic_optimal':
                keys=['last_delivery_sec','drones_used','total_airborne_sec']
                a=tuple(allr['metrics'][k] for k in keys);b=tuple(other['metrics'][k] for k in keys)
                assert a<=b,('nested_policy_dominance',case,policy,a,b)
                row.update(delivery_difference_sec=b[0]-a[0],drone_difference=b[1]-a[1],airborne_difference_sec=b[2]-a[2])
            comparisons.append(row)
    write_csv(folder/'Deney_Sonuclari.csv',records);write_csv(folder/'Yontem_Karsilastirmalari.csv',comparisons)
    json_write(folder/'Deney_Listesi.json',instances)
    json_write(folder/'Ornek_Girdi.json',serial_instance(build_instance()))
    base=policy_results['reference']['all']
    if base['metrics']:
        flights=[]
        for trip in base['metrics']['flights']:
            flights.append(dict(root=trip['root'],slot=trip['slot']+1,route=' > '.join(trip['closed_route']),
                quantities=json.dumps(trip['quantities']),boxes=trip['boxes'],launch_mass_kg=trip['launch_mass_kg'],
                launch_sec=trip['launch_sec'],last_delivery_sec=trip['last_delivery_sec'],return_sec=trip['return_sec'],airborne_sec=trip['airborne_sec']))
        write_csv(folder/'Ornek_Rotalar.csv',flights)
    model_version=hashlib.sha256((HERE/'route_milp.py').read_bytes()).hexdigest()
    manifest=dict(generated_utc=datetime.now(timezone.utc).isoformat(),design_id=DESIGN['id'],
        python=platform.python_version(),platform=platform.platform(),numpy=np.__version__,scipy=scipy.__version__,
        solver='SciPy milp / bundled HiGHS 1.2.0 (version observed from solver banner)',logical_cpu_count=os.cpu_count(),
        hardware=platform.processor() or 'unavailable',
        method='Integer MILP; 3 sequential objectives; all simple routes enumerated per root, subject to declared budgets.',
        time_limit_per_stage_sec=DESIGN['solver_time_limit_per_stage_sec'],mip_relative_gap_target=0,
        case_count=len(jobs),method_case_count=len(records),unique_solver_instances=len(cache),
        reused_identical_input_method_cases=sum(r['cached_duplicate'] for r in records),
        total_wall_seconds=time.perf_counter()-started,
        status_counts={s:sum(r['status']==s for r in records) for s in sorted({r['status'] for r in records})},
        all_completed_plans_independently_checked=True,lexicographic_nested_policy_checks='passed where both proven optimal',
        thread_count='Solver default; no explicit thread cap set',random_seed='No random generation or randomized heuristic specified',
        scope='Source-geography/conditional-demand stress tests after synchronized root arrivals; not measured deployment or end-to-end earthquake response.',
        clinical_calibration=False,flight_qualification=False,
        sha256={'route_milp.py':model_version,'experiment_design.json':hashlib.sha256((HERE/'experiment_design.json').read_bytes()).hexdigest(),
                'run_experiments.py':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},
        input_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [
            ROOT/'experiments/geography/Mesafe_Matrisi_km.csv',ROOT/'experiments/geography/Kume_Uyelikleri.csv',
            ROOT/'experiments/demand/Tesis_Talep_Senaryolari.csv',ROOT/'experiments/demand/input_design.json']})
    json_write(folder/'Hesaplama_Kaydi.json',manifest)
    print(json.dumps({'status_counts':manifest['status_counts'],'wall_seconds':manifest['total_wall_seconds'],
                      'base_metrics':{k:v for k,v in (base['metrics'] or {}).items() if k!='flights'}},ensure_ascii=False),flush=True)


if __name__=='__main__':main()
