"""Reproduce the box-position x preparation-crew experiment offline.

The archived linked reference supplies fixed road routes, release times and the
same-workload ground benchmark. Only usable drone box positions and preparation
crews per root change. Road paths and ground tours are not reoptimized here.
Run with Python 3.9+ and the versions in requirements.txt.
"""
from pathlib import Path
from copy import deepcopy
from datetime import datetime,timezone
import csv,hashlib,json,platform,shutil,subprocess,sys,time
import numpy as np
import scipy

HERE=Path(__file__).resolve().parent
ARCHIVE=HERE/'inputs'
CODE=HERE.parent/'linked'
OUT=HERE/'results'
BOX_LEVELS=[1,2,3,4]
CREW_LEVELS=[1,2,3]
LIMIT=30


def save(path,obj):
    path.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n')


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def snapshot():
    manifest = json.loads((ARCHIVE/'Input_Manifest.json').read_text())
    for row in manifest:
        assert sha(HERE/row['snapshot']) == row['sha256'], row
    return manifest


def serial(inst):
    return {**{k:v for k,v in inst.items() if k!='arcs'},
            'arcs':[{'from_node':a,'to_node':b,'flight_sec':v}for (a,b),v in sorted(inst['arcs'].items())]}


def instance(raw):
    obj=deepcopy(raw)
    obj['arcs']={(e['from_node'],e['to_node']):e['flight_sec']for e in raw['arcs']}
    return obj


def write_csv(path,rows):
    with path.open('w',encoding='utf-8-sig',newline='')as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]))
        writer.writeheader();writer.writerows(rows)


def main():
    snapshot();sys.path.insert(0,str(CODE))
    from linked_drone import solve,check_plan
    from demand_payload import Packaging
    OUT.mkdir(exist_ok=True)
    raw=json.loads((ARCHIVE/'Referans_Girdisi_Ust.json').read_text())
    baseline=instance(raw)
    road_records={s:json.loads((ARCHIVE/f'reference__{s}.json').read_text())for s in ['lower','midpoint','upper']}
    assert sum(r['available_units']for r in baseline['roots'])==351
    assert baseline['packaging']['mission_payload_kg']=='50'
    anchors={(2,1):'reference__upper_plan.json',(4,1):'box_slots_4__upper_plan.json',
             (2,3):'three_preparation_crews__upper_plan.json'}
    protocol={'study_date':'2026-11-21','question':'How do usable box positions and preparation crews jointly affect completion and drone use?',
              'box_positions_per_drone':BOX_LEVELS,'preparation_crews_per_root':CREW_LEVELS,
              'full_factorial_cases':12,'fixed_assigned_units':351,'fixed_air_delivered_units':281,
              'mission_payload_kg':50,'fleet_by_root':{r['id']:r['fleet']for r in baseline['roots']},
              'fixed_preparation_seconds':180,'plan_at':'all-upper road times',
              'replay_at':['all-lower','rounded-midpoint','all-upper'],
              'plan_reoptimized_for_each_resource_pair':True,'plan_fixed_across_road_replays':True,
              'ground_benchmark':'Archived same-batch, same-vehicle ground tours; unchanged by these drone-only factors.',
              'lexicographic_objectives':['last delivery','drones used at optimal last delivery','total airborne seconds at preceding optima'],
              'solver_seconds_per_stage':LIMIT,'mip_relative_gap_target':0,
              'interpretation':'Designed deterministic scenarios, not random samples, probabilities, confidence intervals or measured disaster performance.',
              'scope':'Conditional on retained road paths and archived input assumptions. No new road-regret proof, cost optimization or deployment assessment.'}
    save(HERE/'Experiment_Protocol.json',protocol)
    started=time.perf_counter();rows=[];replays=[];solutions={};anchor_checks=[]
    for boxes in BOX_LEVELS:
        for crews in CREW_LEVELS:
            case=f'boxes_{boxes}_crews_{crews}'
            inst=deepcopy(baseline);inst['packaging']['available_box_slots']=boxes
            for root in inst['roots']:root['preparation_crews']=crews
            stripped=deepcopy(inst);stripped['packaging']['available_box_slots']=2
            for root in stripped['roots']:root['preparation_crews']=1
            assert stripped==baseline
            result=solve(inst,'all',LIMIT)
            cap=Packaging(**inst['packaging']).one_recipient_capacity()
            metrics=result['metrics']
            if metrics is not None:
                assert check_plan(inst,result['plan'])==metrics
            if (boxes,crews)in anchors:
                archived=json.loads((ARCHIVE/anchors[boxes,crews]).read_text())['result']
                goals=['last_delivery_sec','drones_used','total_airborne_sec']
                assert result['status']==archived['status']
                assert [metrics[k]for k in goals]==[archived['metrics'][k]for k in goals]
                anchor_checks.append({'box_positions':boxes,'crews':crews,'all_three_objectives_match':True})
            save(OUT/(case+'.json'),{'case_id':case,'input':serial(inst),
                 'input_sha256':hashlib.sha256(json.dumps(serial(inst),sort_keys=True).encode()).hexdigest(),'result':result})
            solutions[boxes,crews]=result
            ground_sec=road_records['upper']['ground']['last_delivery_sec']
            stages=result.get('stages',[])
            row={'box_positions_per_drone':boxes,'preparation_crews_per_root':crews,
                 'single_recipient_capacity_units':cap,'status':result['status'],
                 'last_delivery_sec':None if metrics is None else metrics['last_delivery_sec'],
                 'last_delivery_min':None if metrics is None else metrics['last_delivery_sec']/60,
                 'drones_used':None if metrics is None else metrics['drones_used'],
                 'ground_last_delivery_sec':ground_sec,
                 'gain_vs_ground_sec':None if metrics is None else ground_sec-metrics['last_delivery_sec'],
                 'gain_vs_ground_pct':None if metrics is None else 100*(ground_sec-metrics['last_delivery_sec'])/ground_sec,
                 'last_drone_return_sec':None if metrics is None else metrics['last_drone_return_sec'],
                 'total_airborne_sec':None if metrics is None else metrics['total_airborne_sec'],
                 'boxes_by_air':None if metrics is None else metrics['boxes_by_air'],
                 'variables':result.get('model_size',{}).get('variables'),
                 'base_constraints':result.get('model_size',{}).get('base_constraints'),
                 'maximum_mip_gap':max([s['mip_gap']for s in stages if s.get('mip_gap')is not None],default=None),
                 'solver_wall_sec':result['total_wall_sec'],'reason':result.get('reason','')}
            rows.append(row)
            for realization,record in road_records.items():
                replay=deepcopy(inst)
                times={r['root']:r for r in record['truck_metrics']}
                for r in replay['roots']:
                    r['arrival_sec']=times[r['id']]['root_release_sec']
                    r['age_at_arrival_sec']=times[r['id']]['age_at_root_sec']
                checked=None if metrics is None else check_plan(replay,result['plan'])
                if checked is not None:
                    assert checked['vehicle_loads']==metrics['vehicle_loads']
                    assert checked['last_delivery_sec']<=metrics['last_delivery_sec']
                    assert checked['drones_used']==metrics['drones_used']
                replays.append({'box_positions_per_drone':boxes,'preparation_crews_per_root':crews,
                    'realization':realization,'upper_plan_status':result['status'],
                    'last_delivery_sec':None if checked is None else checked['last_delivery_sec'],
                    'drones_used':None if checked is None else checked['drones_used'],
                    'ground_last_delivery_sec':record['ground']['last_delivery_sec'],
                    'gain_vs_ground_sec':None if checked is None else record['ground']['last_delivery_sec']-checked['last_delivery_sec']})
            print(json.dumps({k:row[k]for k in ['box_positions_per_drone','preparation_crews_per_root','status','last_delivery_min','drones_used']},ensure_ascii=False),flush=True)
    # Increasing either resource cannot worsen the proven primary optimum.
    monotonicity=[]
    for boxes,crews in solutions:
        old=solutions[boxes,crews]
        if old['status']!='lexicographic_optimal':continue
        for key in [(boxes+1,crews),(boxes,crews+1)]:
            if key not in solutions:continue
            new=solutions[key]
            assert new['status']=='lexicographic_optimal'
            assert new['metrics']['last_delivery_sec']<=old['metrics']['last_delivery_sec']
            monotonicity.append({'from':[boxes,crews],'to':list(key),'passed':True})
    # The one-position cases have a supply-independent fleet-capacity proof.
    for crews in CREW_LEVELS:
        assert solutions[1,crews]['status']=='proven_infeasible_precheck'
    assert len([s for s in solutions.values()if s['status']=='lexicographic_optimal'])==9
    assert all(s['status']==0 and s['mip_gap']==0 for r in solutions.values()for s in r['stages'])
    T=lambda b,c:solutions[b,c]['metrics']['last_delivery_sec']
    interaction={'reference_sec':T(2,1),'four_boxes_one_crew_sec':T(4,1),'two_boxes_three_crews_sec':T(2,3),
                 'four_boxes_three_crews_sec':T(4,3),
                 'box_gain_with_one_crew_sec':T(2,1)-T(4,1),
                 'box_gain_with_three_crews_sec':T(2,3)-T(4,3),
                 'crew_gain_with_two_boxes_sec':T(2,1)-T(2,3),
                 'crew_gain_with_four_boxes_sec':T(4,1)-T(4,3),
                 'joint_gain_sec':T(2,1)-T(4,3),
                 'sum_of_separate_gains_sec':2*T(2,1)-T(4,1)-T(2,3),
                 'overcount_if_separate_gains_are_added_sec':T(2,1)-T(4,1)-T(2,3)+T(4,3)}
    write_csv(OUT/'Resource_Grid_Upper.csv',rows);write_csv(OUT/'Resource_Grid_Replays.csv',replays)
    save(OUT/'Interaction_Summary.json',interaction)
    report={'study_date':'2026-11-21','executed_at_utc':datetime.now(timezone.utc).isoformat(),
            'python':platform.python_version(),'numpy':np.__version__,'scipy':scipy.__version__,
            'platform':platform.platform(),'machine':platform.machine(),
            'full_factorial_cases':len(rows),'proven_optimal_allocations':9,'proven_infeasible_capacity_cases':3,
            'road_replay_records':len(replays),'optimization_stages':sum(len(x['stages'])for x in solutions.values()),
            'zero_reported_mip_gaps':True,'all_raw_plans_independently_recomputed':True,
            'reference_anchor_checks':anchor_checks,'primary_monotonicity_checks':monotonicity,
            'model_variables':sorted({r['variables']for r in rows if r['variables']is not None}),
            'base_constraints':sorted({r['base_constraints']for r in rows if r['base_constraints']is not None}),
            'total_wall_seconds':time.perf_counter()-started,
            'scope':'Exact allocation optima conditional on retained road routes and fixed candidate sorties, not a globally optimal integrated network.'}
    for key in ['hw.physicalcpu','hw.logicalcpu','hw.memsize','machdep.cpu.brand_string']:
        try:
            r=subprocess.run(['sysctl','-n',key],capture_output=True,text=True,timeout=2)
            if r.returncode==0:report[key]=r.stdout.strip()
        except (OSError,subprocess.TimeoutExpired):pass
    save(OUT/'Computation_and_Checks.json',report)
    (HERE/'requirements.txt').write_text(f'numpy=={np.__version__}\nscipy=={scipy.__version__}\n')
    print(json.dumps(interaction,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':main()
