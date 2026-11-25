"""Load-sensitive preparation for archived flight plans; no routing MILP rerun.

Exactness is restricted to scheduling the retained jobs on identical crews.
The independent oracle enumerates orders and partitions on small instances.
"""
from collections import Counter
from copy import deepcopy
from decimal import Decimal
from functools import lru_cache
from itertools import combinations_with_replacement, permutations
from pathlib import Path
import csv
import hashlib
import json
import random
import sys
import time

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'preparation'))
from independent_checks import evaluate as uniform_check

load=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
grams=lambda x:int(Decimal(str(x))*1000)


def save(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def csv_save(path,rows):
    with path.open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def jobs_from_plan(raw,plan,profile):
    arcs={(a['from_node'],a['to_node']):a['flight_sec'] for a in raw['arcs']}
    size=raw['packaging']['units_per_box'];groups={r['id']:[] for r in raw['roots']}
    for flight in sorted(plan,key=lambda f:(f['root'],f['slot'])):
        boxes={node:(q+size-1)//size for node,q in flight['quantities'].items()}
        units=sum(flight['quantities'].values());count=sum(boxes.values())
        duration=profile['alpha']+profile['beta']*count+profile['gamma']*units
        assert type(duration) is int and duration>0
        at=flight['root'];tail=0;handover_offsets=[]
        for node in flight['stops']:
            tail+=arcs[at,node]+raw['stop_fixed_sec']+raw['handover_per_box_sec']*boxes[node]
            handover_offsets.append({'node':node,'units':flight['quantities'][node],'boxes':boxes[node],'offset_sec':tail})
            at=node
        groups[flight['root']].append({'id':f"{flight['root']}_{flight['slot']:02d}",
            'slot':flight['slot'],'duration_sec':duration,'boxes':count,'units':units,
            'delivery_tail_sec':tail,'return_tail_sec':tail+arcs[at,flight['root']],
            'handovers':handover_offsets})
    return groups


def exact_schedule(jobs,crews):
    """Exact subset sequencing, followed by subset partitioning across crews."""
    n=len(jobs);size=1<<n
    processing=[j['duration_sec'] for j in jobs];tails=[j['delivery_tail_sec'] for j in jobs]
    totals=[0]*size;single=[0]*size;orders=[() for _ in range(size)]
    for mask in range(1,size):
        bit=mask&-mask;index=bit.bit_length()-1
        totals[mask]=totals[mask^bit]+processing[index]
        options=[]
        for j in range(n):
            if mask&(1<<j):
                prior=mask^(1<<j)
                options.append((max(single[prior],totals[mask]+tails[j]),orders[prior]+(j,)))
        single[mask],orders[mask]=min(options)

    @lru_cache(None)
    def partition(k,mask):
        if mask==0:return 0,tuple(() for _ in range(k))
        if k==0:return float('inf'),()
        if k==1:return single[mask],(orders[mask],)
        first=mask&-mask;sub=mask;best=None
        while sub:
            if sub&first:
                other,rest=partition(k-1,mask^sub)
                candidate=(max(single[sub],other),(orders[sub],)+rest)
                if best is None or candidate<best:best=candidate
            sub=(sub-1)&mask
        return best

    optimum,assignment=partition(crews,size-1)
    return int(optimum),[list(a) for a in assignment]


def cyclic_schedule(jobs,crews):
    return [[i for i,j in enumerate(jobs) if j['slot']%crews==c] for c in range(crews)]


def schedule_value(jobs,assignment):
    result=0
    for sequence in assignment:
        elapsed=0
        for index in sequence:
            elapsed+=jobs[index]['duration_sec']
            result=max(result,elapsed+jobs[index]['delivery_tail_sec'])
    return result


def brute_force(jobs,crews):
    best=float('inf');n=len(jobs)
    for order in permutations(range(n)):
        for cuts in combinations_with_replacement(range(n+1),crews-1):
            positions=(0,)+cuts+(n,)
            assignment=[order[positions[c]:positions[c+1]] for c in range(crews)]
            best=min(best,schedule_value(jobs,assignment))
    return best


def check_exact_solver():
    rng=random.Random(20261123);count=0
    for n in range(1,6):
        for repeat in range(3):
            jobs=[{'duration_sec':rng.randint(1,12),'delivery_tail_sec':rng.randint(0,15)} for _ in range(n)]
            previous=float('inf')
            for crews in range(1,4):
                optimum,assignment=exact_schedule(jobs,crews)
                assert optimum==brute_force(jobs,crews)==schedule_value(jobs,assignment)
                assert sorted(i for seq in assignment for i in seq)==list(range(n))
                assert optimum<=previous;previous=optimum;count+=1
    return count


def events(raw,jobs_by_root,assignments):
    deliveries=[];returns=[];scheduled=[]
    for root in raw['roots']:
        rid=root['id'];jobs=jobs_by_root[rid]
        if root['root_demand']:
            boxes=(root['root_demand']+raw['packaging']['units_per_box']-1)//raw['packaging']['units_per_box']
            deliveries.append(root['arrival_sec']+raw['stop_fixed_sec']+raw['handover_per_box_sec']*boxes)
        for crew,sequence in enumerate(assignments[rid]):
            elapsed=0
            for index in sequence:
                job=jobs[index];start=root['arrival_sec']+elapsed;elapsed+=job['duration_sec']
                launch=root['arrival_sec']+elapsed
                handovers=[{**e,'delivery_sec':launch+e['offset_sec'],
                    'thermal_age_sec':root['age_at_arrival_sec']+launch+e['offset_sec']-root['arrival_sec']} for e in job['handovers']]
                deliveries.extend(e['delivery_sec'] for e in handovers)
                returned=launch+job['return_tail_sec'];returns.append(returned)
                scheduled.append({'id':job['id'],'root':rid,'slot':job['slot'],'crew':crew,
                    'prep_start_sec':start,'prep_finish_sec':launch,'preparation_sec':job['duration_sec'],
                    'boxes':job['boxes'],'units':job['units'],'handovers':handovers,'return_sec':returned})
    return {'last_delivery_sec':max(deliveries),'last_drone_return_sec':max(returns),
            'drones_used':len(scheduled),'events':scheduled}


def independent_audit(raw,plan,profile,record):
    """Rebuild jobs and flights from raw decisions, without the DP/job builder."""
    roots={r['id']:r for r in raw['roots']};p=raw['packaging']
    paths={(r['from_node'],r['to_node']):r['flight_sec'] for r in raw['arcs']}
    expected={(f['root'],f['slot']):f for f in plan}
    observed={(f['root'],f['slot']):f for f in record['events']}
    assert set(expected)==set(observed) and len(expected)==len(record['events'])
    crews={};delivered={r:Counter() for r in roots};latest=0;last_return=0
    vehicle_boxes={r:(roots[r]['root_demand']+p['units_per_box']-1)//p['units_per_box'] for r in roots}
    minimum_slack=raw['thermal_limit_sec'];airborne_total=0
    for rid,root in roots.items():
        assert sum(f['root']==rid for f in plan)<=root['fleet']
        if root['root_demand']:
            service=raw['stop_fixed_sec']+raw['handover_per_box_sec']*vehicle_boxes[rid]
            latest=max(latest,root['arrival_sec']+service)
            minimum_slack=min(minimum_slack,raw['thermal_limit_sec']-root['age_at_arrival_sec']-service)
    for key,flight in expected.items():
        event=observed[key];rid=flight['root'];root=roots[rid]
        assert 0<=event['crew']<root['preparation_crews']
        assert set(flight['quantities'])==set(flight['stops'])
        boxes={n:(q+p['units_per_box']-1)//p['units_per_box'] for n,q in flight['quantities'].items()}
        box_total=sum(boxes.values());unit_total=sum(flight['quantities'].values())
        duration=profile['alpha']+profile['beta']*box_total+profile['gamma']*unit_total
        assert event['preparation_sec']==duration
        assert event['boxes']==box_total and event['units']==unit_total
        assert event['prep_finish_sec']-event['prep_start_sec']==duration
        assert event['prep_start_sec']>=root['arrival_sec']
        crews.setdefault((rid,event['crew']),[]).append((event['prep_start_sec'],event['prep_finish_sec']))
        mass=unit_total*grams(p['filled_bag_kg'])+box_total*grams(p['box_and_coolant_kg'])+grams(p['attachment_kg'])
        assert box_total<=p['available_box_slots'] and mass<=grams(p['mission_payload_kg'])
        clock=event['prep_finish_sec'];at=rid;air=0
        assert len(event['handovers'])==len(flight['stops'])
        for node,handover in zip(flight['stops'],event['handovers']):
            air+=paths[at,node];clock+=paths[at,node]+raw['stop_fixed_sec']+raw['handover_per_box_sec']*boxes[node]
            age=root['age_at_arrival_sec']+clock-root['arrival_sec']
            assert handover['node']==node and handover['units']==flight['quantities'][node] and handover['boxes']==boxes[node]
            assert handover['delivery_sec']==clock and handover['thermal_age_sec']==age
            minimum_slack=min(minimum_slack,raw['thermal_limit_sec']-age)
            delivered[rid][node]+=flight['quantities'][node];latest=max(latest,clock);at=node
        clock+=paths[at,rid];air+=paths[at,rid]
        assert clock==event['return_sec']
        assert air<=raw['airborne_budget_sec'] and clock-event['prep_finish_sec']<=raw['sortie_elapsed_budget_sec']
        airborne_total+=air;last_return=max(last_return,clock);vehicle_boxes[rid]+=box_total
    for sequence in crews.values():
        ordered=sorted(sequence)
        assert all(a[1]<=b[0] for a,b in zip(ordered,ordered[1:]))
    for rid,root in roots.items():
        assert dict(delivered[rid])==root['demands']
        units=root['root_demand']+sum(delivered[rid].values())
        assert units<=root['available_units']
        assert vehicle_boxes[rid]<=root['vehicle_box_slots']
        assert units*grams(p['filled_bag_kg'])+vehicle_boxes[rid]*grams(p['box_and_coolant_kg'])<=grams(root['vehicle_component_payload_kg'])
    assert latest==record['last_delivery_sec'] and last_return==record['last_drone_return_sec']
    assert len(plan)==record['drones_used']
    return {'minimum_exposure_slack_sec':minimum_slack,'feasible':minimum_slack>=0,
            'total_airborne_sec':airborne_total,'boxes_by_air':sum(e['boxes'] for e in record['events'])}


def main():
    started=time.perf_counter()
    for row in load(HERE/'inputs/Input_Manifest.json'):
        assert sha(HERE/row['snapshot'])==row['sha256'],row['snapshot']
    oracle_count=check_exact_solver()
    protocol=load(HERE/'Experiment_Protocol.json')
    evidence=load(HERE/'inputs/Ground_Reference.json')
    ground={r['realization']:r['fixed_ground_delivery_sec'] for r in evidence['reference']}
    roads={r:load(HERE/'inputs'/f'reference__{r}.json') for r in ['lower','midpoint','upper']}
    output=HERE/'results';output.mkdir(exist_ok=True)
    rows=[];details=[];anchors=0;root_solves=0;checks=0;profiles={p['id']:p for p in protocol['profiles']}
    for boxes in [2,3,4]:
        for crews in [1,2,3]:
            source=load(HERE/'inputs'/f'boxes_{boxes}_crews_{crews}.json')
            raw=source['input'];plan=source['result']['plan']
            old,_=uniform_check(raw,plan)
            assert old['last_delivery_sec']==source['result']['metrics']['last_delivery_sec']
            for profile in protocol['profiles']:
                jobs=jobs_from_plan(raw,plan,profile)
                exact={};cyclic={};restricted={}
                for root in raw['roots']:
                    rid=root['id'];restricted[rid],exact[rid]=exact_schedule(jobs[rid],crews);root_solves+=1
                    cyclic[rid]=cyclic_schedule(jobs[rid],crews)
                    assert restricted[rid]<=schedule_value(jobs[rid],cyclic[rid])
                for policy,assignments in [('fixed_crew_order',cyclic),('rescheduled',exact)]:
                    for realization in ['lower','midpoint','upper']:
                        instance=deepcopy(raw)
                        releases={r['root']:r for r in roads[realization]['truck_metrics']}
                        for root in instance['roots']:
                            root['arrival_sec']=releases[root['id']]['root_release_sec']
                            root['age_at_arrival_sec']=releases[root['id']]['age_at_root_sec']
                        record=events(instance,jobs,assignments)
                        audit=independent_audit(instance,plan,profile,record);checks+=1
                        if profile['id']=='uniform' and realization=='upper':
                            assert record['last_delivery_sec']==old['last_delivery_sec'];anchors+=1
                            if policy=='fixed_crew_order':assert record['last_drone_return_sec']==old['last_drone_return_sec']
                        durations=[j['duration_sec'] for group in jobs.values() for j in group]
                        row={'box_positions':boxes,'crews_per_root':crews,'profile':profile['id'],'policy':policy,'realization':realization,
                             'status':'feasible_retained_plan' if audit['feasible'] else 'exposure_infeasible_replay',
                             'last_delivery_sec':record['last_delivery_sec'] if audit['feasible'] else None,
                             'completion_if_executed_sec':record['last_delivery_sec'],'last_drone_return_sec':record['last_drone_return_sec'],
                             'drones_used':record['drones_used'],'boxes_by_air':audit['boxes_by_air'],
                             'minimum_preparation_sec':min(durations),'maximum_preparation_sec':max(durations),
                             'total_preparation_work_sec':sum(durations),'minimum_exposure_slack_sec':audit['minimum_exposure_slack_sec'],
                             'ground_delivery_sec':ground[realization],
                             'ground_minus_hybrid_sec':ground[realization]-record['last_delivery_sec'] if audit['feasible'] else None}
                        rows.append(row)
                        details.append({**row,'assignments':assignments,'events':record['events'],
                                        'restricted_root_objectives_sec':restricted if policy=='rescheduled' else None})
            print(json.dumps({'completed_configuration':[boxes,crews]}),flush=True)
    assert len(rows)==protocol['expected_records']==216
    csv_save(output/'Load_Preparation_Replays.csv',rows)
    save(output/'Schedules_and_Events.json',details)
    upper=[r for r in rows if r['realization']=='upper' and r['policy']=='rescheduled']
    csv_save(output/'Load_Preparation_Upper.csv',upper)
    lookup={(r['box_positions'],r['crews_per_root'],r['profile'],r['policy'],r['realization']):r for r in rows}
    summary=[]
    for profile in profiles:
        def val(b,c,policy='rescheduled',realization='upper'):
            return lookup[b,c,profile,policy,realization]['last_delivery_sec']
        assert all(val(b,c)<=val(b,c,'fixed_crew_order') for b in [2,3,4] for c in [1,2,3])
        summary.append({'profile':profile,'upper_delivery_seconds':{f'{b}b_{c}c':val(b,c) for b in [2,3,4] for c in [1,2,3]},
            'faster_than_ground':{real:sum(val(b,c,realization=real)<ground[real] for b in [2,3,4] for c in [1,2,3]) for real in ground},
            'separate_gain_overstatement_2_to_3_boxes_sec':val(2,1)-val(3,1)-val(2,3)+val(3,3),
            'separate_gain_overstatement_2_to_4_boxes_sec':val(2,1)-val(4,1)-val(2,3)+val(4,3),
            'rescheduling_gains_upper_seconds':{f'{b}b_{c}c':val(b,c,'fixed_crew_order')-val(b,c) for b in [2,3,4] for c in [1,2,3]},
            'best_retained_configurations_upper':[f"{r['box_positions']}b_{r['crews_per_root']}c" for r in upper if r['profile']==profile and r['last_delivery_sec']==min(v['last_delivery_sec'] for v in upper if v['profile']==profile)]})
    # Test that the independent audit rejects a corrupted processing interval.
    row=details[0];raw=load(HERE/'inputs/boxes_2_crews_1.json')['input'];raw=deepcopy(raw)
    for root in raw['roots']:
        release=next(x for x in roads['lower']['truck_metrics'] if x['root']==root['id'])
        root['arrival_sec']=release['root_release_sec'];root['age_at_arrival_sec']=release['age_at_root_sec']
    plan=load(HERE/'inputs/boxes_2_crews_1.json')['result']['plan']
    bad={'last_delivery_sec':row['last_delivery_sec'],'last_drone_return_sec':row['last_drone_return_sec'],
         'drones_used':row['drones_used'],'events':deepcopy(row['events'])}
    bad['events'][0]['prep_finish_sec']+=1
    try:independent_audit(raw,plan,profiles['uniform'],bad)
    except AssertionError:pass
    else:raise AssertionError('Corrupt preparation event accepted')
    report={'configurations':9,'profiles':4,'schedule_policies':2,'road_realizations':3,'replay_records':216,
            'feasible_replays':sum(r['status']=='feasible_retained_plan' for r in rows),
            'exact_root_scheduling_problems':root_solves,'uniform_anchor_checks':anchors,
            'independent_event_and_load_checks':checks,'exhaustive_small_instance_oracles':oracle_count,
            'corrupted_processing_interval_rejected':True,
            'maximum_jobs_per_root':9,'routing_or_quantity_reoptimization':False,
            'exact_scope':'Only the crew schedule for each retained flight/quantity plan.',
            'profile_results':summary,'wall_sec':time.perf_counter()-started,
            'interpretation':'Analyst load-dependence profiles anchored at a full two-box load. Rankings concern the retained plans; a worse result with larger available capacity is not evidence that its reoptimized optimum worsens.'}
    save(output/'Summary_and_Checks.json',report)
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':main()
