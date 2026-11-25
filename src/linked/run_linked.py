"""Offline linked single-batch experiment with explicit conditional assumptions."""
from copy import deepcopy
import csv,hashlib,itertools,json,math,platform,sys,time
from datetime import datetime,timezone
from pathlib import Path
import numpy as np
import scipy

HERE=Path(__file__).resolve().parent;ROOT=HERE.parent.parent
sys.path.insert(0,str(HERE.parent/'drone'))
from run_experiments import build_instance,serial_instance,read_csv
from linked_drone import solve,check_plan,grams
from road_core import Roads
DESIGN=json.loads((HERE/'integration_design.json').read_text())

def save(path,obj):path.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n')
def csv_save(path,rows):
    with path.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

class Context:
    def __init__(self,graph,route_results):
        self.roads=Roads(graph);self.routes=route_results;self.cache={}
        self.costs={'lower':self.roads.lo,'midpoint':np.ceil((self.roads.lo+self.roads.hi)/2),'upper':self.roads.hi}
        self.return_paths={}
        for root in ['N04','N10']:
            p,_=self.roads.shortest(root,'D01',self.roads.hi)
            if p is None:raise ValueError('No return path from '+root)
            self.return_paths[root]=p

    def leg(self,a,b,realization):
        key=(a,b,realization)
        if key not in self.cache:
            p,t=self.roads.shortest(a,b,self.costs[realization])
            self.cache[key]=dict(path=p,road_sec=None if p is None else int(t),
                                transfer_sec=None if p is None else int(t)+self.roads.sites[a]['access_sec']+self.roads.sites[b]['access_sec'])
        return self.cache[key]

    def instance(self,scenario,realization,changes=None):
        changes=changes or {};overrides=deepcopy(changes.get('drone',{}));inst=build_instance(scenario,overrides)
        load=changes.get('depot_loading_sec',DESIGN['depot_loading_sec'])
        age=changes.get('initial_component_age_sec',DESIGN['initial_component_age_sec'])
        for root in inst['roots']:
            rid=root['id'];path=self.routes[rid]['incumbent']['path']
            root['arrival_sec']=load+self.roads.sites['D01']['access_sec']+int(self.roads.cost(path,self.costs[realization]))+self.roads.sites[rid]['access_sec']+DESIGN['root_unloading_sec']
            root['age_at_arrival_sec']=age+root['arrival_sec']
            root['vehicle_box_slots']=changes.get('vehicle_box_slots',DESIGN['vehicle_box_slots'])
            root['vehicle_component_payload_kg']=changes.get('vehicle_component_payload_kg',DESIGN['vehicle_component_payload_kg'])
        if changes.get('air_unavailable'):inst['arcs']={}
        return inst

    def supply_check(self,inst,changes,hybrid):
        total=sum(r['available_units'] for r in inst['roots'])
        if changes.get('depot_unavailable'):return 'depot_unavailable'
        if total>changes.get('depot_usable_compatible_units',DESIGN['depot_usable_compatible_units']):return 'insufficient_depot_stock'
        if hybrid and any(r['id'] in changes.get('ground_access_closed',[]) for r in inst['roots']):return 'launch_root_ground_access_unavailable'
        return None

    def truck_metrics(self,inst,realization):
        records=[]
        for root in inst['roots']:
            rid=root['id'];out=self.routes[rid]['incumbent']['path'];ret=self.return_paths[rid]
            returned=root['arrival_sec']+self.roads.sites[rid]['access_sec']+int(self.roads.cost(ret,self.costs[realization]))+self.roads.sites['D01']['access_sec']
            if returned>DESIGN['vehicle_return_horizon_sec']:raise ValueError('Vehicle return horizon exceeded')
            records.append(dict(root=rid,road_outbound_sec=int(self.roads.cost(out,self.costs[realization])),
                root_release_sec=root['arrival_sec'],age_at_root_sec=root['age_at_arrival_sec'],vehicle_return_sec=returned,
                outgoing_path=out,return_path=ret))
        return records

    def ground_only(self,inst,realization,changes):
        reason=self.supply_check(inst,changes,False)
        if reason:return dict(status='infeasible',reason=reason)
        p=inst['packaging'];B=p['units_per_box'];plans=[];finish=0
        load=changes.get('depot_loading_sec',DESIGN['depot_loading_sec'])
        age=changes.get('initial_component_age_sec',DESIGN['initial_component_age_sec'])
        for root in inst['roots']:
            demand=dict(root['demands'])
            if root['root_demand']:demand[root['id']]=root['root_demand']
            if any(j in changes.get('ground_access_closed',[]) for j in demand):return dict(status='infeasible',reason='receiver_ground_access_unavailable')
            boxes={j:math.ceil(q/B) for j,q in demand.items()}
            mass=grams(p['filled_bag_kg'])*sum(demand.values())+grams(p['box_and_coolant_kg'])*sum(boxes.values())
            if sum(boxes.values())>root['vehicle_box_slots'] or mass>grams(root['vehicle_component_payload_kg']):return dict(status='infeasible',reason='vehicle_capacity')
            candidates=[]
            for order in itertools.permutations(sorted(demand)):
                at='D01';clock=load;handovers=[];legs=[];possible=True
                for j in order:
                    leg=self.leg(at,j,realization)
                    if leg['path'] is None:possible=False;break
                    clock+=leg['transfer_sec']+inst['stop_fixed_sec']+inst['handover_per_box_sec']*boxes[j]
                    handovers.append(dict(node=j,units=demand[j],boxes=boxes[j],delivery_sec=clock,thermal_age_sec=age+clock))
                    legs.append(dict(from_node=at,to_node=j,**leg));at=j
                if not possible:continue
                ret=self.leg(at,'D01',realization)
                if ret['path'] is None:continue
                returned=clock+ret['transfer_sec']
                if age+clock>inst['thermal_limit_sec'] or returned>DESIGN['vehicle_return_horizon_sec']:continue
                candidates.append(dict(order=['D01']+list(order)+['D01'],handovers=handovers,legs=legs,
                                       return_leg=ret,last_delivery_sec=clock,return_sec=returned,boxes=sum(boxes.values()),mass_kg=mass/1000))
            if not candidates:return dict(status='infeasible',reason='no_thermal_and_return_feasible_ground_tour')
            best=min(candidates,key=lambda x:(x['last_delivery_sec'],x['return_sec'],x['order']));best['cluster_root']=root['id']
            plans.append(best);finish=max(finish,best['last_delivery_sec'])
        return dict(status='optimal_for_realization',last_delivery_sec=finish,vehicle_return_sec=max(p['return_sec'] for p in plans),
                    plans=plans,scope='Exact receiver-order enumeration and turn-aware shortest legs for fixed cluster assignments; full knowledge of this realization')


def run():
    started=time.perf_counter();folder=HERE/'results';folder.mkdir(exist_ok=True)
    graph=json.loads((HERE/'Road_Graph.json').read_text())
    routes={t:json.loads((HERE/('Regret_'+t+'.json')).read_text()) for t in ['N04','N10']}
    ctx=Context(graph,routes)
    jobs=[('reference','S040',{})]
    jobs += [('demand_'+r['scenario_id'],r['scenario_id'],{}) for r in read_csv(ROOT/'experiments/demand/Talep_Senaryolari.csv') if r['scenario_id']!='S040']
    rs=deepcopy(build_instance()['roots'])
    # Only id/fleet/crew entries are passed back into the upstream design builder.
    crews=[dict(id=r['id'],fleet=r['fleet'],preparation_crews=3) for r in rs]
    jobs += [
      ('depot_stock_300','S040',{'depot_usable_compatible_units':300}),
      ('depot_unavailable','S040',{'depot_unavailable':True}),
      ('vehicle_payload_200kg','S040',{'vehicle_component_payload_kg':'200'}),
      ('root_N10_ground_access_lost','S040',{'ground_access_closed':['N10']}),
      ('receiver_N02_ground_access_lost','S040',{'ground_access_closed':['N02']}),
      ('all_flights_unavailable','S040',{'air_unavailable':True}),
      ('loading_15min','S040',{'depot_loading_sec':900}),
      ('prior_component_age_30min','S040',{'initial_component_age_sec':1800}),
      ('thermal_budget_60min','S040',{'drone':{'thermal_limit_sec':3600}}),
      ('box_slots_4','S040',{'drone':{'packaging':{'available_box_slots':4}}}),
      ('three_preparation_crews','S040',{'drone':{'roots':crews}}),
    ]
    records=[];details=[];cache={};joblist=[]
    for number,(case,scenario,changes) in enumerate(jobs,1):
        upper=ctx.instance(scenario,'upper',changes)
        reason=ctx.supply_check(upper,changes,True)
        fingerprint=hashlib.sha256(json.dumps(serial_instance(upper),sort_keys=True).encode()).hexdigest()
        result= dict(status='proven_infeasible_external',reason=reason,plan=[],metrics=None) if reason else None
        reused=False
        if result is None:
            reused=fingerprint in cache
            if not reused:cache[fingerprint]=solve(upper,'all',DESIGN['drone_solver_time_limit_per_stage_sec'])
            result=deepcopy(cache[fingerprint])
        save(folder/(case+'__upper_plan.json'),dict(case=case,scenario=scenario,changes=changes,input_fingerprint=fingerprint,cached=reused,result=result))
        joblist.append(dict(case=case,scenario=scenario,changes=changes,input_fingerprint=fingerprint,cached=reused))
        for realization in ['lower','midpoint','upper']:
            inst=ctx.instance(scenario,realization,changes)
            ground=ctx.ground_only(inst,realization,changes)
            metrics=None;trucks=None
            if result.get('metrics') is not None:
                metrics=check_plan(inst,result['plan']);trucks=ctx.truck_metrics(inst,realization)
                # The very same packing and flight/slot decisions are replayed.
                assert metrics['last_delivery_sec']<=result['metrics']['last_delivery_sec']
                assert metrics['vehicle_loads']==result['metrics']['vehicle_loads']
            joint=metrics is not None and ground['status']=='optimal_for_realization'
            records.append(dict(case_id=case,demand_scenario=scenario,realization=realization,
                assigned_units=sum(r['available_units'] for r in inst['roots']),hybrid_status=result['status'],
                hybrid_last_delivery_sec=None if metrics is None else metrics['last_delivery_sec'],
                hybrid_last_drone_return_sec=None if metrics is None else metrics['last_drone_return_sec'],
                hybrid_last_vehicle_return_sec=None if trucks is None else max(t['vehicle_return_sec'] for t in trucks),
                drones_used=None if metrics is None else metrics['drones_used'],
                ground_status=ground['status'],ground_last_delivery_sec=ground.get('last_delivery_sec'),
                ground_last_vehicle_return_sec=ground.get('vehicle_return_sec'),
                ground_minus_hybrid_delivery_sec=ground['last_delivery_sec']-metrics['last_delivery_sec'] if joint else None,
                hybrid_reason=result.get('reason',''),ground_reason=ground.get('reason','')))
            detail=dict(case_id=case,realization=realization,hybrid_metrics=metrics,truck_metrics=trucks,ground=ground)
            save(folder/(case+'__'+realization+'.json'),detail)
            if case=='reference':details.append(detail)
        if number==1 or number%10==0 or number==len(jobs):print(json.dumps({'case':number,'of':len(jobs),'id':case,'hybrid':result['status']},ensure_ascii=False),flush=True)
    csv_save(folder/'Bagli_Sonuclar.csv',records);save(folder/'Deney_Listesi.json',joblist)
    save(folder/'Referans_Girdisi_Ust.json',serial_instance(ctx.instance('S040','upper')))
    save(folder/'Yol_Matrisleri_ve_Rotalar.json',[dict(from_node=a,to_node=b,realization=c,**v) for (a,b,c),v in sorted(ctx.cache.items())])
    csv_save(folder/'Yol_Sure_Matrisleri.csv',[dict(from_node=a,to_node=b,realization=c,road_sec=v['road_sec'],with_access_sec=v['transfer_sec']) for (a,b,c),v in sorted(ctx.cache.items())])
    report=dict(generated_utc=datetime.now(timezone.utc).isoformat(),design_id=DESIGN['id'],
        python=platform.python_version(),platform=platform.platform(),scipy=scipy.__version__,numpy=np.__version__,
        solver='SciPy milp, bundled HiGHS 1.2.0; exhaustive ground receiver-order enumeration; Dijkstra on turn states',
        design_cases=len(jobs),realization_records=len(records),unique_hybrid_inputs_solved=len(cache),
        hybrid_status_counts={s:sum(x['hybrid_status']==s for x in records) for s in sorted({x['hybrid_status'] for x in records})},
        ground_status_counts={s:sum(x['ground_status']==s for x in records) for s in sorted({x['ground_status'] for x in records})},
        reference=[r for r in records if r['case_id']=='reference'],total_wall_sec=time.perf_counter()-started,
        unchanged_plan_replay_checked=True,road_regret_status={k:{a:r[a] for a in ['status','lower_bound_sec','absolute_gap_sec']} for k,r in routes.items()},
        interpretation='Designed conditional cases, not occurrence probabilities. All-lower/midpoint are unchanged-plan replays. An infeasible upper-planning case is not a proof that the lower realization alone has no feasible plan.',
        sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [HERE/'road_core.py',HERE/'linked_drone.py',Path(__file__),HERE/'integration_design.json',HERE/'Road_Graph.json',HERE/'Regret_N04.json',HERE/'Regret_N10.json']})
    save(folder/'Hesaplama_Kaydi.json',report);print(json.dumps(report['reference'],ensure_ascii=False),flush=True)

if __name__=='__main__':run()
