"""One explicitly hypothetical physical-way closure, known before dispatch."""
from collections import defaultdict
import json,sys,time
from pathlib import Path
import numpy as np
from road_core import Roads
from run_linked import Context,DESIGN,save,csv_save
from linked_drone import solve,check_plan

HERE=Path(__file__).resolve().parent

def run():
    started=time.perf_counter();folder=HERE/'road_stress_results';folder.mkdir(exist_ok=True)
    graph=json.loads((HERE/'Road_Graph.json').read_text())
    base={t:json.loads((HERE/('Regret_'+t+'.json')).read_text()) for t in ['N04','N10']}
    endpoints={s['osm_node'] for s in graph['sites']}
    endpoint_ways={e['way'] for e in graph['edges'] if e['u'] in endpoints or e['v'] in endpoints}
    candidates=defaultdict(float)
    for i in set(base['N04']['incumbent']['path']+base['N10']['incumbent']['path']):
        e=graph['edges'][i]
        if e['way'] not in endpoint_ways:candidates[e['way']]+=e['length_m']
    closed=max(candidates,key=lambda w:(candidates[w],-w))
    roads=Roads(graph,closed_ways=[closed]);routes={}
    for root in ['N04','N10']:
        routes[root]=roads.solve_regret('D01',root,time_limit=30,max_iterations=6)
        save(folder/('Regret_'+root+'.json'),routes[root])
        print(root,routes[root]['status'],routes[root].get('absolute_gap_sec'),flush=True)
    details=dict(closed_osm_way=closed,way_names=sorted({e['name'] for e in graph['edges'] if e['way']==closed}),
        selection_rule='Largest summed length among baseline outbound selected-way segments, excluding ways incident to any modeled facility snap; ID breaks ties',
        counterfactual=True,closure_known_before_dispatch=True,observed_disaster_closure=False,
        deleted_directed_arcs=sum(e['way']==closed for e in graph['edges']),
        time_limit_per_master_sec=30,max_iterations=6)
    save(folder/'Kapanma_Tanimi.json',details)
    if any(r['status']=='unreachable' for r in routes.values()):
        save(folder/'Hesaplama_Kaydi.json',dict(**details,status='root_unreachable'));return
    ctx=Context(graph,routes)
    # Replace the topology for every subsequent route and recompute return paths.
    ctx.roads=roads;ctx.costs={'lower':roads.lo,'midpoint':np.ceil((roads.lo+roads.hi)/2),'upper':roads.hi};ctx.cache={}
    for root in ['N04','N10']:
        p,_=roads.shortest(root,'D01',roads.hi)
        if p is None:raise ValueError('Closure removes a required vehicle return')
        ctx.return_paths[root]=p
    upper=ctx.instance('S040','upper');result=solve(upper)
    save(folder/'Ust_Senaryo_Drone_Plani.json',result);rows=[]
    for realization in ['lower','midpoint','upper']:
        inst=ctx.instance('S040',realization);ground=ctx.ground_only(inst,realization,{})
        metrics=check_plan(inst,result['plan']) if result.get('metrics') else None
        trucks=ctx.truck_metrics(inst,realization) if metrics else None
        for root in inst['roots']:
            roads.validate(routes[root['id']]['incumbent']['path'],'D01',root['id'])
            roads.validate(ctx.return_paths[root['id']],root['id'],'D01')
        save(folder/(realization+'.json'),dict(hybrid_metrics=metrics,truck_metrics=trucks,ground=ground))
        rows.append(dict(case_id='physical_way_closure',realization=realization,hybrid_status=result['status'],
            hybrid_last_delivery_sec=None if metrics is None else metrics['last_delivery_sec'],ground_status=ground['status'],
            ground_last_delivery_sec=ground.get('last_delivery_sec')))
    csv_save(folder/'Kapanma_Sonuclari.csv',rows)
    save(folder/'Hesaplama_Kaydi.json',dict(**details,root_regret_status={k:{a:r[a] for a in ['status','absolute_gap_sec','lower_bound_sec']} for k,r in routes.items()},
        status='completed',total_wall_sec=time.perf_counter()-started,records=rows))
    print(json.dumps(rows),flush=True)

if __name__=='__main__':run()
