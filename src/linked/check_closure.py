"""Replay the archived known-way-closure plan without a new regret solve."""
from copy import deepcopy
import csv
import json
from pathlib import Path
import numpy as np
from road_core import Roads
from run_linked import Context
from linked_drone import check_plan

HERE=Path(__file__).resolve().parent


def load(path):return json.loads(path.read_text(encoding='utf-8'))


def main():
    folder=HERE/'road_stress_results'
    graph=load(HERE/'Road_Graph.json')
    closure=load(folder/'Kapanma_Tanimi.json')
    routes={g:load(folder/('Regret_'+g+'.json')) for g in ('N04','N10')}
    roads=Roads(graph,closed_ways=[closure['closed_osm_way']])
    ctx=Context(graph,routes)
    ctx.roads=roads;ctx.costs={'lower':roads.lo,'midpoint':np.ceil((roads.lo+roads.hi)/2),'upper':roads.hi};ctx.cache={}
    for g in routes:
        ctx.return_paths[g]=roads.shortest(g,'D01',roads.hi)[0]
        assert ctx.return_paths[g] is not None
        roads.validate(routes[g]['incumbent']['path'],'D01',g)
        evaluation=roads.evaluate(routes[g]['incumbent']['path'],'D01',g)
        assert evaluation['regret_sec']==routes[g]['incumbent']['regret_sec']
    saved=load(folder/'Ust_Senaryo_Drone_Plani.json')
    count=0
    for real in ('lower','midpoint','upper'):
        instance=ctx.instance('S040',real)
        old=load(folder/(real+'.json'))
        metrics=check_plan(instance,saved['plan']) if saved.get('metrics') else None
        assert metrics==old['hybrid_metrics']
        trucks=ctx.truck_metrics(instance,real) if metrics else None
        assert trucks==old['truck_metrics']
        ground=ctx.ground_only(instance,real,{})
        assert ground==old['ground']
        count+=1
    report=dict(all_passed=True,realization_records_checked=count,
                closed_osm_way=closure['closed_osm_way'],new_regret_optimization=False,
                source_or_operational_validation=False)
    (HERE/'Closure_Replay_Check.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report))


if __name__=='__main__':main()
