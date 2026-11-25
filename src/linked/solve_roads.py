"""Recompute the two reference access problems from the archived road graph.

Wall times, cut counts and time-limited bounds may vary by hardware. Scientific
claims must use the returned status and the feasible-route/global-bound gap.
"""
import hashlib,json,time
from pathlib import Path
from road_core import Roads

HERE=Path(__file__).resolve().parent
if __name__=='__main__':
    raw=(HERE/'Road_Graph.json').read_bytes();roads=Roads(json.loads(raw));records={}
    for target in ['N04','N10']:
        limit=20 if target=='N04' else 90
        r=roads.solve_regret('D01',target,time_limit=limit,max_iterations=10)
        r.update(time_limit_per_master_sec=limit,graph_sha256=hashlib.sha256(raw).hexdigest(),
                 code_sha256=hashlib.sha256((HERE/'road_core.py').read_bytes()).hexdigest())
        (HERE/('Regret_'+target+'.json')).write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
        print(target,r['status'],r.get('absolute_gap_sec'),flush=True)
