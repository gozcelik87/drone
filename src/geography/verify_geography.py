"""Independent small-instance and evidence-semantics checks."""
import json
import math
from pathlib import Path
from geographic_core import assign, distance, silhouette, solve_all

HERE=Path(__file__).resolve().parent
checks={}

# Hand-solvable two separated pairs: minimum k=2 cost is 2; four medoid ties.
x=[0,1,10,11]
d=[[abs(a-b) for b in x] for a in x]
results, selected=solve_all(d,['a','b','c','d'])
r=results[0]
checks['four_point_oracle']=r['k']==2 and r['total_distance_to_medoid_km']==2 and r['medoid_ids']==['a','c'] and r['objective_ties_within_1e_9_km']==4
expected=(2*(9.5/10.5)+2*(8.5/9.5))/4
checks['ordinary_silhouette_hand_calculation']=math.isclose(r['ordinary_silhouette_mean'],expected,abs_tol=1e-12)
checks['singleton_silhouette_zero']=silhouette(d,[0,1,2,2])[0][:2]==[0,0]
checks['equidistant_assignment_lowest_id']=assign([[0,1,2],[1,0,1],[2,1,0]],(0,2))[0]==[0,0,2]

# GeographicLib's published inverse example catches swapped latitude/longitude.
checks['geodesic_reference_example']=abs(distance((-41.32,174.81),(40.96,-5.50))*1000-19959679.26735382)<1e-6

nodes=json.loads((HERE/'Tesis_Agi.json').read_text())
checks['no_silent_old_id_reuse']=all(n['id'].startswith('N') and n['historical_node_equivalence'] is False for n in nodes)
checks['operational_inputs_unfabricated']=all(n['demand_bags'] is None and n['blood_handling_eligible'] is None and n['road_accessible'] is None for n in nodes)
assert all(checks.values()),checks
(HERE/'Bagimsiz_Dogrulama.json').write_text(json.dumps({'checks':checks,'passed':len(checks),'failed':0},ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'passed':len(checks),'failed':0}))
