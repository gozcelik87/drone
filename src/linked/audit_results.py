"""Cross-check archived results against source demands, road legs and event clocks."""
from collections import Counter
from io import BytesIO
import csv,hashlib,json,math,posixpath
from pathlib import Path
from zipfile import ZipFile
import xml.etree.ElementTree as ET
import numpy as np
from road_core import Roads

HERE=Path(__file__).resolve().parent;ROOT=HERE.parent.parent
def load(path):return json.loads(path.read_text())
def csvread(path):
    with path.open(encoding='utf-8-sig') as f:return list(csv.DictReader(f))

def main():
    graph=load(HERE/'Road_Graph.json');roads=Roads(graph);design=load(HERE/'integration_design.json')
    demandrows=csvread(ROOT/'experiments/demand/Tesis_Talep_Senaryolari.csv')
    demands={sid:{r['node_id']:int(r['net_transport_units']) for r in demandrows if r['scenario_id']==sid and int(r['net_transport_units'])} for sid in {r['scenario_id'] for r in demandrows}}
    results=csvread(HERE/'results/Bagli_Sonuclar.csv');jobs={j['case']:j for j in load(HERE/'results/Deney_Listesi.json')}
    checked_ground=checked_hybrid=0
    def leg_check(a,b,leg,cost):
        roads.validate(leg['path'],a,b)
        value=sum(cost[i] for i in leg['path'])+sum(graph['turn_delay_sec'] for i,j in zip(leg['path'],leg['path'][1:]) if graph['edges'][i]['way']!=graph['edges'][j]['way'])
        assert value==leg['road_sec']
        assert value+roads.sites[a]['access_sec']+roads.sites[b]['access_sec']==leg['transfer_sec']
    for row in results:
        case,real=row['case_id'],row['realization'];job=jobs[case];changes=job['changes']
        expected=demands[job['scenario']];detail=load(HERE/'results'/(case+'__'+real+'.json'))
        cost=roads.lo if real=='lower' else roads.hi if real=='upper' else np.ceil((roads.lo+roads.hi)/2)
        initial_age=changes.get('initial_component_age_sec',design['initial_component_age_sec'])
        thermal=changes.get('drone',{}).get('thermal_limit_sec',7200)
        ground=detail['ground']
        if ground['status']=='optimal_for_realization':
            totals=Counter();delivery=0;returned=0
            for plan in ground['plans']:
                now=changes.get('depot_loading_sec',design['depot_loading_sec']);at='D01';boxes=units=0
                for leg,h in zip(plan['legs'],plan['handovers']):
                    node=h['node'];leg_check(at,node,leg,cost)
                    assert h['boxes']==math.ceil(h['units']/12)
                    now+=leg['transfer_sec']+60+30*h['boxes']
                    assert h['delivery_sec']==now and h['thermal_age_sec']==now+initial_age
                    assert now+initial_age<=thermal
                    totals[node]+=h['units'];boxes+=h['boxes'];units+=h['units'];at=node
                leg_check(at,'D01',plan['return_leg'],cost)
                assert now==plan['last_delivery_sec'] and now+plan['return_leg']['transfer_sec']==plan['return_sec']
                assert plan['return_sec']<=design['vehicle_return_horizon_sec']
                assert abs(plan['mass_kg']-(.35*units+8.45*boxes))<1e-6
                assert boxes<=changes.get('vehicle_box_slots',design['vehicle_box_slots'])
                assert plan['mass_kg']<=float(changes.get('vehicle_component_payload_kg',design['vehicle_component_payload_kg']))
                delivery=max(delivery,now);returned=max(returned,plan['return_sec'])
            assert dict(totals)==expected
            assert delivery==ground['last_delivery_sec']==int(row['ground_last_delivery_sec'])
            assert returned==int(row['ground_last_vehicle_return_sec']);checked_ground+=1
        else:assert row['ground_last_delivery_sec']==''
        metrics=detail['hybrid_metrics']
        if metrics is not None:
            totals=Counter();maximum=0
            releases={r['root']:r for r in detail['truck_metrics']}
            upper=load(HERE/'results'/(case+'__upper_plan.json'))['result']
            assert metrics['vehicle_loads']==upper['metrics']['vehicle_loads']
            for root,t in releases.items():
                roads.validate(t['outgoing_path'],'D01',root);roads.validate(t['return_path'],root,'D01')
                roadtime=roads.cost(t['outgoing_path'],cost)
                release=changes.get('depot_loading_sec',design['depot_loading_sec'])+roads.sites['D01']['access_sec']+roadtime+roads.sites[root]['access_sec']+design['root_unloading_sec']
                assert release==t['root_release_sec'] and initial_age+release==t['age_at_root_sec']
                assert release+roads.sites[root]['access_sec']+roads.cost(t['return_path'],cost)+roads.sites['D01']['access_sec']==t['vehicle_return_sec']
                if root in expected:
                    totals[root]+=expected[root];maximum=max(maximum,release+60+30*math.ceil(expected[root]/12))
            for flight in metrics['flights']:
                for h in flight['handovers']:
                    totals[h['node']]+=h['units'];maximum=max(maximum,h['delivery_sec'])
                    assert h['thermal_age_sec']==initial_age+h['delivery_sec']<=thermal
            assert dict(totals)==expected
            assert maximum==metrics['last_delivery_sec']==int(row['hybrid_last_delivery_sec'])
            assert maximum<=upper['metrics']['last_delivery_sec'];checked_hybrid+=1
        else:assert row['hybrid_last_delivery_sec']==''

    report=dict(all_passed=True,main_records=len(results),
        independently_checked_ground_records=checked_ground,
        independently_checked_hybrid_clock_records=checked_hybrid,
        all_quantity_totals_match_source_demands=True,
        clinical_or_operational_validation=False)
    (HERE/'Sonuc_Denetimi.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report))
if __name__=='__main__':main()
