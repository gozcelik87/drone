"""Independent event/load replay.

Independent reconstruction from flight decisions and physical inputs.
Does not use solver constraints or the solver check_plan function.
"""
from collections import Counter
from decimal import Decimal
import math

def grams(x):return int(Decimal(str(x))*1000)

def evaluate(raw,plan):
    p=raw['packaging'];B=p['units_per_box'];slots=p['available_box_slots']
    mb,mw,ma,Q=map(grams,[p['filled_bag_kg'],p['box_and_coolant_kg'],p['attachment_kg'],p['mission_payload_kg']])
    legs={(e['from_node'],e['to_node']):e['flight_sec']for e in raw['arcs']}
    latest=returned=airtotal=boxesair=0;vehicles=[];flights=[]
    for root in raw['roots']:
        rid=root['id'];selected=sorted((f for f in plan if f['root']==rid),key=lambda f:f['slot'])
        assert [f['slot']for f in selected]==list(range(len(selected)))
        assert len(selected)<=root['fleet']
        delivered=Counter();vboxes=math.ceil(root['root_demand']/B)
        if root['root_demand']:
            service=raw['stop_fixed_sec']+raw['handover_per_box_sec']*vboxes
            assert root['age_at_arrival_sec']+service<=raw['thermal_limit_sec']
            latest=max(latest,root['arrival_sec']+service)
        for f in selected:
            stops=f['stops'];q=f['quantities']
            assert len(stops)==len(set(stops)) and set(q)==set(stops)<=set(root['demands'])
            assert all(type(v)is int and v>0 for v in q.values())
            boxes={j:math.ceil(q[j]/B)for j in stops};bn=sum(boxes.values())
            mass=mb*sum(q.values())+mw*bn+ma
            assert bn<=slots and mass<=Q
            wait=raw['preparation_sec']*(1+f['slot']//root['preparation_crews'])
            launch=root['arrival_sec']+wait;t=launch;a=0;lastnode=rid;handovers=[]
            for j in stops:
                a+=legs[lastnode,j];t+=legs[lastnode,j]+raw['stop_fixed_sec']+raw['handover_per_box_sec']*boxes[j]
                age=root['age_at_arrival_sec']+t-root['arrival_sec']
                assert age<=raw['thermal_limit_sec']
                delivered[j]+=q[j];latest=max(latest,t)
                handovers.append(dict(node=j,units=q[j],boxes=boxes[j],delivery_sec=t,thermal_age_sec=age))
                lastnode=j
            a+=legs[lastnode,rid];ret=t+legs[lastnode,rid]
            assert a<=raw['airborne_budget_sec'] and ret-launch<=raw['sortie_elapsed_budget_sec']
            airtotal+=a;boxesair+=bn;vboxes+=bn;returned=max(returned,ret)
            flights.append(dict(root=rid,slot=f['slot'],launch_sec=launch,return_sec=ret,handovers=handovers,mass_grams=mass))
        assert dict(delivered)==root['demands']
        units=root['root_demand']+sum(root['demands'].values())
        mass=units*mb+vboxes*mw
        assert units<=root['available_units'] and vboxes<=root['vehicle_box_slots'] and mass<=grams(root['vehicle_component_payload_kg'])
        vehicles.append(dict(root=rid,units=units,boxes=vboxes,component_mass_kg=mass/1000))
    return dict(last_delivery_sec=latest,last_drone_return_sec=returned,drones_used=len(plan),total_airborne_sec=airtotal,boxes_by_air=boxesair,vehicle_loads=vehicles),flights
