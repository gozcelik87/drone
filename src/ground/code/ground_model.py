"""Ground-only delivery with exact receiver-order enumeration."""
from pathlib import Path
from decimal import Decimal
import itertools, json, math
import numpy as np
from road_core import Roads
DESIGN=json.loads((Path(__file__).resolve().parents[2]/'linked/integration_design.json').read_text())
grams=lambda x:int(Decimal(str(x))*1000)

class Ground:
    def __init__(self, graph):
        self.roads=Roads(graph)
        self.cache={}
        self.costs={'lower':self.roads.lo,'midpoint':np.ceil((self.roads.lo+self.roads.hi)/2),'upper':self.roads.hi}

    def leg(self,a,b,realization):
        key=(a,b,realization)
        if key not in self.cache:
            p,t=self.roads.shortest(a,b,self.costs[realization])
            self.cache[key]=dict(path=p,road_sec=None if p is None else int(t),
                                transfer_sec=None if p is None else int(t)+self.roads.sites[a]['access_sec']+self.roads.sites[b]['access_sec'])
        return self.cache[key]

    def supply_check(self,inst,changes,hybrid):
        total=sum(r['available_units'] for r in inst['roots'])
        if changes.get('depot_unavailable'):return 'depot_unavailable'
        if total>changes.get('depot_usable_compatible_units',DESIGN['depot_usable_compatible_units']):return 'insufficient_depot_stock'
        if hybrid and any(r['id'] in changes.get('ground_access_closed',[]) for r in inst['roots']):return 'launch_root_ground_access_unavailable'
        return None

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
