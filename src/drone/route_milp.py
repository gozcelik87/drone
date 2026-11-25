"""Closed-sortie MILP with sealed recipient boxes and finite preparation crews.

Research model: one sortie per drone, fixed root pools, no fleet reuse, all
planned shipments ready at vehicle arrival. Enumerated simple closed routes
guarantee connectivity. All times are integer seconds; masses are integer grams.
"""
from dataclasses import dataclass
from decimal import Decimal
from itertools import combinations, permutations
from math import ceil, isfinite
from pathlib import Path
import sys
import time

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, milp
from scipy.sparse import coo_matrix

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'demand'))
from demand_payload import Packaging


def nat(x, name, positive=False):
    if type(x) is not int or x < int(positive):
        raise ValueError(name + ': integer of the required sign is missing')
    return x


def grams(x):
    d = Decimal(str(x))*1000
    if not d.is_finite() or d < 0 or d != d.to_integral_value():
        raise ValueError('Mass must be nonnegative and specified to whole grams')
    return int(d)


@dataclass(frozen=True)
class Route:
    stops: tuple
    legs: tuple
    airborne_sec: int
    last_arrival_sec: int


def route_set(root, demand, arcs, package, airborne_budget, policy='all'):
    """All simple closed routes, direct routes, or NN orders for every subset.

    NN uses the shortest available next leg with ID tie breaking. It is a route
    ordering restriction; the same exact quantity allocator is used afterward.
    Missing legs are forbidden. Enumeration scales factorially, not polynomially.
    """
    if policy not in ('all', 'direct', 'nearest_neighbor'):
        raise ValueError('Unknown route policy')
    package.validate(); nat(airborne_budget, 'airborne budget', True)
    nodes = tuple(sorted(j for j, d in demand.items() if nat(d, 'demand') > 0))
    if root in demand:
        raise ValueError('Root demand is a separate ground handover')
    if len(nodes) > 8:
        raise ValueError('Explicit enumeration size guard; use column generation for larger instances')
    for edge, value in arcs.items():
        if len(edge) != 2 or edge[0] == edge[1]: raise ValueError('Invalid flight leg')
        nat(value, 'flight leg seconds', True)
    routes = []
    max_stops = min(len(nodes), package.available_box_slots)
    if policy == 'direct': max_stops = min(max_stops, 1)
    for size in range(1, max_stops+1):
        for subset in combinations(nodes, size):
            if policy == 'nearest_neighbor':
                left=set(subset); at=root; order=[]
                while left:
                    possible=[j for j in left if (at,j) in arcs]
                    if not possible:break
                    j=min(possible,key=lambda j:(arcs[at,j],j))
                    order.append(j); left.remove(j); at=j
                orders=[tuple(order)] if not left else []
            else: orders=permutations(subset)
            for order in orders:
                closed=(root,)+tuple(order)+(root,)
                legs=tuple(zip(closed,closed[1:]))
                if any(e not in arcs for e in legs):continue
                air=sum(arcs[e] for e in legs)
                if air>airborne_budget:continue
                if not package.pack(dict.fromkeys(order,1))['feasible']:continue
                routes.append(Route(tuple(order),legs,air,air-arcs[legs[-1]]))
    return routes


def validate_instance(instance):
    p=Packaging(**instance['packaging']);p.validate()
    for v in [p.filled_bag_kg,p.box_and_coolant_kg,p.attachment_kg,p.mission_payload_kg]:grams(v)
    nat(instance['airborne_budget_sec'],'airborne budget',True)
    nat(instance['sortie_elapsed_budget_sec'],'elapsed sortie budget',True)
    nat(instance['thermal_limit_sec'],'thermal limit',True)
    for key in ['stop_fixed_sec','handover_per_box_sec','preparation_sec']:
        nat(instance[key],key)
    for edge,value in instance['arcs'].items():
        if len(edge)!=2 or edge[0]==edge[1]:raise ValueError('Invalid flight leg')
        nat(value,'flight seconds',True)
    roots=instance['roots']
    if len({r['id'] for r in roots})!=len(roots):raise ValueError('Duplicate roots')
    served=[]
    for r in roots:
        for key in ['fleet','root_demand','arrival_sec','age_at_arrival_sec','available_units']:
            nat(r[key],key)
        nat(r['preparation_crews'],'preparation crews',True)
        for j,d in r['demands'].items():nat(d,'node demand');served.append(j)
        if r['id'] in r['demands']:raise ValueError('Root demand must be separated')
        if r['root_demand']:served.append(r['id'])
    if len(set(served))!=len(served):raise ValueError('Demand duplicated across roots')
    return p


def ground_handover(instance,r):
    if not r['root_demand']:return 0
    B=instance['packaging']['units_per_box']
    return instance['stop_fixed_sec']+instance['handover_per_box_sec']*ceil(r['root_demand']/B)


def check_plan(instance,plan):
    """Recompute edges, boxes, mass, release times and clocks from raw plan.

    Does not trust solver timing variables or stored route objects. Single-wave
    crews have fixed preparation slots. Receiving handover teams are unconstrained.
    """
    package=validate_instance(instance)
    roots={r['id']:r for r in instance['roots']}
    totals={(r['id'],j):0 for r in roots.values() for j in r['demands']}
    occupied=set(); flights=[];last=0; last_return=None;airtotal=0;boxed=0
    for r in roots.values():
        if r['available_units']<r['root_demand']+sum(r['demands'].values()):
            raise ValueError('Root supply insufficient')
        if r['root_demand']:
            gh=ground_handover(instance,r)
            if r['age_at_arrival_sec']+gh>instance['thermal_limit_sec']:
                raise ValueError('Ground handover exceeds thermal limit')
            last=max(last,r['arrival_sec']+gh)
    for trip in plan:
        r=roots[trip['root']]; k=nat(trip['slot'],'slot')
        if k>=r['fleet'] or (r['id'],k) in occupied:raise ValueError('Fleet slot reused or exceeded')
        occupied.add((r['id'],k))
        stops=tuple(trip['stops']); q=trip['quantities']
        if not stops or len(set(stops))!=len(stops) or set(stops)!=set(q):
            raise ValueError('Invalid service sequence or quantities')
        if any(j not in r['demands'] or type(q[j]) is not int or q[j]<1 for j in stops):
            raise ValueError('Every stop must deliver positive integer units')
        packing=package.pack(q)
        if not packing['feasible']:raise ValueError('Packed mass or box slots exceeded')
        closed=(r['id'],)+stops+(r['id'],)
        legs=tuple(zip(closed,closed[1:]))
        if any(e not in instance['arcs'] for e in legs):raise ValueError('Unavailable flight leg')
        air=sum(instance['arcs'][e] for e in legs)
        if air>instance['airborne_budget_sec']:raise ValueError('Return-inclusive airborne budget exceeded')
        launch_rel=instance['preparation_sec']*(1+k//r['preparation_crews'])
        elapsed=0;handovers=[]
        for edge,j in zip(legs[:-1],stops):
            elapsed+=instance['arcs'][edge]+instance['stop_fixed_sec']
            elapsed+=instance['handover_per_box_sec']*packing['boxes_by_recipient'][j]
            age=r['age_at_arrival_sec']+launch_rel+elapsed
            if age>instance['thermal_limit_sec']:raise ValueError('Handover exceeds thermal limit')
            totals[r['id'],j]+=q[j]
            handovers.append(dict(node=j,units=q[j],boxes=packing['boxes_by_recipient'][j],
                                  delivery_sec=r['arrival_sec']+launch_rel+elapsed,thermal_age_sec=age))
        return_elapsed=elapsed+instance['arcs'][legs[-1]]
        if return_elapsed>instance['sortie_elapsed_budget_sec']:
            raise ValueError('Elapsed sortie budget exceeded')
        finish=r['arrival_sec']+launch_rel+elapsed
        returned=r['arrival_sec']+launch_rel+return_elapsed
        last=max(last,finish);last_return=max(last_return or 0,returned)
        airtotal+=air;boxed+=packing['boxes']
        flights.append(dict(root=r['id'],slot=k,closed_route=list(closed),quantities=q,
                            boxes=packing['boxes'],launch_mass_kg=str(packing['launch_mass_kg']),
                            launch_sec=r['arrival_sec']+launch_rel,airborne_sec=air,
                            last_delivery_sec=finish,return_sec=returned,handovers=handovers))
    if any(totals[r['id'],j]!=d for r in roots.values() for j,d in r['demands'].items()):
        raise ValueError('Demand must be met exactly; shortage or oversupply')
    return dict(last_delivery_sec=last,last_drone_return_sec=last_return,drones_used=len(plan),
                total_airborne_sec=airtotal,boxes_by_air=boxed,flights=flights)


def solve(instance,policy='all',time_limit=30):
    started=time.perf_counter();p=validate_instance(instance)
    out=dict(policy=policy,status=None,stages=[],plan=[],metrics=None)
    cap=p.one_recipient_capacity()
    def infeasible(reason):
        out.update(status='proven_infeasible_precheck',reason=reason,total_wall_sec=time.perf_counter()-started)
        return out
    for r in instance['roots']:
        if r['available_units']<r['root_demand']+sum(r['demands'].values()):
            return infeasible('Insufficient pre-positioned units at '+r['id'])
        if sum(r['demands'].values())>r['fleet']*cap:
            return infeasible('Remote units exceed single-wave fleet capacity at '+r['id'])
        if sum(ceil(d/p.units_per_box) for d in r['demands'].values())>r['fleet']*p.available_box_slots:
            return infeasible('Recipient box lower bound exceeds fleet slots at '+r['id'])
        if r['root_demand'] and r['age_at_arrival_sec']+ground_handover(instance,r)>instance['thermal_limit_sec']:
            return infeasible('Ground handover thermal limit at '+r['id'])
    low=[];high=[];integ=[];meta=[];rows=[];rowlo=[];rowhi=[]
    def var(name,upper,integer=True,lower=0):
        i=len(meta);meta.append(name);low.append(lower);high.append(upper);integ.append(int(integer));return i
    def row(coef,lower=-np.inf,upper=np.inf):
        rows.append({k:v for k,v in coef.items() if v});rowlo.append(lower);rowhi.append(upper)
    ground=max([r['arrival_sec']+ground_handover(instance,r) for r in instance['roots'] if r['root_demand']]+[0])
    T=var(('last_delivery',),np.inf,lower=ground)
    demand_terms={(r['id'],j):{} for r in instance['roots'] for j in r['demands']}
    box_terms={(r['id'],j):{} for r in instance['roots'] for j in r['demands']}
    tripvars=[];routecounts={}
    mg,wg,ag,Qg=map(grams,[p.filled_bag_kg,p.box_and_coolant_kg,p.attachment_kg,p.mission_payload_kg])
    B=p.units_per_box
    analytical_bounds={}
    for root in instance['roots']:
        routes=route_set(root['id'],root['demands'],instance['arcs'],p,instance['airborne_budget_sec'],policy)
        routecounts[root['id']]=len(routes);previous=None;root_used={}
        # Valid workload/crew lower bound. Even the last of M required flights
        # cannot finish before its earliest preparation slot and minimum service.
        D=sum(root['demands'].values())
        needed=max(ceil(D/cap),ceil(sum(ceil(d/B) for d in root['demands'].values())/p.available_box_slots)) if D else 0
        if policy=='direct' and D:needed=max(needed,sum(ceil(d/cap) for d in root['demands'].values()))
        if needed>root['fleet']:return infeasible('Policy workload exceeds fixed fleet at '+root['id'])
        if needed and routes:
            quickest=min(r.last_arrival_sec+(instance['stop_fixed_sec']+instance['handover_per_box_sec'])*len(r.stops) for r in routes)
            bound=root['arrival_sec']+instance['preparation_sec']*(1+(needed-1)//root['preparation_crews'])+quickest
            low[T]=max(low[T],bound);analytical_bounds[root['id']]=bound
        # Route-specific upper bounds: the other recipients each need their
        # own box and at least one unit. These bounds strengthen the relaxation.
        route_caps={}
        for ri,r in enumerate(routes):
            qcap=max(q for q in range(1,cap+1)
                     if p.pack({j:(q if j==r.stops[0] else 1) for j in r.stops})['feasible'])
            route_caps[ri]=qcap
        for k in range(root['fleet']):
            used={};timing={T:-1};thermal={};elapsed={}
            launch=instance['preparation_sec']*(1+k//root['preparation_crews'])
            for ri,r in enumerate(routes):
                base=r.last_arrival_sec+instance['stop_fixed_sec']*len(r.stops)
                x=var(('route',root['id'],k,ri),1);used[x]=1;root_used[x]=1
                qs={};bs={};mass={x:-(Qg-ag)};slots={x:-p.available_box_slots}
                timing[x]=root['arrival_sec']+launch+base
                thermal[x]=launch+base+root['age_at_arrival_sec']-instance['thermal_limit_sec']
                elapsed[x]=r.airborne_sec+instance['stop_fixed_sec']*len(r.stops)-instance['sortie_elapsed_budget_sec']
                for j in r.stops:
                    qmax=min(root['demands'][j],route_caps[ri]);bmax=ceil(qmax/B)
                    q=var(('q',root['id'],k,ri,j),qmax)
                    b=var(('b',root['id'],k,ri,j),bmax)
                    qs[j]=q;bs[j]=b;demand_terms[root['id'],j][q]=1
                    box_terms[root['id'],j][b]=1
                    row({q:1,x:-1},lower=0)
                    row({q:1,x:-qmax},upper=0)
                    row({b:1,x:-bmax},upper=0)
                    row({q:1,b:-B},upper=0)
                    row({q:1,b:-B,x:B-1},lower=0)
                    mass[q]=mg;mass[b]=wg;slots[b]=1
                    for target in (timing,thermal,elapsed):target[b]=instance['handover_per_box_sec']
                row(mass,upper=0);row(slots,upper=0)
                tripvars.append(dict(root=root['id'],slot=k,route=r,x=x,q=qs,b=bs))
            row(used,upper=1);row(timing,upper=0);row(thermal,upper=0);row(elapsed,upper=0)
            if previous is not None:row({**previous,**{j:-v for j,v in used.items()}},lower=0)
            previous=used
        row(root_used,lower=needed)
    for root in instance['roots']:
        for j,demand in root['demands'].items():
            row(demand_terms[root['id'],j],lower=demand,upper=demand)
            row(box_terms[root['id'],j],lower=ceil(demand/B))
    out['route_counts']=routecounts
    out['analytical_delivery_lower_bounds_sec']=analytical_bounds
    out['model_size']=dict(variables=len(meta),binary_route_variables=len(tripvars),
                           integer_quantity_variables=sum(len(t['q']) for t in tripvars),
                           integer_box_variables=sum(len(t['b']) for t in tripvars),base_constraints=len(rows))
    goals=[('last_delivery_sec',{T:1}),('drones_used',{t['x']:1 for t in tripvars}),
           ('total_airborne_sec',{t['x']:t['route'].airborne_sec for t in tripvars})]
    last_solution=None
    for name,objective in goals:
        rr=[];cc=[];dd=[]
        for i,coefficients in enumerate(rows):
            for j,value in coefficients.items():rr.append(i);cc.append(j);dd.append(value)
        mat=coo_matrix((dd,(rr,cc)),shape=(len(rows),len(meta))).tocsc()
        c=np.zeros(len(meta))
        for i,v in objective.items():c[i]=v
        before=time.perf_counter()
        result=milp(c,integrality=np.array(integ),bounds=Bounds(low,high),
                    constraints=LinearConstraint(mat,rowlo,rowhi),
                    options=dict(time_limit=float(time_limit),mip_rel_gap=0.0,presolve=True))
        rec=dict(objective=name,status=int(result.status),message=result.message,
                 wall_sec=time.perf_counter()-before,variables=len(meta),constraints=len(rows))
        for key in ['fun','mip_gap','mip_node_count','mip_dual_bound']:
            v=getattr(result,key,None);rec[key]=float(v) if v is not None and isfinite(v) else None
        out['stages'].append(rec)
        if getattr(result,'x',None) is not None:last_solution=result.x
        if result.status!=0:
            out['status']='proven_infeasible_solver' if result.status==2 else 'solver_incomplete'
            break
        optimum=int(round(result.fun))
        row(objective,upper=optimum)
    else:out['status']='lexicographic_optimal'
    if last_solution is not None:
        if np.max(np.abs(last_solution-np.rint(last_solution)))>1e-5:
            raise ArithmeticError('Nonintegral solver output')
        values=np.rint(last_solution).astype(int)
        for t in tripvars:
            if values[t['x']]:
                q={j:int(values[i]) for j,i in t['q'].items()}
                expected=p.pack(q)['boxes_by_recipient']
                assert all(values[t['b'][j]]==expected[j] for j in q)
                out['plan'].append(dict(root=t['root'],slot=t['slot'],stops=list(t['route'].stops),quantities=q))
        out['metrics']=check_plan(instance,out['plan'])
        if out['status']=='lexicographic_optimal':
            assert [out['metrics'][name] for name,_ in goals]==[round(s['fun']) for s in out['stages']]
    out['total_wall_sec']=time.perf_counter()-started
    return out
