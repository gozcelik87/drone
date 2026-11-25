"""Turn-aware shortest paths and exact interval-regret constraint generation.

The same directed road segment has one uncertain time regardless of its incoming
turn. This avoids treating shared streets as independent origin/destination arcs.
"""
from collections import defaultdict
import heapq,math,time
import numpy as np
from scipy.optimize import milp,Bounds,LinearConstraint
from scipy.sparse import coo_matrix,vstack


class Roads:
    def __init__(self,graph,closed_ways=(),interval_scale=1.0):
        self.graph=graph;self.edges=graph['edges'];self.closed_ways=set(closed_ways)
        self.active={e['id'] for e in self.edges if e['way'] not in self.closed_ways}
        self.lo=np.array([e['lower_sec'] for e in self.edges],dtype=float)
        self.hi=np.array([e['lower_sec']+math.ceil((e['upper_sec']-e['lower_sec'])*interval_scale) for e in self.edges],dtype=float)
        self.sites={s['id']:s for s in graph['sites']};self.out=defaultdict(list)
        self.rules=defaultdict(list)
        for r in graph['restrictions']:self.rules[(r['via'],r['from_way'])].append(r)
        for e in self.edges:
            if e['id'] in self.active:self.out[e['u']].append(e['id'])
        self.next={};self.previous=defaultdict(list)
        for i in sorted(self.active):
            a=self.edges[i];allowed=[]
            for j in self.out[a['v']]:
                b=self.edges[j]
                if b['v']==a['u']:continue # conservative immediate U-turn exclusion
                legal=True
                for r in self.rules[(a['v'],a['way'])]:
                    if r['restriction'].startswith('only_') and b['way']!=r['to_way']:legal=False
                    if r['restriction'].startswith('no_') and b['way']==r['to_way']:
                        if r['restriction']=='no_u_turn' and a['way']==b['way']:
                            if b['v']==a['u']:legal=False
                        else:legal=False
                if legal:
                    turn=graph['turn_delay_sec'] if a['way']!=b['way'] else 0
                    allowed.append((j,turn));self.previous[j].append((i,turn))
            self.next[i]=allowed

    def node(self,site):return self.sites[site]['osm_node'] if isinstance(site,str) else site

    def shortest(self,source,target,cost=None,allowed=None,return_distances=False):
        s,t=self.node(source),self.node(target);cost=self.lo if cost is None else cost
        if s==t:return ([],0) if not return_distances else ({}, {})
        allowed=self.active if allowed is None else allowed
        distances={};parent={};q=[]
        for i in self.out[s]:
            if i in allowed:distances[i]=float(cost[i]);parent[i]=None;heapq.heappush(q,(distances[i],i))
        best=None
        while q:
            value,i=heapq.heappop(q)
            if value!=distances.get(i):continue
            if self.edges[i]['v']==t and not return_distances:best=i;break
            for j,turn in self.next[i]:
                if j not in allowed:continue
                candidate=value+cost[j]+turn
                if candidate<distances.get(j,math.inf):
                    distances[j]=candidate;parent[j]=i;heapq.heappush(q,(candidate,j))
        if return_distances:return distances,parent
        if best is None:return None,math.inf
        path=[];i=best
        while i is not None:path.append(i);i=parent[i]
        path.reverse();self.validate(path,s,t)
        return path,distances[best]

    def reverse_distances(self,target,cost):
        t=self.node(target);d={i:0.0 for i in self.active if self.edges[i]['v']==t};q=[(v,i) for i,v in d.items()];heapq.heapify(q)
        while q:
            value,i=heapq.heappop(q)
            if value!=d[i]:continue
            for j,turn in self.previous[i]:
                candidate=value+cost[i]+turn
                if candidate<d.get(j,math.inf):d[j]=candidate;heapq.heappush(q,(candidate,j))
        return d

    def validate(self,path,source,target):
        s,t=self.node(source),self.node(target)
        if not path:
            if s!=t:raise ValueError('Missing path')
            return
        if self.edges[path[0]]['u']!=s or self.edges[path[-1]]['v']!=t:raise ValueError('Wrong terminals')
        if len(set(path))!=len(path):raise ValueError('Repeated physical directed segment')
        for i in path:
            if i not in self.active:raise ValueError('Closed segment')
        for i,j in zip(path,path[1:]):
            if j not in {k for k,_ in self.next[i]}:raise ValueError('Disconnected or prohibited turn')

    def turn_cost(self,path):
        return sum(dict(self.next[i])[j] for i,j in zip(path,path[1:]))

    def cost(self,path,cost):return float(sum(cost[i] for i in path)+self.turn_cost(path))

    def evaluate(self,path,source,target):
        self.validate(path,source,target);scenario=self.lo.copy();scenario[path]=self.hi[path]
        comparator,best=self.shortest(source,target,scenario)
        regret=self.cost(path,self.hi)-best
        assert regret>=-1e-8
        return dict(path=path,lower_sec=int(self.cost(path,self.lo)),midpoint_sec=self.cost(path,(self.lo+self.hi)/2),
                    upper_sec=int(self.cost(path,self.hi)),regret_sec=int(round(regret)),
                    induced_comparator= comparator,induced_comparator_sec=int(round(best)),
                    length_m=sum(self.edges[i]['length_m'] for i in path))

    def solve_regret(self,source,target,time_limit=30,max_iterations=30):
        started=time.perf_counter();baselines={};candidate_paths=[]
        for name,cost in [('lower',self.lo),('midpoint',(self.lo+self.hi)/2),('upper',self.hi)]:
            p,_=self.shortest(source,target,cost)
            if p is None:return dict(status='unreachable',source=source,target=target)
            baselines[name]=self.evaluate(p,source,target);candidate_paths.append(p)
        incumbent=min(baselines.values(),key=lambda r:(r['regret_sec'],r['upper_sec'],r['path']))
        # Any improving path has lower cost <= incumbent regret + shortest upper cost.
        # This follows from R(P)>=c_lower(P)-min_Q c_upper(Q).
        bound=incumbent['regret_sec']+baselines['upper']['upper_sec']
        forward,_=self.shortest(source,target,self.lo,return_distances=True)
        reverse=self.reverse_distances(target,self.lo)
        keep={i for i in self.active if forward.get(i,math.inf)+reverse.get(i,math.inf)<=bound+1e-8}
        ids=sorted(keep);xindex={i:k for k,i in enumerate(ids)};s,t=self.node(source),self.node(target)
        transitions=[]
        for i in self.out[s]:
            if i in keep:transitions.append((-1,i,0))
        for i in ids:
            if self.edges[i]['v']==t:transitions.append((i,-2,0))
            else:
                for j,turn in self.next[i]:
                    if j in keep:transitions.append((i,j,turn))
        n=len(ids);zstart=n;nv=n+len(transitions)+1;ri=nv-1
        incoming=defaultdict(list);outgoing=defaultdict(list)
        for k,(a,b,turn) in enumerate(transitions):outgoing[a].append(zstart+k);incoming[b].append(zstart+k)
        rows=[];cols=[];values=[];lbs=[];ubs=[]
        def row(items,lb,ub):
            k=len(lbs)
            for c,v in items:
                if v:rows.append(k);cols.append(c);values.append(v)
            lbs.append(lb);ubs.append(ub)
        for i in ids:
            row([(k,1) for k in incoming[i]]+[(xindex[i],-1)],0,0)
            row([(k,1) for k in outgoing[i]]+[(xindex[i],-1)],0,0)
        row([(k,1) for k in outgoing[-1]],1,1)
        row([(k,1) for k in incoming[-2]],1,1)
        turn_coefs=[(zstart+k,c) for k,(_,_,c) in enumerate(transitions) if c]
        row([(xindex[i],self.lo[i]) for i in ids]+turn_coefs,-np.inf,bound)
        base=coo_matrix((values,(rows,cols)),shape=(len(lbs),nv)).tocsr()
        cuts=[];cut_set=set()
        def add_cut(p):
            key=tuple(p)
            if key in cut_set:return False
            cut_set.add(key);cuts.append(p);return True
        for p in candidate_paths:add_cut(p)
        for value in baselines.values():add_cut(value['induced_comparator'])
        ub=np.ones(nv);ub[ri]=incumbent['regret_sec'];lb=np.zeros(nv)
        integrality=np.zeros(nv);integrality[zstart:ri]=1
        objective=np.zeros(nv);objective[ri]=1
        iterations=[];global_lb=0
        for it in range(max_iterations):
            rr=[];cc=[];vv=[];cut_ub=[]
            for k,p in enumerate(cuts):
                selected=set(p)
                for i in ids:
                    rr.append(k);cc.append(xindex[i]);vv.append(self.lo[i] if i in selected else self.hi[i])
                for c,v in turn_coefs:rr.append(k);cc.append(c);vv.append(v)
                rr.append(k);cc.append(ri);vv.append(-1)
                cut_ub.append(self.cost(p,self.lo))
            matrix=vstack([base,coo_matrix((vv,(rr,cc)),shape=(len(cuts),nv))]).tocsc()
            start=time.perf_counter()
            result=milp(objective,integrality=integrality,bounds=Bounds(lb,ub),
                constraints=LinearConstraint(matrix,np.array(lbs+[-np.inf]*len(cuts)),np.array(ubs+cut_ub)),
                options={'time_limit':time_limit,'mip_rel_gap':0,'presolve':True})
            dual=getattr(result,'mip_dual_bound',None)
            if dual is not None and np.isfinite(dual):global_lb=max(global_lb,math.ceil(float(dual)-1e-6))
            log=dict(iteration=it+1,solver_status=int(result.status),message=result.message,wall_sec=time.perf_counter()-start,
                     master_objective=None if result.fun is None else float(result.fun),dual_bound=None if dual is None else float(dual),
                     mip_gap=None if getattr(result,'mip_gap',None) is None else float(result.mip_gap),cuts=len(cuts))
            iterations.append(log)
            if result.x is None:break
            chosen={k for k in range(len(transitions)) if result.x[zstart+k]>.5}
            next_step={a:(b,k) for k,(a,b,c) in enumerate(transitions) if k in chosen}
            path=[];at=-1;seen=set()
            while at!=-2:
                if at in seen or at not in next_step:raise AssertionError('Invalid master source path')
                seen.add(at);at,k=next_step[at]
                if at>=0:path.append(at)
            evaluated=self.evaluate(path,source,target)
            if (evaluated['regret_sec'],evaluated['upper_sec'],evaluated['path'])<(incumbent['regret_sec'],incumbent['upper_sec'],incumbent['path']):incumbent=evaluated
            ub[ri]=incumbent['regret_sec'];log.update(candidate_regret_sec=evaluated['regret_sec'],incumbent_regret_sec=incumbent['regret_sec'],global_lower_bound_sec=global_lb)
            if global_lb>=incumbent['regret_sec']:break
            if not add_cut(evaluated['induced_comparator']):break
            if result.status!=0:break
        return dict(status='optimal' if global_lb==incumbent['regret_sec'] else 'bounded_feasible',source=source,target=target,
                    incumbent=incumbent,lower_bound_sec=global_lb,absolute_gap_sec=incumbent['regret_sec']-global_lb,
                    baseline=baselines,iterations=iterations,total_wall_sec=time.perf_counter()-started,
                    model_size=dict(kept_road_arcs=n,turn_transitions=len(transitions),variables=nv,binary_variables=len(transitions),
                                    constraints=base.shape[0]+len(cuts)),
                    separation_comparator_paths=cuts,
                    valid_pruning_lower_cost_bound_sec=bound,
                    scope='Global regret optimum/bound within the archived bounded screened graph with shared physical arc uncertainty; not the entire Istanbul street network.')
