"""Rebuild a bounded, directed, turn-aware research road graph offline.

OSM topology is evidence. Speeds, delay intervals and facility access times are
declared research assumptions, not observed earthquake traffic or site access.
"""
from collections import Counter,defaultdict
import csv,hashlib,json,math,re
from pathlib import Path
from geographiclib.geodesic import Geodesic

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent.parent
BBOX=(40.99,28.765,41.07,28.85)
SPEED={'motorway':60,'motorway_link':30,'trunk':50,'trunk_link':25,
       'primary':35,'primary_link':25,'secondary':30,'secondary_link':20,
       'tertiary':25,'tertiary_link':20,'unclassified':20,'residential':20,
       'living_street':10,'service':10}
FACTOR={'motorway':1.25,'motorway_link':1.5,'trunk':1.5,'trunk_link':1.5,
        'primary':1.5,'primary_link':1.5,'secondary':1.75,'secondary_link':1.75,
        'tertiary':2,'tertiary_link':2,'unclassified':2,'residential':2,
        'living_street':2.5,'service':2.5}

def write_json(name,obj):
    (HERE/name).write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n')

def write_csv(name,rows):
    with (HERE/name).open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

def distance(a,b):return Geodesic.WGS84.Inverse(a[0],a[1],b[0],b[1])['s12']

def vehicle_access(tags):
    value=next((tags[k] for k in ['motorcar','motor_vehicle','vehicle','access'] if k in tags),None)
    return value not in {'no','private','permit','customers','delivery','destination','agricultural','forestry'}

def main():
    raw=(HERE/'sources/road_network.json').read_bytes();osm=json.loads(raw)
    assert 'remark' not in osm,osm.get('remark')
    nodes={n['id']:n for n in osm['elements'] if n['type']=='node'}
    ways={w['id']:w for w in osm['elements'] if w['type']=='way'}
    relations=[r for r in osm['elements'] if r['type']=='relation']
    excluded_complex=set();restrictions=[];relation_audit=[]
    for r in relations:
        t=r.get('tags',{});ms=r['members'];via=[m for m in ms if m['role']=='via']
        fr=[m['ref'] for m in ms if m['role']=='from' and m['type']=='way']
        to=[m['ref'] for m in ms if m['role']=='to' and m['type']=='way']
        if any(k in t for k in ['restriction:conditional','restriction:motorcar:conditional']):valid=False
        else:valid=len(via)==1 and via[0]['type']=='node' and len(fr)==1 and len(to)==1 and t.get('restriction','').startswith(('no_','only_'))
        # Conservatively exclude involved ways for unsupported/malformed relations.
        if not valid:
            involved=[m['ref'] for m in ms if m['type']=='way'];excluded_complex.update(involved)
            relation_audit.append(dict(id=r['id'],action='exclude_involved_ways',ways=involved))
        else:
            restrictions.append(dict(id=r['id'],via=via[0]['ref'],from_way=fr[0],to_way=to[0],restriction=t['restriction']))
            relation_audit.append(dict(id=r['id'],action='enforce_via_node',ways=fr+to))
    blocked=set()
    for nid,n in nodes.items():
        t=n.get('tags',{});bar=t.get('barrier')
        explicitly_open=t.get('motorcar',t.get('motor_vehicle',t.get('access'))) in ('yes','permissive')
        if not vehicle_access(t) or (bar and bar not in ('kerb','cattle_grid') and not explicitly_open):blocked.add(nid)
    def inside(nid):
        n=nodes[nid];return BBOX[0]<=n['lat']<=BBOX[2] and BBOX[1]<=n['lon']<=BBOX[3]
    pieces=[];excluded=Counter()
    for wid,w in sorted(ways.items()):
        t=w.get('tags',{});h=t.get('highway')
        if h not in SPEED:excluded['non_driving_highway']+=1;continue
        if wid in excluded_complex:excluded['complex_or_malformed_restriction']+=1;continue
        if not vehicle_access(t) or any('conditional' in k for k in t):excluded['access_or_conditional']+=1;continue
        if t.get('oneway') not in (None,'yes','no','-1'):excluded['unsupported_oneway']+=1;continue
        piece=[]
        for nid in w['nodes']:
            if nid not in blocked and inside(nid):piece.append(nid)
            else:
                if len(piece)>1:pieces.append((wid,piece,t))
                piece=[]
        if len(piece)>1:pieces.append((wid,piece,t))
    neighbors=defaultdict(set);membership=defaultdict(set);ends=set()
    for wid,p,t in pieces:
        ends.update((p[0],p[-1]))
        for n in p:membership[n].add(wid)
        for a,b in zip(p,p[1:]):neighbors[a].add(b);neighbors[b].add(a)
    # Curated depot record; source provenance is in data/sources.csv.
    depot=json.loads((HERE/'Depo_Kaydi.json').read_text(encoding='utf-8'))
    sites=[depot]+[n for n in json.loads((ROOT/'experiments/geography/Tesis_Agi.json').read_text()) if n['id'] in ['N01','N02','N04','N06','N07','N10']]
    snaps=[]
    for s in sites:
        lat,lon=s['latitude'],s['longitude']
        # Fast planar candidate ordering followed by WGS84 checks, not a new geocode.
        shortlist=sorted(neighbors,key=lambda n:((nodes[n]['lat']-lat)**2+((nodes[n]['lon']-lon)*math.cos(math.radians(lat)))**2))[:12]
        nid=min(shortlist,key=lambda n:distance((lat,lon),(nodes[n]['lat'],nodes[n]['lon'])))
        meters=distance((lat,lon),(nodes[nid]['lat'],nodes[nid]['lon']))
        assert meters<=150,(s['id'],'snap exceeds research-screen limit',meters)
        snaps.append(dict(id=s['id'],name=s['name'],latitude=lat,longitude=lon,osm_node=nid,
                          snap_latitude=nodes[nid]['lat'],snap_longitude=nodes[nid]['lon'],snap_distance_m=round(meters,3),
                          access_sec=60+math.ceil(meters/1.4),access_status='Assumed facility transfer: 60 sec + straight-line offset / 1.4 m/s; not verified access'))
    protected=ends|{n for n in neighbors if len(neighbors[n])!=2 or len(membership[n])>1}|{s['osm_node'] for s in snaps}|{r['via'] for r in restrictions}
    edges=[]
    for wid,p,t in pieces:
        start=0
        for k in range(1,len(p)):
            if p[k] not in protected and k!=len(p)-1:continue
            seq=p[start:k+1];start=k
            if seq[0]==seq[-1]:continue
            length=sum(distance((nodes[a]['lat'],nodes[a]['lon']),(nodes[b]['lat'],nodes[b]['lon'])) for a,b in zip(seq,seq[1:]))
            h=t['highway'];lower=max(1,math.ceil(length*3.6/SPEED[h]));upper=math.ceil(lower*FACTOR[h])
            oneway=t.get('oneway')
            if oneway is None and (t.get('junction')=='roundabout' or h in ('motorway','motorway_link')):oneway='yes'
            orders=[seq] if oneway=='yes' else [seq[::-1]] if oneway=='-1' else [seq,seq[::-1]]
            for order in orders:
                edges.append(dict(id=len(edges),u=order[0],v=order[-1],way=wid,highway=h,name=t.get('name',''),
                                  length_m=round(length,6),lower_sec=lower,upper_sec=upper,
                                  osm_nodes=order,geometry=[[nodes[n]['lon'],nodes[n]['lat']] for n in order]))
    graph=dict(id='KCE_ROAD_2026_11_19',bbox=BBOX,osm_base_timestamp=osm['osm3s']['timestamp_osm_base'],
               attribution='© OpenStreetMap contributors, ODbL 1.0, https://www.openstreetmap.org/copyright',
               edges=edges,restrictions=restrictions,sites=snaps,turn_delay_sec=5,
               immediate_u_turns='excluded everywhere; site arrival/departure resets direction',
               traffic_status='Class speeds and interval multipliers are analyst stress assumptions; no measured disaster traffic')
    write_json('Road_Graph.json',graph);write_json('Depo_Kaydi.json',depot)
    write_csv('Tesis_Yol_Eslesmeleri.csv',snaps)
    write_csv('Yol_Yaylari.csv',[{k:v for k,v in e.items() if k not in ('osm_nodes','geometry')} for e in edges])
    write_json('Yol_Donus_Kisitlari_Denetimi.json',relation_audit)
    report=dict(source_sha256=hashlib.sha256(raw).hexdigest(),osm_base_timestamp=graph['osm_base_timestamp'],
                raw_nodes=len(nodes),raw_ways=len(ways),raw_relations=len(relations),
                graph_nodes=len({e[k] for e in edges for k in ['u','v']}),directed_graph_arcs=len(edges),
                via_node_restrictions=len(restrictions),unsupported_relation_count=len(relations)-len(restrictions),
                conservatively_excluded_restriction_ways=len(excluded_complex),blocked_barrier_or_access_nodes=len(blocked),
                excluded_way_reasons=dict(excluded),snap_max_m=max(s['snap_distance_m'] for s in snaps),
                speed_kmh=SPEED,upper_multiplier=FACTOR,bbox=BBOX,
                scope='Exact only within this bounded screened graph. OSM completeness, actual post-disaster access and facility access unverified.')
    write_json('Yol_Agi_Kurulum_Kaydi.json',report);print(json.dumps(report,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
