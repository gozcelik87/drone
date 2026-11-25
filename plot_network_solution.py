"""Plot the recorded reference road paths and drone dispatch without optimization."""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
from zipfile import ZipFile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
from matplotlib.ticker import FixedLocator, FormatStrFormatter

ROOT = Path(__file__).resolve().parent
BLUE = '#176782'
ORANGE = '#B65C24'
DARK = '#203342'
GREY = '#60717B'


def inputs(run):
    graph_path = run/'experiments/linked/Road_Graph.json'
    graph = json.loads(graph_path.read_text())
    physical = {k:v for k,v in graph.items() if k != 'id'}
    canonical = hashlib.sha256(json.dumps(physical, sort_keys=True, ensure_ascii=False).encode()).hexdigest()
    expected = json.loads((ROOT/'expected/input_checks.json').read_text())
    assert canonical == expected['experiments/linked/Road_Graph.json'], 'Road graph differs from reference input.'
    with ZipFile(ROOT/'data/inputs.zip') as z:
        cases = json.loads(z.read('experiments/ground/inputs/Linked_Cases.json'))
        reference = next(c for c in cases if c['case_id'] == 'reference')
        routes = {rid:json.loads(z.read(f'experiments/linked/Regret_{rid}.json')) for rid in ['N04','N10']}
    return graph, reference, routes


def verify_solution(graph, reference, routes):
    raw = reference['realizations']['upper']['input']
    plan = reference['hybrid_plan']
    spec = importlib.util.spec_from_file_location('event_check',ROOT/'src/preparation/independent_checks.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    metrics, flights = module.evaluate(raw,plan)
    saved = reference['realizations']['upper']['hybrid_metrics']
    assert metrics == {k:v for k,v in saved.items() if k != 'flights'}
    assert (metrics['last_delivery_sec'],metrics['drones_used']) == (4287,12)
    assert raw['preparation_sec'] == 180 and raw['packaging']['available_box_slots'] == 2
    assert all(r['preparation_crews'] == 1 for r in raw['roots'])
    expected_flights = {(f['root'],f['slot']):f for f in saved['flights']}
    delivered, latest, groups = Counter(), {}, Counter()
    for f,p in zip(flights,sorted(plan,key=lambda x:(x['root'],x['slot']))):
        old = expected_flights[f['root'],f['slot']]
        assert (f['launch_sec'],f['return_sec'],f['handovers']) == (old['launch_sec'],old['return_sec'],old['handovers'])
        assert len(p['stops']) == 1, 'This figure groups the reference single-receiver sorties only.'
        groups[f['root'],p['stops'][0]] += 1
        for event in f['handovers']:
            node = event['node'];delivered[node] += event['units']
            latest[node] = max(latest.get(node,0),event['delivery_sec'])
    root_rows = {}
    for root in raw['roots']:
        quantity = root['root_demand']
        local_time = None
        if quantity:
            local_time = root['arrival_sec'] + raw['stop_fixed_sec'] + raw['handover_per_box_sec']*math.ceil(quantity/raw['packaging']['units_per_box'])
            delivered[root['id']] += quantity;latest[root['id']] = local_time
        root_rows[root['id']] = {'release_sec':root['arrival_sec'],'local_units':quantity,'local_handover_sec':local_time}
    assert sum(delivered.values()) == 351 and sum(root_rows[r]['local_units'] for r in root_rows) == 70
    assert all(n == 3 for n in groups.values()) and len(groups) == 4
    assert max(latest.values()) == 4287 and latest['N02'] == 4287
    edges = {e['id']:e for e in graph['edges']};sites={s['id']:s for s in graph['sites']}
    road_rows = {}
    for rid,route in routes.items():
        ids=route['incumbent']['path'];path=[edges[i] for i in ids]
        assert path[0]['u'] == sites['D01']['osm_node'] and path[-1]['v'] == sites[rid]['osm_node']
        assert all(a['v']==b['u'] for a,b in zip(path,path[1:]))
        upper=sum(e['upper_sec'] for e in path)+sum(graph['turn_delay_sec'] for a,b in zip(path,path[1:]) if a['way']!=b['way'])
        assert upper == route['incumbent']['upper_sec']
        assert 300+sites['D01']['access_sec']+upper+sites[rid]['access_sec']+120 == root_rows[rid]['release_sec']
        road_rows[rid]={'edge_ids':ids,'upper_road_sec':upper}
    return {'scenario':'reference','road_realization':'upper','batch_units':351,
            'root_events':root_rows,
            'receivers':{n:{'total_units':delivered[n],'last_handover_sec':latest[n]} for n in sorted(delivered)},
            'route_groups':[{'root':a,'receiver':b,'sorties':n} for (a,b),n in sorted(groups.items())],
            'drone_flights':flights,'road_paths':road_rows,'metrics':metrics,
            'plot_semantics':'Panel b is a schematic. Repeated routes are grouped; every sortie uses a different drone. Receiver times denote completion of all assigned handovers.',
            'checks':{'independent_events_match_saved_solution':True,'all_quantities_accounted_for':True,
                      'road_paths_and_root_releases_match':True,'no_optimization_performed':True}}


def draw(graph, evidence, output):
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':14,'axes.labelsize':14,
                         'axes.titlesize':16,'xtick.labelsize':12,'ytick.labelsize':12,
                         'svg.fonttype':'none','pdf.fonttype':42,'savefig.facecolor':'white'})
    fig=plt.figure(figsize=(12.2,8.4),facecolor='white')
    ax=fig.add_axes([.075,.17,.385,.69])
    bx=fig.add_axes([.50,.17,.48,.69])
    fig.text(.075,.963,'Reference solution at upper road times',fontsize=20,fontweight='bold',color=DARK)
    fig.text(.075,.923,'351 RBC units  ·  2 box positions per drone  ·  1 preparation crew per launch site  ·  3 min preparation per sortie',fontsize=13,color=GREY)
    fig.text(.075,.876,'(a) Vehicle access · Stage III',fontsize=16,fontweight='bold',color=DARK)
    fig.text(.505,.876,'(b) Drone dispatch · Stages IV–V',fontsize=16,fontweight='bold',color=DARK)

    segments=[];seen=set();edges={e['id']:e for e in graph['edges']}
    for e in graph['edges']:
        key=(e['way'],min(e['u'],e['v']),max(e['u'],e['v']))
        if key not in seen:segments.append(e['geometry']);seen.add(key)
    ax.add_collection(LineCollection(segments,colors='#D7DDE0',linewidths=.32,zorder=1,rasterized=True))
    for rid,color in [('N04',BLUE),('N10',ORANGE)]:
        lines=[edges[i]['geometry'] for i in evidence['road_paths'][rid]['edge_ids']]
        ax.add_collection(LineCollection(lines,colors=color,linewidths=2.8,zorder=3,label='D01 → '+rid))
    offsets={'D01':(7,8),'N01':(-37,-15),'N04':(7,7),'N02':(8,-15),'N06':(8,6),'N07':(8,3),'N10':(-37,7)}
    for site in graph['sites']:
        depot=site['id']=='D01';root=site['id'] in ('N04','N10')
        ax.scatter(site['longitude'],site['latitude'],s=90 if depot else 62,
                   marker='s' if depot else '^' if root else 'o',
                   c='#891F2B' if depot else DARK,zorder=5)
        ax.annotate(site['id'],(site['longitude'],site['latitude']),xytext=offsets[site['id']],
                    textcoords='offset points',fontsize=12,fontweight='bold',color=DARK)
    ax.set_xlim(28.765,28.85);ax.set_ylim(40.99,41.07)
    ax.set_aspect(1/math.cos(math.radians(41.03)))
    ax.set_xlabel('Longitude');ax.set_ylabel('Latitude')
    ax.xaxis.set_major_locator(FixedLocator([28.77,28.79,28.81,28.83,28.85]))
    ax.yaxis.set_major_locator(FixedLocator([40.99,41.01,41.03,41.05,41.07]))
    ax.xaxis.set_major_formatter(FormatStrFormatter('%.2f'));ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    ax.legend(loc='upper left',fontsize=12,frameon=True,facecolor='white',edgecolor='#D7DDE0')
    for spine in ax.spines.values():spine.set_color('#A7B3BA')

    bx.set_xlim(0,1);bx.set_ylim(0,1);bx.axis('off')
    for bottom,height,color in [(.645,.315,BLUE),(.045,.56,ORANGE)]:
        bx.add_patch(FancyBboxPatch((.025,bottom),.95,height,boxstyle='round,pad=0.012,rounding_size=0.02',
                                    facecolor=color,edgecolor='none',alpha=.045,zorder=0))
    positions={'N04':(.20,.83),'N01':(.78,.83),'N10':(.20,.28),
               'N06':(.78,.53),'N07':(.78,.32),'N02':(.78,.12)}
    for group in evidence['route_groups']:
        rid,node=group['root'],group['receiver'];a,b=positions[rid],positions[node]
        color=BLUE if rid=='N04' else ORANGE
        bx.add_patch(FancyArrowPatch(a,b,connectionstyle='arc3,rad=-0.07',arrowstyle='-|>',
                                    mutation_scale=18,linewidth=2.2,color=color,shrinkA=10,shrinkB=10,zorder=2))
        return_curve='arc3,rad=0.26' if node=='N06' else 'arc3,rad=-0.07'
        bx.add_patch(FancyArrowPatch(b,a,connectionstyle=return_curve,arrowstyle='-|>',
                                    mutation_scale=16,linewidth=1.8,color=color,linestyle=(0,(4,3)),
                                    shrinkA=10,shrinkB=10,alpha=.8,zorder=2))
        mx=(a[0]+b[0])/2;my=(a[1]+b[1])/2
        bx.text(mx,my+.028,'×3',fontsize=13,fontweight='bold',ha='center',va='center',color=color,
                bbox={'boxstyle':'round,pad=0.18','fc':'white','ec':'none'},zorder=4)
    for rid in ['N04','N10']:
        x,y=positions[rid];r=evidence['root_events'][rid];color=BLUE if rid=='N04' else ORANGE
        bx.scatter(x,y,marker='^',s=140,c=color,edgecolor='white',linewidth=.8,zorder=5)
        bx.text(x,y+.048,rid,ha='center',fontsize=16,fontweight='bold',color=color)
        release_offset=.058 if rid=='N04' else .115
        bx.text(x,y-release_offset,f"Release {r['release_sec']/60:.2f} min",ha='center',va='top',fontsize=13.5,color=DARK)
        if r['local_units']:
            bx.text(x,y-.105,f"{r['local_units']} units locally\n{r['local_handover_sec']/60:.2f} min",ha='center',va='top',fontsize=13.5,color=DARK,linespacing=1.25)
        else:
            bx.text(x,y-.165,'No local demand',ha='center',va='top',fontsize=12,color=GREY)
    for node in ['N01','N06','N07','N02']:
        x,y=positions[node];r=evidence['receivers'][node];color=BLUE if node=='N01' else ORANGE
        bx.scatter(x,y,s=100,c='white',edgecolors=color,linewidths=2,zorder=5)
        bx.text(x,y+.046,node,ha='center',fontsize=16,fontweight='bold',color=DARK)
        label_offset=.085 if node=='N06' else .048
        bx.text(x,y-label_offset,f"{r['total_units']} units · {r['last_handover_sec']/60:.2f} min",ha='center',va='top',
                fontsize=14.5,fontweight='bold' if node=='N02' else 'normal',color=DARK)
    bx.text(.52,.995,'Receiver labels: total quantity · final handover',ha='center',va='top',fontsize=12,color=GREY)

    # Shared, compact legend and timing definitions.
    handles=[Line2D([0],[0],color=ORANGE,lw=2.2,label='Delivery leg'),
             Line2D([0],[0],color=ORANGE,lw=1.8,ls=(0,(4,3)),label='Return leg')]
    fig.legend(handles=handles,loc='lower left',bbox_to_anchor=(.505,.105),frameon=False,
               ncol=2,fontsize=12.5,columnspacing=1.8,handlelength=2.2)
    fig.text(.505,.083,'×3: three sorties flown by three different drones',fontsize=11.5,color=GREY)
    fig.text(.075,.047,'All times start at depot batch readiness. Root release follows unloading; panel (b) is schematic.',fontsize=11.5,color=GREY)
    fig.text(.075,.017,'Road data: © OpenStreetMap contributors · ODbL · openstreetmap.org/copyright',fontsize=10.5,color=GREY)
    output.mkdir(parents=True,exist_ok=True)
    base=output/'Figure_2_Network_Solution'
    fig.savefig(base.with_suffix('.png'),dpi=450,metadata={'Software':'Matplotlib','Description':'Reference vehicle access and drone dispatch.'})
    fig.savefig(base.with_suffix('.pdf'),dpi=300,metadata={'Title':'Reference vehicle access and drone dispatch','Author':'','Creator':'Matplotlib','CreationDate':None,'ModDate':None})
    fig.savefig(base.with_suffix('.svg'),dpi=300,metadata={'Title':'Reference vehicle access and drone dispatch','Date':None,'Creator':'Matplotlib'})
    fig.savefig(output/'Figure_2_Preview.png',dpi=140)
    plt.close(fig)


def main():
    if sys.flags.optimize:
        raise RuntimeError('Run without -O: solution checks require assertions.')
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True,help='Completed reproduce.py check or recompute directory')
    parser.add_argument('--output-dir',type=Path,default=ROOT/'figure_outputs')
    args=parser.parse_args()
    graph,reference,routes=inputs(args.run_dir)
    evidence=verify_solution(graph,reference,routes)
    draw(graph,evidence,args.output_dir)
    (args.output_dir/'Figure_2_Data_and_Checks.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'figure':'Figure 2','batch_units':351,'drones':12,
                      'last_delivery_min':evidence['metrics']['last_delivery_sec']/60,
                      'checks':evidence['checks']},indent=2))


if __name__=='__main__':main()
