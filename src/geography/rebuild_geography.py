"""Recompute geography from the recorded 16-site curated input."""
import csv, json
from pathlib import Path
from geographic_core import matrix, solve_all
HERE=Path(__file__).resolve().parent
def save_json(name,data):
    (HERE/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def save_csv(name,rows):
    with (HERE/name).open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def main():
    nodes=json.loads((HERE/'Tesis_Agi.json').read_text(encoding='utf-8'))
    d = matrix(nodes)
    ids = [f['id'] for f in nodes]
    save_csv('Mesafe_Matrisi_km.csv', [dict([('from_node',ids[i])]+[(ids[j],format(v,'.9f')) for j,v in enumerate(row)]) for i,row in enumerate(d)])
    save_csv('Olası_Hava_Bacaklari.csv', [dict(from_node=ids[i],to_node=ids[j],geodesic_km=format(d[i][j],'.9f'),
        flight_allowed='',actual_flight_km='',flight_min='',road_travel_min='',
        status='geometric_pair_only_permission_and_operational_feasibility_unknown')
        for i in range(len(d)) for j in range(len(d)) if i!=j])
    all_results, chosen = solve_all(d, ids)
    cross_results, cross_chosen = solve_all(matrix(nodes,True),ids)
    core = [f for f in nodes if f['tier']=='A']
    core_results, core_chosen = solve_all(matrix(core),[f['id'] for f in core])
    save_json('Kume_Analizi.json', {'main_network':all_results,'selected_k':chosen['k'],
        'selection_rule':'highest ordinary mean silhouette among the distance-minimizing solutions for k=2..6; smaller k on exact score tie',
        'scope':'descriptive geography only; not routing-, fleet- or demand-optimal k',
        'crosscheck_coordinate_sensitivity':cross_results,'crosscheck_selected_k':cross_chosen['k'],
        'tier_A_subset_sensitivity':core_results,'tier_A_subset_selected_k':core_chosen['k'],
        'tier_A_subset_note':'Different node set; geographic selection sensitivity, not a like-for-like service-performance comparison.'})
    save_csv('Kume_Sayisi_Karsilastirmasi.csv', [dict(k=r['k'],medoid_ids=';'.join(r['medoid_ids']),
        cluster_sizes=';'.join(map(str,r['cluster_sizes'])),objective_km=r['total_distance_to_medoid_km'],
        silhouette=r['ordinary_silhouette_mean'],max_radius_km=r['max_distance_to_medoid_km'],
        singleton_clusters=r['singleton_clusters'],evaluated_sets=r['evaluated_medoid_sets'],
        optimality_gap=0,elapsed_seconds=r['elapsed_seconds']) for r in all_results])
    groups=[]
    for i,f in enumerate(nodes):
        m=chosen['assigned_medoid_indices'][i]
        groups.append(dict(id=f['id'],name=f['name'],cluster_id='C'+str(chosen['medoid_indices'].index(m)+1),
            medoid_id=ids[m],is_geographic_medoid=int(i==m),distance_to_medoid_km=d[i][m],
            silhouette=chosen['point_silhouettes'][i],operational_root_eligible='unknown'))
    save_csv('Kume_Uyelikleri.csv', groups)
    print(json.dumps(dict(sites=len(nodes),selected_k=chosen['k'],medoids=chosen['medoid_ids'],source_extraction_repeated=False)))
if __name__=='__main__':main()
