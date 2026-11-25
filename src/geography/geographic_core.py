"""Verbatim geographic computations; curated coordinates are loaded separately."""
import itertools, math, time
from geographiclib.geodesic import Geodesic

def distance(a, b):
    return Geodesic.WGS84.Inverse(a[0], a[1], b[0], b[1])['s12'] / 1000

def matrix(nodes, crosschecks=False):
    coords = [(f['crosscheck_latitude'], f['crosscheck_longitude'])
              if crosschecks and f['crosscheck_latitude'] is not None
              else (f['latitude'], f['longitude']) for f in nodes]
    return [[distance(a, b) if i != j else 0.0 for j, b in enumerate(coords)] for i, a in enumerate(coords)]

def assign(d, medoids):
    # IDs are ordered; Python min retains the first medoid for exact distance ties.
    groups = [min(medoids, key=lambda m: (d[i][m], m)) for i in range(len(d))]
    return groups, math.fsum(d[i][m] for i, m in enumerate(groups))

def silhouette(d, labels):
    groups = {label: [i for i, x in enumerate(labels) if x == label] for label in sorted(set(labels))}
    values = []
    for i, label in enumerate(labels):
        peers = [j for j in groups[label] if j != i]
        if not peers:
            values.append(0.0)
            continue
        a = math.fsum(d[i][j] for j in peers) / len(peers)
        b = min(math.fsum(d[i][j] for j in members) / len(members)
                for other, members in groups.items() if other != label)
        values.append((b-a)/max(a,b) if max(a,b) else 0.0)
    return values, math.fsum(values)/len(values)

def solve_all(d, ids):
    results = []
    for k in range(2, min(6, len(d)-1)+1):
        started = time.perf_counter()
        best = math.inf
        count = 0
        tied = 0
        for medoids in itertools.combinations(range(len(d)), k):
            labels, obj = assign(d, medoids)
            count += 1
            if obj < best - 1e-9:
                best, chosen, best_labels, tied = obj, medoids, labels, 1
            elif abs(obj-best) <= 1e-9:
                # Ties resolved by lexicographic medoid tuple, fixed by enumeration.
                tied += 1
        values, score = silhouette(d, best_labels)
        sizes = [best_labels.count(m) for m in chosen]
        results.append(dict(k=k, medoid_ids=[ids[m] for m in chosen],
                            medoid_indices=list(chosen), assigned_medoid_indices=best_labels,
                            total_distance_to_medoid_km=best, mean_distance_to_medoid_km=best/len(d),
                            max_distance_to_medoid_km=max(d[i][m] for i,m in enumerate(best_labels)),
                            ordinary_silhouette_mean=score, point_silhouettes=values,
                            cluster_sizes=sizes, singleton_clusters=sum(n==1 for n in sizes),
                            evaluated_medoid_sets=count, expected_medoid_sets=math.comb(len(d),k),
                            objective_ties_within_1e_9_km=tied,
                            elapsed_seconds=time.perf_counter()-started,
                            optimization_status='global minimum by exhaustive enumeration for this fixed k'))
    selected = min(results, key=lambda r: (-r['ordinary_silhouette_mean'],r['k']))
    return results, selected
