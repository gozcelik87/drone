"""Tiny exhaustive oracle plus adversarial route/packaging/resource checks."""
from copy import deepcopy
from itertools import permutations, product
import json
from math import ceil
from pathlib import Path
import unittest

from route_milp import Packaging, check_plan, route_set, solve


def tiny():
    return dict(packaging=dict(filled_bag_kg='1',box_and_coolant_kg='1',units_per_box=2,
                              attachment_kg='0',mission_payload_kg='5',available_box_slots=2,nominal_payload_kg='150'),
                airborne_budget_sec=20,sortie_elapsed_budget_sec=50,thermal_limit_sec=100,
                stop_fixed_sec=1,handover_per_box_sec=1,preparation_sec=2,
                roots=[dict(id='O',fleet=2,preparation_crews=1,root_demand=0,arrival_sec=0,
                            age_at_arrival_sec=0,available_units=5,demands={'A':3,'B':2})],
                arcs={('O','A'):1,('A','O'):1,('O','B'):3,('B','O'):3,('A','B'):2,('B','A'):2})


def oracle(instance):
    """Independent full route/quantity/ordered-flight enumeration, tiny one-root case.

    Uses neither MILP route_set nor check_plan nor Packaging.pack.
    """
    assert len(instance['roots'])==1
    r=instance['roots'][0];ds=r['demands'];p=instance['packaging'];options=[]
    assert len(ds)<=3 and sum(ds.values())<=8 and r['fleet']<=3
    for length in range(1,len(ds)+1):
        for order in permutations(ds,length):
            chain=('O',)+order+('O',);legs=tuple(zip(chain,chain[1:]))
            if any(e not in instance['arcs'] for e in legs):continue
            air=sum(instance['arcs'][e] for e in legs)
            if air>instance['airborne_budget_sec']:continue
            for vals in product(*(range(1,ds[j]+1) for j in order)):
                q=dict(zip(order,vals));boxes=sum(ceil(v/p['units_per_box']) for v in vals)
                mass=float(p['filled_bag_kg'])*sum(vals)+float(p['box_and_coolant_kg'])*boxes+float(p['attachment_kg'])
                if boxes>p['available_box_slots'] or mass>float(p['mission_payload_kg']):continue
                service=instance['stop_fixed_sec']*length+instance['handover_per_box_sec']*boxes
                if air+service>instance['sortie_elapsed_budget_sec']:continue
                duration=air-instance['arcs'][legs[-1]]+service
                options.append((q,duration,air))
    best=None
    for n in range(1,r['fleet']+1):
        for choices in product(options,repeat=n):
            if any(sum(q.get(j,0) for q,_,_ in choices)!=d for j,d in ds.items()):continue
            finish=[instance['preparation_sec']*(1+k//r['preparation_crews'])+x[1] for k,x in enumerate(choices)]
            if max(finish)+r['age_at_arrival_sec']>instance['thermal_limit_sec']:continue
            key=(max(finish)+r['arrival_sec'],n,sum(x[2] for x in choices))
            if best is None or key<best:best=key
    return best


class ScientificModelChecks(unittest.TestCase):
    def test_matches_independent_exhaustive_oracle(self):
        i=tiny();expected=oracle(i);result=solve(i)
        self.assertEqual(result['status'],'lexicographic_optimal')
        self.assertEqual(tuple(result['metrics'][k] for k in ['last_delivery_sec','drones_used','total_airborne_sec']),expected)

    def test_oracle_with_parallel_crews(self):
        i=tiny();i['roots'][0]['preparation_crews']=2
        result=solve(i)
        self.assertEqual(tuple(result['metrics'][k] for k in ['last_delivery_sec','drones_used','total_airborne_sec']),oracle(i))

    def test_capacity_and_crew_bounds_over_tiny_grid(self):
        for da,db,crews in product([1,2,3],[1,2],[1,2]):
            i=tiny();i['roots'][0].update(demands={'A':da,'B':db},available_units=da+db,preparation_crews=crews)
            expected=oracle(i);r=solve(i)
            self.assertEqual(r['status'],'lexicographic_optimal')
            self.assertEqual(tuple(r['metrics'][k] for k in ['last_delivery_sec','drones_used','total_airborne_sec']),expected)
            self.assertTrue(all(x<=expected[0] for x in r['analytical_delivery_lower_bounds_sec'].values()))

    def test_no_return_leg_no_route(self):
        i=tiny();del i['arcs']['A','O'];del i['arcs']['B','O']
        self.assertEqual(solve(i)['status'],'proven_infeasible_solver')

    def test_return_included_in_endurance(self):
        i=tiny();i['roots'][0]['demands']={'A':1};i['arcs']['A','O']=30
        self.assertEqual(solve(i)['status'],'proven_infeasible_solver')

    def test_disconnected_cycle_cannot_serve(self):
        i=tiny();i['arcs']={('A','B'):1,('B','A'):1}
        self.assertEqual(solve(i)['status'],'proven_infeasible_solver')

    def test_single_drone_split_delivery_capacity(self):
        i=tiny();i['roots'][0]['fleet']=1
        self.assertIsNone(oracle(i))
        self.assertEqual(solve(i)['status'],'proven_infeasible_precheck')

    def test_exact_delivery_checker_rejects_oversupply(self):
        i=tiny();result=solve(i);p=deepcopy(result['plan'])
        j=p[0]['stops'][0];p[0]['quantities'][j]+=1
        with self.assertRaises(ValueError):check_plan(i,p)

    def test_sealed_recipient_boxes_prevent_false_capacity(self):
        i=tiny();i['packaging'].update(units_per_box=12,mission_payload_kg='50')
        i['roots'][0].update(fleet=1,demands={'A':1,'B':1,'C':1},available_units=3)
        self.assertEqual(solve(i)['status'],'proven_infeasible_precheck')

    def test_ground_handover_needs_no_drone(self):
        i=tiny();i['roots'][0].update(fleet=0,demands={},root_demand=5,arrival_sec=7)
        r=solve(i)['metrics'];self.assertEqual((r['last_delivery_sec'],r['drones_used']),(11,0))
        self.assertIsNone(r['last_drone_return_sec'])

    def test_preparation_and_prior_age_count_in_cold_chain(self):
        i=tiny();i['thermal_limit_sec']=9;i['roots'][0]['age_at_arrival_sec']=5
        self.assertIsNone(oracle(i));self.assertEqual(solve(i)['status'],'proven_infeasible_solver')

    def test_ground_cold_chain_is_checked(self):
        i=tiny();i['roots'][0].update(fleet=0,demands={},root_demand=5,age_at_arrival_sec=99)
        self.assertEqual(solve(i)['status'],'proven_infeasible_precheck')

    def test_landed_service_counts_in_elapsed_mission_limit(self):
        i=tiny();i['sortie_elapsed_budget_sec']=2
        self.assertEqual(solve(i)['status'],'proven_infeasible_solver')

    def test_direct_policy_cannot_deliver_to_two_sites_on_one_drone(self):
        i=tiny();i['roots'][0].update(fleet=1,demands={'A':1,'B':1},available_units=2)
        self.assertTrue(solve(i,'direct')['status'].startswith('proven_infeasible_'))
        self.assertEqual(solve(i,'all')['status'],'lexicographic_optimal')

    def test_ordering_policy_is_nested(self):
        i=tiny();p=Packaging(**i['packaging']);r=i['roots'][0]
        sets={mode:{x.stops for x in route_set('O',r['demands'],i['arcs'],p,20,mode)}
              for mode in ['all','nearest_neighbor','direct']}
        self.assertTrue(sets['direct']<=sets['nearest_neighbor']<=sets['all'])

    def test_missing_mission_budget_rejected(self):
        i=tiny();i['airborne_budget_sec']=None
        with self.assertRaises(ValueError):solve(i)

    def test_physical_drone_cannot_be_used_twice(self):
        i=tiny();p=solve(i)['plan'];p[1]['slot']=p[0]['slot']
        with self.assertRaises(ValueError):check_plan(i,p)

    def test_post_arrival_clock_separates_delivery_and_return(self):
        i=tiny();i['roots'][0]['demands']={'A':1};i['arcs']['A','O']=9
        r=solve(i)['metrics'];self.assertEqual(r['last_drone_return_sec']-r['last_delivery_sec'],9)


if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ScientificModelChecks))
    record=dict(tests_run=result.testsRun,errors=len(result.errors),failures=len(result.failures),
                all_passed=result.wasSuccessful(),scope='Independent tiny-instance oracle and model semantics; not operational validation.')
    (Path(__file__).resolve().parent/'Model_Dogrulama_Kaydi.json').write_text(json.dumps(record,indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
