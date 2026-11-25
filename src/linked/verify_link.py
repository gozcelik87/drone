"""Independent interval-corner oracle and linked load/clock checks."""
from copy import deepcopy
import importlib.util,itertools,json,math,sys,unittest
from pathlib import Path
import numpy as np
from road_core import Roads
import linked_drone

HERE=Path(__file__).resolve().parent
sys.modules['route_milp']=linked_drone
spec=importlib.util.spec_from_file_location('drone_checks',HERE.parent/'drone/verify_milp.py')
drone_checks=importlib.util.module_from_spec(spec);spec.loader.exec_module(drone_checks)

def toy(bounds=None):
    pairs=[(0,1),(1,3),(0,2),(2,3),(1,2)]
    bounds=bounds or [(2,8),(2,3),(3,5),(3,6),(1,2)]
    return dict(edges=[dict(id=i,u=a,v=b,way=i,length_m=1,lower_sec=l,upper_sec=h) for i,((a,b),(l,h)) in enumerate(zip(pairs,bounds))],
                restrictions=[],sites=[],turn_delay_sec=0)

PATHS=[[0,1],[2,3],[0,4,3]]
def corner_oracle(g,paths=PATHS):
    # Explicit paths and all 2^m endpoint scenarios; does not use road_core.
    worst=[0]*len(paths)
    for bits in itertools.product([0,1],repeat=len(g['edges'])):
        values=[e['upper_sec'] if bits[i] else e['lower_sec'] for i,e in enumerate(g['edges'])]
        totals=[sum(values[i] for i in p)+g['turn_delay_sec']*(len(p)-1) for p in paths]
        for k,v in enumerate(totals):worst[k]=max(worst[k],v-min(totals))
    return worst

class RoadAndLinkChecks(unittest.TestCase):
    def test_induced_scenario_matches_all_interval_corners(self):
        g=toy();r=Roads(g)
        self.assertEqual([r.evaluate(p,0,3)['regret_sec'] for p in PATHS],corner_oracle(g))

    def test_regret_master_matches_corner_oracle_grid(self):
        for a,b,c in itertools.product([0,3],[0,2],[0,4]):
            g=toy([(2,2+a),(2,4),(3,3+b),(3,3+c),(1,2)])
            x=Roads(g).solve_regret(0,3,time_limit=5)
            self.assertEqual(x['status'],'optimal');self.assertEqual(x['incumbent']['regret_sec'],min(corner_oracle(g)))

    def test_shared_segment_uncertainty_cancels(self):
        g=toy();g['edges'].append(dict(id=5,u=4,v=0,way=5,length_m=1,lower_sec=1,upper_sec=100))
        paths=[[5]+p for p in PATHS];r=Roads(g)
        self.assertEqual([r.evaluate(p,4,3)['regret_sec'] for p in paths],corner_oracle(toy()))

    def test_forbidden_turn_changes_shortest_path(self):
        g=toy();g['restrictions']=[dict(id=1,via=1,from_way=0,to_way=1,restriction='no_left_turn')]
        p,c=Roads(g).shortest(0,3)
        self.assertEqual(c,6);self.assertNotEqual(p,[0,1])

    def test_only_turn_excludes_other_exit(self):
        g=toy();g['restrictions']=[dict(id=1,via=1,from_way=0,to_way=4,restriction='only_right_turn')]
        r=Roads(g)
        with self.assertRaises(ValueError):r.validate([0,1],0,3)
        r.validate([0,4,3],0,3)

    def test_oneway_not_made_bidirectional(self):
        self.assertIsNone(Roads(toy()).shortest(3,0)[0])

    def test_closure_is_absent_not_large_finite_cost(self):
        r=Roads(toy(),closed_ways=[0,2]);self.assertEqual(r.solve_regret(0,3)['status'],'unreachable')

    def test_turn_delay_charged_once(self):
        g=toy();g['turn_delay_sec']=5;r=Roads(g)
        self.assertEqual(r.cost([0,1],r.lo),9)
        self.assertEqual([r.evaluate(p,0,3)['regret_sec'] for p in PATHS],corner_oracle(g))

    def test_vehicle_total_boxes_not_per_drone_only(self):
        i=drone_checks.tiny();i['roots'][0].update(vehicle_box_slots=2,vehicle_component_payload_kg='100')
        self.assertEqual(linked_drone.solve(i)['status'],'proven_infeasible_precheck')

    def test_vehicle_mass_includes_all_box_tare(self):
        i=drone_checks.tiny();i['roots'][0].update(vehicle_box_slots=10,vehicle_component_payload_kg='7')
        self.assertEqual(linked_drone.solve(i)['status'],'proven_infeasible_precheck')

    def test_vehicle_box_constraint_is_in_allocator(self):
        i=drone_checks.tiny();i['roots'][0].update(vehicle_box_slots=3,vehicle_component_payload_kg='8')
        r=linked_drone.solve(i);self.assertEqual(r['status'],'lexicographic_optimal')
        self.assertEqual(r['metrics']['vehicle_loads'][0]['boxes'],3)
        self.assertEqual(r['metrics']['vehicle_loads'][0]['component_mass_kg'],8)

    def test_checker_rejects_small_vehicle_on_otherwise_valid_plan(self):
        i=drone_checks.tiny();p=linked_drone.solve(i)['plan'];i['roots'][0]['vehicle_box_slots']=1
        with self.assertRaises(ValueError):linked_drone.check_plan(i,p)

    def test_no_thermal_clock_reset_at_root(self):
        i=drone_checks.tiny();i['roots'][0].update(arrival_sec=95,age_at_arrival_sec=95)
        self.assertEqual(linked_drone.solve(i)['status'],'proven_infeasible_solver')

    def test_fixed_plan_earlier_arrivals_preserve_packing_and_advance_clock(self):
        i=drone_checks.tiny();i['roots'][0].update(arrival_sec=30,age_at_arrival_sec=30)
        x=linked_drone.solve(i);j=deepcopy(i);j['roots'][0].update(arrival_sec=10,age_at_arrival_sec=10)
        m=linked_drone.check_plan(j,x['plan'])
        self.assertEqual(m['last_delivery_sec'],x['metrics']['last_delivery_sec']-20)
        self.assertEqual(m['vehicle_loads'],x['metrics']['vehicle_loads'])

    def test_archived_graph_preserves_oneway_and_exclusions(self):
        g=json.loads((HERE/'Road_Graph.json').read_text());raw=json.loads((HERE/'sources/road_network.json').read_text())
        ways={x['id']:x for x in raw['elements'] if x['type']=='way'}
        banned={w for r in json.loads((HERE/'Yol_Donus_Kisitlari_Denetimi.json').read_text()) if r['action']=='exclude_involved_ways' for w in r['ways']}
        for e in g['edges']:
            w=ways[e['way']];self.assertNotIn(e['way'],banned)
            original=w['nodes'];p=e['osm_nodes'];tag=w['tags'].get('oneway')
            if tag is None and (w['tags'].get('junction')=='roundabout' or w['tags'].get('highway') in ('motorway','motorway_link')):tag='yes'
            if tag=='yes':self.assertTrue(any(original[i:i+len(p)]==p for i in range(len(original)-len(p)+1)))
            if tag=='-1':self.assertTrue(any(original[i:i+len(p)]==p[::-1] for i in range(len(original)-len(p)+1)))
            self.assertLessEqual(e['lower_sec'],e['upper_sec']);self.assertGreater(e['lower_sec'],0)
        self.assertTrue(all(s['snap_distance_m']<=150 for s in g['sites']))

if __name__=='__main__':
    suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(drone_checks.ScientificModelChecks),
                             unittest.defaultTestLoader.loadTestsFromTestCase(RoadAndLinkChecks)])
    r=unittest.TextTestRunner(verbosity=2).run(suite)
    (HERE/'Model_Dogrulama_Kaydi.json').write_text(json.dumps(dict(tests_run=r.testsRun,failures=len(r.failures),errors=len(r.errors),
        all_passed=r.wasSuccessful(),scope='Includes 18 drone-model checks, independent interval-corner checks, aggregate vehicle load, turn/access preprocessing and linked thermal/event clocks; not operational validation.'),indent=2)+'\n')
    raise SystemExit(0 if r.wasSuccessful() else 1)
