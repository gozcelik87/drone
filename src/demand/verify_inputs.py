"""Independent arithmetic and adversarial boundary checks for research inputs."""
import json
from pathlib import Path
import unittest
from decimal import Decimal
from demand_payload import (Packaging, apportion, conditional_demand,
                            net_replenishment, thermal_age_at_delivery)


class ScientificInputChecks(unittest.TestCase):
    def test_hand_computed_severity_conversion(self):
        # 100*.5*.2*2 + 10*1*.5*4 = 40 units; selected half = 20.
        r = conditional_demand({'L2': 100, 'L3': 10}, {'L2': '.5', 'L3': 1},
                               {'L2': '.2', 'L3': '.5'}, {'L2': 2, 'L3': 4}, '.5')
        self.assertEqual((r['district_gross_units'], r['network_gross_units']), (40, 20))

    def test_reference_case_without_binary_rounding_drift(self):
        r = conditional_demand({'L2': 4079, 'L3': 925}, {'L2': 1, 'L3': 1},
                               {'L2': '.07', 'L3': '.07'}, {'L2': 4, 'L3': 4}, '.25')
        self.assertEqual(r['network_expected_units'], Decimal('350.2800'))
        self.assertEqual(r['network_gross_units'], 351)
        self.assertEqual(r['district_gross_units'], 1402)
        self.assertEqual(r['outside_network_units'], 1051)

    def test_missing_clinical_parameter_rejected(self):
        with self.assertRaises(ValueError):
            conditional_demand({'L2': 10}, {'L2': 1}, {'L2': None}, {'L2': 2}, 1)

    def test_invalid_fraction_and_fatality_rejected(self):
        for level, rate in [('L2', 1.1), ('L4', .1)]:
            with self.assertRaises(ValueError):
                conditional_demand({level: 10}, {level: 1}, {level: rate}, {level: 2}, 1)

    def test_apportion_small_total_and_zero_weight(self):
        self.assertEqual(apportion(2, {'C': 1, 'A': 1, 'B': 1, 'X': 0}),
                         {'A': 1, 'B': 1, 'C': 0, 'X': 0})

    def test_351_exactly_allocated(self):
        self.assertEqual(apportion(351, dict.fromkeys(['N01','N02','N04','N06','N07'], 1)),
                         {'N01': 71, 'N02': 70, 'N04': 70, 'N06': 70, 'N07': 70})

    def test_no_receiver_cannot_absorb_positive_demand(self):
        for weights in [{}, {'N01': 0}]:
            with self.assertRaises(ValueError):
                apportion(1, weights)
        self.assertEqual(apportion(0, {}), {})

    def test_stock_cannot_create_negative_transport(self):
        self.assertEqual(net_replenishment({'A': 5, 'B': 7}, {'A': 9, 'B': 2}, {'A': 0, 'B': 3}),
                         {'A': 0, 'B': 2})

    def test_stock_must_be_integer_and_explicit(self):
        for stock in [{}, {'A': None}, {'A': 2.5}]:
            with self.assertRaises(ValueError):
                net_replenishment({'A': 5}, stock, {'A': 0})

    def test_recipient_fragmentation_changes_feasibility(self):
        p = Packaging('.35', '8.45', 12, 2, 50, 2)
        a, b = p.pack({'A': 24}), p.pack({'A': 8, 'B': 8, 'C': 8})
        self.assertEqual((a['launch_mass_kg'], b['launch_mass_kg']), (Decimal('27.30'), Decimal('35.75')))
        self.assertTrue(a['feasible']); self.assertFalse(b['feasible'])
        self.assertTrue(b['mass_feasible']); self.assertFalse(b['slots_feasible'])

    def test_exact_mass_boundary(self):
        p = Packaging('.35', '8.45', 12, 2, '14.65', 1)
        self.assertTrue(p.pack({'A': 12})['feasible'])
        self.assertFalse(Packaging('.35', '8.45', 12, 2, '14.64', 1).pack({'A': 12})['feasible'])

    def test_partial_box_still_has_full_tare(self):
        p = Packaging('.35', '8.45', 12, 2, 50, 4)
        self.assertEqual(p.one_recipient_capacity(), 40)
        self.assertEqual(p.pack({'A': 40})['launch_mass_kg'], Decimal('49.80'))
        self.assertFalse(p.pack({'A': 41})['mass_feasible'])

    def test_zero_slots_and_payload_exhausted(self):
        self.assertEqual(Packaging('.35', '8.45', 12, 2, 150, 0).one_recipient_capacity(), 0)
        self.assertEqual(Packaging('.35', '8.45', 12, 2, 2, 1).one_recipient_capacity(), 0)

    def test_nominal_limit_cannot_be_exceeded(self):
        with self.assertRaises(ValueError):
            Packaging('.35', '8.45', 12, 2, 151, 1).validate()

    def test_payload_capacity_does_not_imply_bag_count(self):
        self.assertEqual(Packaging('.35', '8.45', 12, 2, 150, 2).one_recipient_capacity(), 24)

    def test_nonfinite_and_negative_mass_rejected(self):
        for m in [None, 'NaN', 'Infinity', -1, 0]:
            with self.assertRaises(ValueError):
                Packaging(m, '8.45', 12, 2, 150, 2).validate()

    def test_thermal_age_includes_ground_exposure(self):
        self.assertEqual(thermal_age_at_delivery(20, 30, 5, 10, 65)['age_min'], 65)
        self.assertTrue(thermal_age_at_delivery(20, 30, 5, 10, 65)['feasible'])
        self.assertFalse(thermal_age_at_delivery(20, 30, 5, 10, 64)['feasible'])
        with self.assertRaises(ValueError):
            thermal_age_at_delivery(0, 0, 0, 1, None)

    def test_generated_instances_conserve_units(self):
        import csv
        here = Path(__file__).resolve().parent
        with (here/'Talep_Senaryolari.csv').open(encoding='utf-8-sig') as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), 81)
        for r in rows:
            self.assertEqual(int(r['network_gross_units']) + int(r['outside_selected_network_units']),
                             int(r['district_gross_units']))
            self.assertEqual(int(r['credited_stock_units']) + int(r['net_transport_units']),
                             int(r['network_gross_units']))


if __name__ == '__main__':
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(ScientificInputChecks)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    record = dict(tests_run=result.testsRun, failures=len(result.failures), errors=len(result.errors),
                  passed=result.wasSuccessful(), scope='Arithmetic, invalid inputs, package fragmentation and thermal-time boundaries. No clinical or aviation validation.')
    (Path(__file__).resolve().parent/'Dogrulama_Kaydi.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
