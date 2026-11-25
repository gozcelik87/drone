"""Reproduce provisional source-informed demand and packaging stress inputs."""
import csv
from datetime import datetime, timezone
from decimal import Decimal, ROUND_FLOOR
import hashlib
from itertools import product
import json
import platform
from pathlib import Path
import re
import time

from demand_payload import Packaging, apportion, conditional_demand, net_replenishment

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent


def save_json(name, data):
    def encode(x):
        if isinstance(x, Decimal):
            return str(x)
        raise TypeError(type(x).__name__)
    (HERE / name).write_text(json.dumps(data, ensure_ascii=False, indent=2, default=encode) + '\n')


def save_csv(name, rows):
    with (HERE / name).open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def source_injuries():
    path=HERE/'IBB_Mahalle_Yarali_Girdileri.csv'
    with path.open(encoding='utf-8-sig',newline='') as f: rows=list(csv.DictReader(f))
    assert len(rows)==21
    sums={k:sum(int(r[k]) for r in rows) for k in ('L1','L2','L3','L4')}
    assert sums=={'L1':7413,'L2':4079,'L3':925,'L4':1515}
    save_json('Curated_Injury_Input_Check.json',dict(rows=21,totals=sums,
        csv_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        visual_source_PDF_check=False,source_extraction_repeated=False))
    return sums


def main():
    start = time.perf_counter()
    design = json.loads((HERE / 'input_design.json').read_text())
    assert design['mode'] == 'analyst_stress_instances_only'
    # Full provider documents are not distributed; package hashes are checked by reproduce.py.
    totals = source_injuries()
    facilities = json.loads((ROOT / 'experiments/geography/Tesis_Agi.json').read_text())
    weights = design['receiving_weights']
    assert set(weights) <= {f['id'] for f in facilities}
    assert all(f['type'] == 'hospital' for f in facilities if f['id'] in weights)
    assert all(f['blood_handling_eligible'] is None for f in facilities)
    summary, demands, reference = [], [], None
    axes = [design[k] for k in ['transfusion_rate_grid', 'units_per_transfused_patient_grid',
                                'network_share_grid', 'usable_stock_credit_fraction_grid']]
    for index, (p, u, share, stock_fraction) in enumerate(product(*axes), 1):
        case = conditional_demand({'L2': totals['L2'], 'L3': totals['L3']}, design['arrival_fraction'],
                                  {'L2': p, 'L3': p}, {'L2': u, 'L3': u}, share)
        allocated = apportion(case['network_gross_units'], weights)
        stock = {j: int((Decimal(stock_fraction) * n).to_integral_value(rounding=ROUND_FLOOR))
                 for j, n in allocated.items()}
        inbound = dict.fromkeys(allocated, 0)
        net = net_replenishment(allocated, stock, inbound)
        ident = f'S{index:03d}'
        selected = (dict(transfusion_rate=p, units=u, network_share=share, stock_credit=stock_fraction)
                    == design['illustrative_reference'])
        row = dict(scenario_id=ident, status='UNVALIDATED_STRESS_INSTANCE', product='adult_RBC',
                   horizon_hours=design['horizon_hours'], transfusion_rate=p,
                   units_per_transfused_patient=u, network_share=share, stock_credit_fraction=stock_fraction,
                   district_expected_units=case['district_expected_units'],
                   network_expected_units=case['network_expected_units'],
                   district_gross_units=case['district_gross_units'],
                   network_gross_units=case['network_gross_units'],
                   outside_selected_network_units=case['outside_network_units'],
                   credited_stock_units=sum(stock.values()), net_transport_units=sum(net.values()),
                   illustrative_reference=selected)
        summary.append(row)
        for facility in facilities:
            j = facility['id']
            demands.append(dict(scenario_id=ident, node_id=j, name=facility['name'],
                                receiving_assumption=j in weights,
                                clinical_eligibility_verified=False,
                                allocated_gross_units=allocated.get(j, 0),
                                credited_stock_units=stock.get(j, 0), committed_inbound_units=0,
                                net_transport_units=net.get(j, 0),
                                status='conditional_instance_only; other_nodes_zero_by_scope'))
        assert sum(allocated.values()) == case['network_gross_units']
        assert sum(net.values()) + sum(stock.values()) == case['network_gross_units']
        if selected:
            reference = dict(summary=row, allocation=allocated, stock_credit=stock, net_transport=net,
                             clinical_calibration=False, receiving_eligibility_verified=False)
    assert reference is not None and len(summary) == 81
    save_csv('Talep_Senaryolari.csv', summary)
    save_csv('Tesis_Talep_Senaryolari.csv', demands)
    save_json('Ornek_Senaryo.json', reference)

    e = design['payload_experiment']
    capacities = []
    for m, payload, slots in product(e['filled_bag_kg_grid'], e['mission_payload_kg_grid'], e['box_slots_grid']):
        profile = Packaging(m, e['box_and_coolant_kg'], e['units_per_box'], e['attachment_kg'],
                            payload, slots, e['nominal_payload_kg'])
        cap = profile.one_recipient_capacity()
        packed = profile.pack({'one_receiver': cap})
        capacities.append(dict(filled_bag_kg=m, box_and_coolant_kg=e['box_and_coolant_kg'],
                               attachment_kg=e['attachment_kg'], mission_payload_kg=payload,
                               box_slots=slots, units_per_box=e['units_per_box'],
                               one_recipient_upper_bound_units=cap, boxes_at_bound=packed['boxes'],
                               launch_mass_kg_at_bound=packed['launch_mass_kg'],
                               status='UNVALIDATED_PACKAGING_ANALOGUE; recipient_splitting_can_reduce_capacity'))
    save_csv('Paketli_Kapasite_Duyarliligi.csv', capacities)
    profile = Packaging('0.35', '8.45', 12, 2, 50, 2)
    examples = {name: profile.pack(q) for name, q in {
        '24_units_one_receiver': {'A': 24},
        '24_units_three_receivers': {'A': 8, 'B': 8, 'C': 8},
        '25_units_one_receiver': {'A': 25},
    }.items()}
    save_json('Aliciya_Ozel_Paketleme_Ornekleri.json', examples)
    records = [
        ('L2', 4079, 'persons', 'IBB Table 5-2, p.48; transcription verified', 'ibb_district', '48', 'Earthquake scenario estimate, not transfusion recipients'),
        ('L3', 925, 'persons', 'IBB Table 5-2, p.48; transcription verified', 'ibb_district', '48', 'Separate severity class; not excluded as already served'),
        ('p', '0.05/0.07/0.10', 'fraction', 'literature-inspired stress', 'tabatabaie_2010', 'abstract', 'No local/pooled-cohort validation'),
        ('u', '2/4/6', 'RBC units/transfused patient', 'analyst stress', '', '', 'Not empirical means or clinical prescription'),
        ('network_share', '0.10/0.25/0.50', 'fraction', 'analyst stress', '', '', 'Not achieved demand coverage'),
        ('weights', 'equal across N01/N02/N04/N06/N07', 'relative', 'analyst stress', '', '', 'Eligibility, catchment and receiving capacity unknown'),
        ('stock_credit', '0/0.25/0.50', 'fraction of node gross units', 'analyst stress', '', '', 'Floor by receiver; actual stocks unknown'),
        ('bag_mass', '0.30/0.35/0.40', 'kg/filled bag', 'analyst stress', '', '', 'Volume is not a weighed mass; no assured mass bounds'),
        ('box_tare_PCM', 8.45, 'kg/box with coolant', 'approximate published handling reference', 'nhsbt_transport_mass', '2', 'Long-journey box; identity with current DAT48 configuration unconfirmed'),
        ('B', 12, 'RBC units/box', 'published packing reference', 'nhsbt_dat48_2025', '2', 'Specific large-box configuration; no automatic drone transfer'),
        ('holdover_reference', 9, 'hours', 'published packing reference', 'nhsbt_dat48_2025', '2', 'External -5 to +35 C and specified packing; actual flight-qualified holdover remains null'),
        ('attachment', 2, 'kg', 'analyst stress', '', '', 'Must be measured for actual mounting kit'),
        ('box_slots', '1/2/4/8', 'boxes', 'analyst stress', '', '', 'Cargo fit and fixation unverified'),
        ('nominal_payload', 150, 'kg', 'manufacturer reference', 'dasal_c150', 'product page', 'Not an operational bag limit or payload-endurance guarantee'),
    ]
    save_csv('Parametre_Kanitlari.csv', [dict(zip(
        ['parameter','value','unit','evidence_status','source_id','page','limitation'], r)) for r in records])
    save_json('Hesaplama_Kaydi.json', {
        'generated_utc': datetime.now(timezone.utc).isoformat(),
        'python': platform.python_version(), 'platform': platform.platform(),
        'elapsed_seconds': time.perf_counter() - start, 'demand_scenarios': len(summary),
        'facility_scenario_rows': len(demands), 'packaging_scenarios': len(capacities),
        'reference_scenario': reference['summary']['scenario_id'],
        'scenario_range_net_units': [min(r['net_transport_units'] for r in summary), max(r['net_transport_units'] for r in summary)],
        'solver': 'None. Exact arithmetic/input generation; no optimization or operational simulation.',
        'all_source_snapshot_hashes_match': None, 'reproduction_boundary': 'curated CSV; no primary-PDF visual verification',
        'clinical_validation': False, 'flight_validation': False,
        'design_sha256': hashlib.sha256((HERE / 'input_design.json').read_bytes()).hexdigest(),
        'code_sha256': {f: hashlib.sha256((HERE / f).read_bytes()).hexdigest()
                        for f in ['demand_payload.py', 'build_inputs.py']},
    })
    print(json.dumps({'scenarios': len(summary), 'packaging_cases': len(capacities),
                      'illustrative_reference': reference}, ensure_ascii=False, default=str))


if __name__ == '__main__':
    main()
