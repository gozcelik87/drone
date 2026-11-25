"""Conditional research inputs; not a clinical forecast or flight clearance.

All quantities are component-specific adult RBC units. No missing empirical
parameter is silently filled here. Decimal arithmetic avoids rounding a value
just above an integer because of binary floating-point error.
"""
from decimal import Decimal, ROUND_CEILING, ROUND_FLOOR
from dataclasses import dataclass


def number(value, name, minimum=0, maximum=None):
    if isinstance(value, bool) or value is None:
        raise ValueError(name + ': a finite numeric value is required')
    try:
        d = Decimal(str(value))
    except Exception as exc:
        raise ValueError(name + ': invalid number') from exc
    if not d.is_finite() or d < minimum or (maximum is not None and d > maximum):
        raise ValueError(name + ': outside the admissible range')
    return d


def integer(value, name, minimum=0):
    if type(value) is not int or value < minimum:
        raise ValueError(name + ': an integer is required')
    return value


def ceil_units(value):
    return int(number(value, 'units').to_integral_value(rounding=ROUND_CEILING))


def conditional_demand(injuries, arrival_fractions, transfusion_rates,
                       units_per_recipient, network_share):
    """District expectation and an explicitly scoped network replenishment load.

    Injury categories must be mutually exclusive. Fatalities are not recipients.
    Rates and average units refer to the same component and planning horizon.
    network_share is a catchment/scenario share, never a claim of service achieved.
    """
    keys = set(injuries)
    if not keys or any(set(d) != keys for d in
                       (arrival_fractions, transfusion_rates, units_per_recipient)):
        raise ValueError('Every injury class needs all three conversion inputs')
    contributions = {}
    for level, count in injuries.items():
        if level == 'L4':
            raise ValueError('Fatalities cannot be counted as surviving recipients')
        contributions[level] = (
            number(count, 'injuries') * number(arrival_fractions[level], 'arrival', maximum=1)
            * number(transfusion_rates[level], 'transfusion rate', maximum=1)
            * number(units_per_recipient[level], 'units per transfused recipient'))
    district = sum(contributions.values(), Decimal(0))
    share = number(network_share, 'network share', maximum=1)
    scoped = district * share
    district_integer, network_integer = ceil_units(district), ceil_units(scoped)
    return {
        'by_class': contributions,
        'district_expected_units': district,
        'network_expected_units': scoped,
        'district_gross_units': district_integer,
        'network_gross_units': network_integer,
        'outside_network_units': district_integer - network_integer,
        'outside_meaning': 'Outside the selected planning scope; not proven unmet demand',
    }


def apportion(total, weights):
    """Hamilton apportionment: exact total, zero-weight exclusion, ID tie break."""
    integer(total, 'total')
    if not weights:
        if total == 0:
            return {}
        raise ValueError('Positive demand needs at least one receiver')
    w = {k: number(v, 'allocation weight') for k, v in weights.items()}
    denom = sum(w.values(), Decimal(0))
    if denom == 0:
        if total:
            raise ValueError('Positive demand cannot be allocated to zero weights')
        return {k: 0 for k in sorted(w)}
    quotas = {k: Decimal(total) * v / denom for k, v in w.items()}
    result = {k: int(v.to_integral_value(rounding=ROUND_FLOOR)) for k, v in quotas.items()}
    order = sorted((k for k in w if w[k] > 0), key=lambda k: (-(quotas[k] - result[k]), k))
    for k in order[:total - sum(result.values())]:
        result[k] += 1
    return {k: result[k] for k in sorted(result)}


def net_replenishment(gross, usable_stock, committed_inbound):
    """Stocks must be usable compatible units reserved for this demand horizon.

    Do not credit the same units here and as inbound supply to the routing model.
    Routine demand must be reserved separately or added to gross before this call.
    """
    if set(gross) != set(usable_stock) or set(gross) != set(committed_inbound):
        raise ValueError('Stock and inbound must be specified for every receiver')
    return {k: max(0, integer(gross[k], 'gross') - integer(usable_stock[k], 'stock')
                   - integer(committed_inbound[k], 'inbound')) for k in gross}


@dataclass(frozen=True)
class Packaging:
    filled_bag_kg: object
    box_and_coolant_kg: object
    units_per_box: int
    attachment_kg: object
    mission_payload_kg: object
    available_box_slots: int
    nominal_payload_kg: object = 150

    def validate(self):
        if number(self.filled_bag_kg, 'filled bag mass') <= 0:
            raise ValueError('Filled bag mass must be positive')
        number(self.box_and_coolant_kg, 'box and coolant mass')
        number(self.attachment_kg, 'attachment mass')
        integer(self.units_per_box, 'qualified units per box', 1)
        integer(self.available_box_slots, 'box slots')
        payload = number(self.mission_payload_kg, 'mission payload')
        if payload > number(self.nominal_payload_kg, 'nominal payload'):
            raise ValueError('Mission allowance exceeds the nominal payload')

    def pack(self, quantities):
        """Minimum separate sealed boxes per recipient on one sortie.

        No box is shared between recipients. A partly filled box retains its full
        tare/coolant mass. Shipment boxes are handed over; attachment returns.
        Returned packaging/waste would require a different load model.
        """
        self.validate()
        q = {j: integer(n, 'delivered units') for j, n in quantities.items()}
        boxes = {j: (n + self.units_per_box - 1) // self.units_per_box for j, n in q.items()}
        n_units, n_boxes = sum(q.values()), sum(boxes.values())
        mass = (number(self.filled_bag_kg, 'bag mass') * n_units
                + number(self.box_and_coolant_kg, 'box mass') * n_boxes
                + number(self.attachment_kg, 'attachment'))
        return {'units': n_units, 'boxes': n_boxes, 'boxes_by_recipient': boxes,
                'launch_mass_kg': mass,
                'mass_feasible': mass <= number(self.mission_payload_kg, 'payload'),
                'slots_feasible': n_boxes <= self.available_box_slots,
                'feasible': mass <= number(self.mission_payload_kg, 'payload')
                            and n_boxes <= self.available_box_slots}

    def one_recipient_capacity(self):
        """Upper bound for one recipient; multi-recipient fragmentation can lower it."""
        self.validate()
        # Finite enumeration also covers cases where a partly filled box is best.
        best = 0
        for units in range(1, self.units_per_box * self.available_box_slots + 1):
            if self.pack({'receiver': units})['feasible']:
                best = units
        return best


def thermal_age_at_delivery(age_at_dispatch, vehicle_time, preparation_time,
                            elapsed_to_handover, qualified_holdover):
    """Full exposure age, including handover, not just flight time.

    Time zero is vehicle dispatch. age_at_dispatch is time already outside
    controlled storage, including any delay before package sealing. A return leg
    affects endurance, but not the age of an intact box already handed over.
    """
    values = (age_at_dispatch, vehicle_time, preparation_time, elapsed_to_handover)
    age = sum((number(t, 'time') for t in values), Decimal(0))
    limit = number(qualified_holdover, 'qualified holdover')
    if limit <= 0:
        raise ValueError('A positive qualified holdover is required')
    return {'age_min': age, 'feasible': age <= limit}
