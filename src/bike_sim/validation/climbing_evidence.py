"""Interval-based support and validity evidence, independent of outcome labels."""
from math import isfinite

from bike_sim.sim.research.quality import energy_quality


def support_evidence(rows: list[dict], *, required_duration_s: float | None = None) -> dict:
    if required_duration_s is not None and (
            isinstance(required_duration_s, bool) or not isfinite(required_duration_s)
            or required_duration_s <= 0.):
        raise ValueError('required duration must be finite and positive')
    result = {
        'interval_count': len(rows), 'complete': False,
        'first_both_unloaded_s': None, 'max_both_unloaded_s': 0.,
        'max_front_gap_m': 0., 'max_rear_gap_m': 0.,
        'first_invalid_s': None, 'invalid_reason': [], 'first_unknown_s': None,
        'model_valid': None, 'numerically_valid': None, 'valid_for_learning': False,
        'max_energy_residual_ratio': None, 'support_observed': bool(rows),
        'all_invalid_reasons': [], 'invalid_reason_counts': {},
        'valid_prefix_duration_s': 0., 'valid_prefix_max_gap_m': 0.,
    }
    previous_end = None
    loss_start = None
    loss_duration = 0.
    model_known = numerical_known = bool(rows)
    model_valid = numerical_valid = True
    prefix_valid = bool(rows and abs(rows[0]['time_s']) <= 1e-9)
    for row in rows:
        start, end = row['time_s'], row['end_time_s']
        if (not isfinite(start) or not isfinite(end) or start < 0. or end <= start):
            raise ValueError('support evidence requires finite ordered intervals')
        if previous_end is not None and abs(start - previous_end) > 1e-9:
            raise ValueError('support evidence requires contiguous intervals')
        previous_end = end
        available = False
        unknown = False
        support_unknown = False
        interval_gap = 0.
        for side in ('front', 'rear'):
            pedal = row.get('rider', {}).get(side + '_pedal', {})
            gap = pedal.get('gap_m')
            load = pedal.get('normal_load_n')
            if gap is None or load is None or 'in_platform' not in pedal:
                result['support_observed'] = False
                unknown = True
                support_unknown = True
                continue
            if not isfinite(gap) or not isfinite(load) or load < 0.:
                raise ValueError('support evidence requires finite gaps and nonnegative loads')
            result['max_' + side + '_gap_m'] = max(result['max_' + side + '_gap_m'], gap)
            interval_gap = max(interval_gap, gap)
            available = available or (pedal['in_platform'] is True and load > 1.)
        if available or support_unknown:
            loss_start = None
            loss_duration = 0.
        else:
            if loss_start is None:
                loss_start = start
            loss_duration += end - start
            result['max_both_unloaded_s'] = max(result['max_both_unloaded_s'], loss_duration)
            if loss_duration >= .20 - 1e-12 and result['first_both_unloaded_s'] is None:
                result['first_both_unloaded_s'] = loss_start
        reasons = []
        status = row.get('model_status', {}).get('model_valid')
        if status is False:
            model_valid = False
            reasons.append('model_scope')
        elif status is not True:
            model_known = False
            unknown = True
        energy = row.get('energy')
        if not energy or 'residual_j' not in energy or 'energy_scale_j' not in energy:
            numerical_known = False
            unknown = True
        else:
            try:
                quality = energy_quality(energy)
            except (ValueError, ArithmeticError):
                numerical_valid = False
                reasons.append('invalid_energy')
            else:
                peak = result['max_energy_residual_ratio']
                result['max_energy_residual_ratio'] = max(peak or 0., quality.residual_ratio)
                if not quality.acceptable:
                    numerical_valid = False
                    reasons.append('energy_quality')
        if row.get('model_status', {}).get('numerically_valid') is False:
            numerical_valid = False
            reasons.append('latched_numerical_failure')
        if reasons and result['first_invalid_s'] is None:
            result['first_invalid_s'] = start
            result['invalid_reason'] = reasons
        for reason in reasons:
            counts = result['invalid_reason_counts']
            counts[reason] = counts.get(reason, 0) + 1
        if unknown or reasons:
            prefix_valid = False
        if prefix_valid:
            result['valid_prefix_duration_s'] += end - start
            result['valid_prefix_max_gap_m'] = max(result['valid_prefix_max_gap_m'], interval_gap)
        if unknown and result['first_unknown_s'] is None:
            result['first_unknown_s'] = start
    result['model_valid'] = False if not model_valid else True if model_known else None
    result['all_invalid_reasons'] = sorted(result['invalid_reason_counts'])
    result['numerically_valid'] = False if not numerical_valid else True if numerical_known else None
    result['valid_for_learning'] = (result['model_valid'] is True
        and result['numerically_valid'] is True and result['support_observed'])
    result['complete'] = bool(rows and required_duration_s is not None
        and abs(rows[0]['time_s']) <= 1e-9
        and abs(rows[-1]['end_time_s'] - required_duration_s) <= 1e-8)
    return result
