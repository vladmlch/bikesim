"""Evaluate complete episodes and paired policies without a surrogate reward."""
from collections import defaultdict

from bike_sim.physics.checks import scalar


def assess_run(summary, *, required_duration_s, minimum_progress_m=0.):
    required = scalar(required_duration_s, 'required duration', positive=True)
    minimum = scalar(minimum_progress_m, 'minimum progress', minimum=0.)
    duration = scalar(summary.get('duration_s', 0.), 'actual duration', minimum=0.)
    progress = scalar(summary.get('progress_m', 0.), 'actual progress')
    reasons = []
    if summary.get('numerically_valid') is not True:
        reasons.append('numerical_invalid')
    if summary.get('model_status', {}).get('model_valid') is not True:
        reasons.append('model_invalid')
    if duration+1e-9 < required:
        reasons.append('incomplete_horizon')
    if progress < minimum:
        reasons.append('insufficient_progress')
    outcome = summary.get('outcome') or ''
    if outcome.startswith('crash:'):
        reasons.append('crash')
    elif outcome not in ('duration', 'finish'):
        reasons.append('incomplete_outcome')
    if summary.get('operator_intervention') is True:
        reasons.append('operator_intervention')
    return {'accepted': not reasons, 'reasons': reasons}


def pair_key(record):
    return (record['scenario_id'], record['seed'], record['plant_and_inputs_sha256'])


def paired_summary(records):
    groups = defaultdict(list)
    for record in records:
        groups[pair_key(record)].append(record)
    pairs = []
    exclusions = []
    for key, group in sorted(groups.items()):
        policies = {record['policy_id']: record['metrics'] for record in group}
        reasons = []
        if len(policies) != len(group):
            reasons.append('duplicate_policy')
        if 'passthrough' not in policies or len(policies) < 2:
            reasons.append('missing_baseline_or_candidate')
        durations = []
        for record in group:
            metrics = record['metrics']
            duration = scalar(metrics.get('duration_s', 0.), 'pair duration', minimum=0.)
            durations.append(duration)
            if metrics.get('numerically_valid') is not True:
                reasons.append('numerical_invalid')
            if metrics.get('model_status', {}).get('model_valid') is not True:
                reasons.append('model_invalid')
            if record.get('operator_intervention') or metrics.get('operator_intervention'):
                reasons.append('operator_intervention')
            if metrics.get('outcome') in ('policy_error','simulation_error','operator_stop'):
                reasons.append('incomplete_run')
        if durations and max(durations)-min(durations) > 1e-8:
            reasons.append('different_horizons')
        entry = {'scenario_id': key[0], 'seed': key[1], 'plant_and_inputs_sha256': key[2],
                 'policies': policies, 'comparable': not reasons}
        pairs.append(entry)
        if reasons:
            exclusions.append(dict(entry, reasons=sorted(set(reasons))))
    return {'schema_version': 1, 'records': records, 'pairs': pairs,
            'exclusions': exclusions,
            'comparable_pair_count': sum(pair['comparable'] for pair in pairs)}
