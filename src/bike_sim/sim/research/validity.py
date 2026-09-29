"""Model applicability diagnostics, separate from numerical-energy checks.

No clipping, force correction or hidden stabilizer is performed here. A run can
conserve energy while evaluating a tire/road model outside its declared scope.
"""


def model_violations(sample, maximum_compression_fraction=.15, maximum_linkage_error_m=.002):
    reasons = []
    for side in ('front', 'rear'):
        tire = sample.channels['tires'][side]
        if tire.get('multi_support', False):
            reasons.append(side+':multi_support')
        if tire.get('normal_load_n', 0.) > 0. and tire.get('outside_material_load_range', False):
            reasons.append(side+':material_load_range')
        radius = tire.get('unloaded_radius_m')
        if radius is not None and tire.get('penetration_m', 0.) > radius*maximum_compression_fraction:
            reasons.append(side+':tire_compression')
        if any(p['source_geom'] == 'catch_plane' and p['normal_load_n'] > 0. for p in tire.get('patches', ())):
            reasons.append(side+':catch_plane')
    error = sample.channels.get('suspension', {}).get('linkage_closure_max_m', 0.)
    if error > maximum_linkage_error_m:
        reasons.append('linkage:closure_error')
    return tuple(reasons)
