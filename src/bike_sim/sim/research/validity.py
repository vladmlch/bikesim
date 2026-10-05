"""Pure model-scope checks; never change the physical state or actuator input."""
from math import isfinite
from collections.abc import Mapping


def channel_violations(channels: Mapping, maximum_compression_fraction=.15,
                       maximum_linkage_error_m=.002) -> tuple[str, ...]:
    if not all(isfinite(v) and v >= 0 for v in
               (maximum_compression_fraction, maximum_linkage_error_m)):
        raise ValueError('applicability thresholds must be finite and nonnegative')
    reasons = []
    for side in ('front', 'rear'):
        tire = channels.get('tires', {}).get(side, {})
        if tire.get('multi_support', False) and not tire.get('supports_multiple_contacts', False):
            reasons.append(side+':multi_support')
        if tire.get('normal_load_n', 0.) > 0. and tire.get('outside_material_load_range', False):
            reasons.append(side+':material_load_range')
        if tire.get('outside_material_deflection_range', False):
            reasons.append(side+':material_deflection_range')
        radius = tire.get('unloaded_radius_m')
        if radius is None or not isfinite(radius) or radius <= 0:
            reasons.append(side+':missing_radius')
        elif tire.get('penetration_m', 0.) > radius*maximum_compression_fraction:
            reasons.append(side+':tire_compression')
        if any(p.get('source_geom') == 'catch_plane' and p.get('normal_load_n', 0.) > 0.
               for p in tire.get('patches', ())):
            reasons.append(side+':catch_plane')
        if tire.get('outside_profile_domain', False):
            reasons.append(side+':profile_domain')
    error = channels.get('suspension', {}).get('linkage_closure_max_m', 0.)
    if not isfinite(error) or error > maximum_linkage_error_m:
        reasons.append('linkage:closure_error')
    # Per-step attachment budgets measured from the solved interval (V1/V2).
    # A violated support makes the whole run invalid, not a clipped sample.
    reasons.extend(channels.get('attachment_violations', ()))
    return tuple(reasons)


def model_violations(sample, maximum_compression_fraction=.15, maximum_linkage_error_m=.002):
    """Compatibility adapter for immutable interval samples."""
    return channel_violations(sample.channels, maximum_compression_fraction, maximum_linkage_error_m)
