"""Traceable radial tire calibration records with explicit geometry and provenance.

A TireCalibration pairs an explicit radial load-deflection curve with measured
or synthetic tire geometry (radius, inflated width, rim internal width) and a
dataset provenance tag ('synthetic' or 'measured').
"""
from dataclasses import dataclass
from pathlib import Path
import json

from bike_sim.physics.checks import scalar
from bike_sim.physics.tire_curve import TabulatedTireSpec


@dataclass(frozen=True)
class TireCalibration:
    dataset_id: str
    source_kind: str
    tyre_label: str
    radius_m: float
    inflated_width_m: float
    rim_internal_width_m: float
    material: TabulatedTireSpec

    def __post_init__(self):
        for name in ('dataset_id', 'tyre_label'):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f'{name} is required')
        if self.source_kind not in ('synthetic', 'measured'):
            raise ValueError('source_kind must describe input data provenance')
        for name in ('radius_m', 'inflated_width_m', 'rim_internal_width_m'):
            scalar(getattr(self, name), name, positive=True)
        if not isinstance(self.material, TabulatedTireSpec):
            raise ValueError('an explicit radial curve is required')


_FIELDS = {
    'dataset_id',
    'source_kind',
    'tyre_label',
    'radius_m',
    'inflated_width_m',
    'rim_internal_width_m',
    'pressure_pa_gauge',
    'radial_c_ns_m',
    'deflection_m',
    'force_n',
    'valid_load_range_n',
    'provenance',
}


def calibration_from_record(record: dict) -> TireCalibration:
    if not isinstance(record, dict) or set(record) != _FIELDS:
        raise ValueError('calibration fields must match the version-1 schema exactly')
    material = TabulatedTireSpec(
        deflection_m=tuple(record['deflection_m']),
        force_n=tuple(record['force_n']),
        radial_c_ns_m=record['radial_c_ns_m'],
        pressure_pa_gauge=record['pressure_pa_gauge'],
        provenance=record['provenance'],
        valid_load_range_n=tuple(record['valid_load_range_n']),
    )
    return TireCalibration(
        **{
            k: record[k]
            for k in (
                'dataset_id',
                'source_kind',
                'tyre_label',
                'radius_m',
                'inflated_width_m',
                'rim_internal_width_m',
            )
        },
        material=material,
    )


def load_calibration(path: Path | str) -> TireCalibration:
    path = Path(path)
    with path.open(encoding='utf-8') as stream:
        return calibration_from_record(json.load(stream))
