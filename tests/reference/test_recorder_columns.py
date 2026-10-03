import csv
import json
from types import SimpleNamespace

import numpy as np
import pytest

from bike_sim.sim.ride.physical_recorder import PhysicalRecorder, flatten_numbers
from bike_sim.sim.ride.physical_samples import PhysicalSample


def test_columns_are_rectangular_and_csv_matches_rows(tmp_path):
    recorder = PhysicalRecorder.__new__(PhysicalRecorder)
    recorder.samples = []; recorder._columns = {}; recorder._length = 0
    recorder.append_columns(flatten_numbers({'a': 1., 'b': {'c': 2.}}))
    recorder.append_columns(flatten_numbers({'a': 3., 'd': 4.}))
    columns = recorder.columns()
    assert set(columns) == {'a', 'b.c', 'd'}
    np.testing.assert_array_equal(columns['a'], [1., 3.])
    assert np.isnan(columns['b.c'][1]) and np.isnan(columns['d'][0])
    assert recorder.rows == 2
    recorder.write_csv(tmp_path / 't.csv')
    with open(tmp_path / 't.csv') as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]['a'] == '1.0' and rows[1]['d'] == '4.0' and rows[0]['d'] == ''


def test_sample_owns_channels_without_recursive_copy_and_arrays_remain_readonly():
    channels = {'drive': {'power': 2.}, 'values': np.arange(3.)}
    sample = PhysicalSample(0, 0., .0005, np.zeros(4), np.zeros(4), {'f':np.ones(4)}, channels)
    assert sample.channels is channels
    assert not sample.qpos.flags.writeable and not sample.qvel.flags.writeable
    assert not sample.forces['f'].flags.writeable
    assert sample.as_dict()['values'] == [0., 1., 2.]


def test_column_csv_and_streamed_jsonl_preserve_the_original_interval_row(tmp_path):
    recorder = PhysicalRecorder.__new__(PhysicalRecorder)
    recorder.sim = SimpleNamespace(root_x_qposadr=0, root_x_dofadr=0, root_pitch_qposadr=2)
    recorder.samples = []; recorder._columns = {}; recorder._length = 0
    samples = []
    for i in range(3):
        channels = {'mass':dict(mass_kg=100.,com_m=np.array([i,0.,1.]),
                    kinetic_energy_j=2.,gravitational_energy_j=10.),
                    'energy':dict(mechanical_energy_j=12.,residual_j=.01),
                    'component_work_j':{'f':.5*i},'scalar_array':np.array(1.25)}
        sample = PhysicalSample(i,i*.0005,(i+1)*.0005,np.array([i,0.,.1]),
                                np.array([2.,0.,0.]),{'f':np.ones(3)},channels)
        samples.append(sample)
        recorder.append_columns(recorder._row(sample))
        recorder.samples.append(sample.as_dict())
    recorder.write_csv(tmp_path / 'telemetry.csv')
    with (tmp_path / 'telemetry.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    for sample, actual in zip(samples, rows):
        expected = recorder._row(sample)
        assert set(expected) == set(actual)
        for name, value in expected.items():
            assert float(actual[name]) == value
    recorder.write_jsonl(tmp_path / 'intervals.jsonl')
    payloads = [json.loads(line) for line in (tmp_path / 'intervals.jsonl').read_text().splitlines()]
    assert payloads == [sample.as_dict() for sample in samples]


@pytest.mark.slow
def test_runtime_publishes_owned_channels_and_preserves_previous_samples(tmp_path):
    from pathlib import Path
    from bike_sim.cli import research as research_cli
    from bike_sim.sim.ride.control import RideControl
    track = tmp_path / 'flat.toml'
    track.write_text('name="probe"\nlength_m=200.0\nsurface="hardpack"\n'
                     'grade_profile={knots=[[0.0,0.0],[200.0,0.0]]}\n')
    args = research_cli.parser().parse_args([
        '--physics-config', str(Path(__file__).resolve().parents[2] /
                                'examples/research/viewer_physics_welded.toml'),
        '--track-file', str(track), '--dt', '.0005', '--duration', '.02',
        '--diagnostic-model-limits', '--out', str(tmp_path / 'out')])
    env = research_cli.make_environment(args)
    sim = env.sim
    from bike_sim.sim.ride.physical_session import configuration_metadata
    metadata = configuration_metadata(sim)
    assert metadata['timestep_s'] == .0005
    assert metadata['controller_interval_s'] == .005
    command = RideControl(motor_torque_nm=0., human_torque_nm=0.)
    published = []
    for _ in range(10):
        sim.step(control=command)
        published.extend(sim.physical.completed_samples)
    snapshots = [sample.as_dict() for sample in published]
    for _ in range(20):
        sim.step(control=command)
        published.extend(sim.physical.completed_samples)
    assert len({id(sample.channels) for sample in published}) == 30
    assert [sample.as_dict() for sample in published[:10]] == snapshots
    assert published[-1].channels['rider_control'] is not sim.physical.rider_control.last_terms
