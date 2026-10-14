"""bike-batch spec expansion and serial plumbing; pool execution is exercised by hand."""
import json
from pathlib import Path

import pytest

from bike_sim.cli import batch

ROOT = Path(__file__).resolve().parents[1]
WELDED = ROOT / 'examples' / 'research' / 'viewer_physics_welded.toml'


def _track(tmp_path):
    track = tmp_path / 'track.toml'
    track.write_text('name = "probe"\nlength_m = 30.0\nsurface = "hardpack"\n'
                     'grade_profile = { knots = [[0.0, 0.0], [30.0, 0.0]] }\n')
    return track


def test_grid_cartesian_product_and_order():
    jobs = batch.expand_spec({'base': {'scenario': 'rough_uphill'},
                              'grid': {'seed': [1, 2], 'rider-mass': [60., 80.]}})
    assert [j.params['seed'] for j in jobs] == [1, 1, 2, 2]
    assert [j.params['rider-mass'] for j in jobs] == [60., 80., 60., 80.]
    assert jobs[0].argv == ['--scenario', 'rough_uphill', '--seed', '1', '--rider-mass', '60.0']


def test_bool_flags_and_job_overrides():
    jobs = batch.expand_spec({'base': {'assist': True, 'ideal-sensors': False},
                              'job': [{'seed': 7, 'motor-torque': 120.}]})
    assert jobs[0].argv == ['--assist', '--seed', '7', '--motor-torque', '120.0']


def test_range_tables():
    jobs = batch.expand_spec({'grid': {'seed': {'start': 10, 'count': 3},
                                     'rear-teeth': {'start': 40, 'stop': 52, 'step': 5}}})
    assert [j.params['seed'] for j in jobs] == [10, 10, 10, 11, 11, 11, 12, 12, 12]
    assert [j.params['rear-teeth'] for j in jobs] == [40, 45, 50] * 3


def test_out_is_reserved_everywhere():
    for section in ({'base': {'out': 'x'}}, {'grid': {'out': ['x']}}, {'job': [{'out': 'x'}]}):
        with pytest.raises(ValueError, match='out'):
            batch.expand_spec(section)


def test_bad_values_and_sections_are_named():
    with pytest.raises(ValueError, match='unknown spec sections'):
        batch.expand_spec({'junk': {}})
    with pytest.raises(ValueError, match='grid.seed'):
        batch.expand_spec({'grid': {'seed': []}})
    with pytest.raises(ValueError, match='rider-mass'):
        batch.expand_spec({'base': {'rider-mass': [60., 80.]}})
    # an empty spec is one default episode, not an error
    assert len(batch.expand_spec({'job': []})) == 1
    assert len(batch.expand_spec({})) == 1


def test_slug_reflects_overrides():
    jobs = batch.expand_spec({'base': {'duration': 5.},
                              'grid': {'seed': [42]}})
    assert jobs[0].name == '0000-seed42'


def test_validate_rejects_flags_research_does_not_know(tmp_path):
    from bike_sim.cli import research
    jobs = batch.expand_spec({'base': {'not-a-flag': 3}})
    with pytest.raises(ValueError, match='not-a-flag|rejects argv'):
        batch._validate_argv(research.parser(), jobs)


@pytest.mark.slow
def test_serial_two_job_episode(tmp_path):
    """Two minimal episodes through the real pipeline, sequentially."""
    spec = tmp_path / 'spec.toml'
    spec.write_text(f'''[base]
physics-config = "{WELDED}"
track-file = "{_track(tmp_path)}"
duration = 0.2
initial-speed = 0
energy-tolerance = 1e9
diagnostic-model-limits = true
record-decimation = 400

[grid]
seed = [0, 1]
''')
    out = tmp_path / 'batch'
    code = batch.main(['--spec', str(spec), '--out', str(out), '--serial'])
    records = [json.loads(line)
               for line in (out / 'batch_results.jsonl').read_text().splitlines()]
    assert len(records) == 2
    assert {r['name'] for r in records} == {'0000-seed0', '0001-seed1'}
    assert all(r['exit'] in (0, 2, 3) for r in records)  # plumbing, not physics
    assert all((out / r['name']).is_dir() for r in records)
    assert code == (0 if all(r['exit'] == 0 for r in records) else 1)
