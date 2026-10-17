"""bike-batch spec expansion, CPU policy and actual serial/pool equivalence."""
import json
import os
from pathlib import Path
import subprocess
import sys

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


@pytest.mark.parametrize('jobs', ['0', '-2'])
def test_nonpositive_worker_count_is_rejected_before_output(tmp_path, jobs):
    out = tmp_path / 'out'
    with pytest.raises(SystemExit) as error:
        batch.main(['--spec', str(tmp_path / 'absent.toml'), '--out', str(out), '--jobs', jobs])
    assert error.value.code == 2
    assert not out.exists()


def test_explicit_serial_job_count_avoids_discovery_and_reports_threads(tmp_path, monkeypatch, capsys):
    spec = tmp_path / 'spec.toml'
    spec.write_text('[base]\nseed = 9\n')

    def forbidden():
        raise AssertionError('explicit --jobs performed CPU discovery')

    def job(task):
        index, name, argv, out = task
        return dict(index=index, name=name, exit=0, wall_s=0., out=out)

    monkeypatch.setattr(batch, '_performance_cores', forbidden)
    monkeypatch.setattr(batch, '_run_job', job)
    monkeypatch.setenv('OPENBLAS_NUM_THREADS', '2')
    out = tmp_path / 'batch'
    assert batch.main(['--spec', str(spec), '--out', str(out), '--jobs', '1']) == 0
    text = capsys.readouterr().out
    assert 'workers=1' in text and 'start_method=in-process' in text
    report = batch._tail_json(text.strip())
    assert report['workers'] == 1
    assert report['thread_policy']['OPENBLAS_NUM_THREADS'] == '2'
    assert os.environ['OPENBLAS_NUM_THREADS'] == '2'


@pytest.mark.slow
def test_actual_two_worker_batch_matches_serial_jobs(tmp_path):
    spec = tmp_path / 'spec.toml'
    spec.write_text(f'''[base]
physics-config = "{WELDED}"
track-file = "{_track(tmp_path)}"
duration = 0.01
check-policy = "diagnostic"
motor-torque = 0
human-torque = 0

[grid]
seed = [0, 1]
''')
    environment = dict(os.environ)
    for name in ('DISPLAY', 'WAYLAND_DISPLAY', 'MUJOCO_GL'):
        environment.pop(name, None)
    records = []
    for mode in (['--serial'], ['--jobs', '2']):
        out = tmp_path / ('serial' if mode[0] == '--serial' else 'pool')
        result = subprocess.run(
            [sys.executable, '-m', 'bike_sim.cli.batch', '--spec', str(spec),
             '--out', str(out), *mode], cwd=ROOT, env=environment,
            capture_output=True, text=True, timeout=180, check=False,
        )
        diagnostic = result.stdout + result.stderr
        assert result.returncode in (0, 1), diagnostic
        assert 'resource_tracker:' not in result.stderr
        rows = [json.loads(line) for line in (out / 'batch_results.jsonl').read_text().splitlines()]
        assert len(rows) == 2, diagnostic
        assert all(row['exit'] in (0, 2) and row.get('outcome') == 'duration'
                   and 'error' not in row for row in rows), diagnostic
        records.append({row['index']: {key: row[key] for key in (
            'name', 'exit', 'outcome', 'metrics', 'params', 'argv')} for row in rows})
        assert {Path(row['out']).parent for row in rows} == {out}
        assert len({row['out'] for row in rows}) == 2
    assert records[0] == records[1]
