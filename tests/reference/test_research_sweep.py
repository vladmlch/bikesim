"""research_sweep fans one bike-research process out per seed."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def test_dry_run_lists_one_command_per_seed(tmp_path, capsys):
    from tools.research_sweep import main
    assert main(['--out', str(tmp_path), '--seeds', '3-5', '--dry-run', '--', '--duration', '1']) == 0
    lines = [l for l in capsys.readouterr().out.splitlines() if 'bike-research' in l]
    assert len(lines) == 3
    assert all('--duration 1' in l for l in lines)
    assert [l.split('--seed ')[1].split()[0] for l in lines] == ['3', '4', '5']


def test_seed_ranges_and_lists_parse():
    from tools.research_sweep import parse_seeds
    assert parse_seeds('1-3,7') == [1, 2, 3, 7]
    assert parse_seeds('4') == [4]


def test_summary_records_return_codes(tmp_path, monkeypatch):
    from tools import research_sweep
    monkeypatch.setattr(research_sweep, 'run_one', lambda cmd, log: (0 if '--seed 2' in ' '.join(cmd) else 3, .1))
    assert research_sweep.main(['--out', str(tmp_path), '--seeds', '1-2', '--jobs', '2']) == 1
    summary = json.loads((tmp_path/'sweep.json').read_text())
    assert {row['seed']: row['returncode'] for row in summary['runs']} == {1: 3, 2: 0}
