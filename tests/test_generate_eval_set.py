import json
from tools.generate_eval_set import main
from bike_sim.terrain.trackfile import load_track
from bike_sim.terrain.generator import TerrainGenSpec


def test_eval_set_is_reproducible_and_loadable(tmp_path):
    assert main(['--n', '3', '--seed0', '1000', '--out', str(tmp_path/'a'), '--date', '2026-09-30']) == 0
    assert main(['--n', '3', '--seed0', '1000', '--out', str(tmp_path/'b'), '--date', '2026-09-30']) == 0
    for i in range(3):
        name = f'eval_{i:02d}.toml'
        assert (tmp_path/'a'/name).read_text() == (tmp_path/'b'/name).read_text()
        assert load_track(tmp_path/'a'/name).length_m > 0.
    manifest = json.loads((tmp_path/'a'/'manifest.json').read_text())
    assert [t['seed'] for t in manifest['tracks']] == [1000, 1001, 1002]
    assert manifest['spec_sha256'] == TerrainGenSpec().sha256() and manifest['date'] == '2026-09-30'
    assert (tmp_path/'a'/'manifest.json').read_text() == (tmp_path/'b'/'manifest.json').read_text()


def test_refuses_to_overwrite_existing_eval_set(tmp_path):
    main(['--n', '1', '--out', str(tmp_path)])
    assert main(['--n', '1', '--out', str(tmp_path)]) == 2
