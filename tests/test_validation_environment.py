"""Never confuse a successful numeric run with a verified locked environment."""
import json
from bike_sim.validation.environment import environment_contract, source_fingerprint


def fixture_project(tmp_path):
    (tmp_path/'pyproject.toml').write_text('''[project]
name="fixture"
requires-python=">=3.12"
dependencies=["numpy>=2.0", "scipy>=1.18"]
''')
    (tmp_path/'uv.lock').write_text('''version=1
[[package]]
name="numpy"
version="2.5.2"
[[package]]
name="scipy"
version="1.18.1"
''')
    return tmp_path


def test_environment_reports_mismatch_without_rewriting_dependencies(tmp_path):
    root=fixture_project(tmp_path)
    contract=environment_contract(root,versions={'numpy':'2.3.5','scipy':'1.17.0'},python_version='3.13.5')
    assert not contract['declared_requirements_satisfied']
    assert not contract['locked_versions_match']
    assert contract['packages']['numpy']['declared_satisfied']
    assert not contract['packages']['scipy']['declared_satisfied']
    assert contract['packages']['scipy']['locked_version']=='1.18.1'
    assert contract['packages']['scipy']['installed_version']=='1.17.0'
    assert (root/'pyproject.toml').read_text().endswith('dependencies=["numpy>=2.0", "scipy>=1.18"]\n')
    json.dumps(contract,allow_nan=False)


def test_exact_lock_and_python_floor_are_independent(tmp_path):
    root=fixture_project(tmp_path)
    exact={'numpy':'2.5.2','scipy':'1.18.1'}
    assert environment_contract(root,versions=exact,python_version='3.13.5')['locked_environment_verified']
    result=environment_contract(root,versions=exact,python_version='3.11.9')
    assert result['locked_versions_match']
    assert not result['locked_environment_verified']


def test_missing_lock_is_not_reported_as_verified(tmp_path):
    root=fixture_project(tmp_path)
    (root/'uv.lock').unlink()
    result=environment_contract(root,versions={'numpy':'2.5.2','scipy':'1.18.1'},python_version='3.13.5')
    assert not result['locked_environment_verified']
    assert result['uv_lock_sha256'] is None


def test_source_fingerprint_tracks_paths_and_bytes_not_bytecode(tmp_path):
    root=tmp_path/'src';root.mkdir()
    source=root/'a.py';source.write_text('a=1\n')
    before=source_fingerprint(root)
    cache=root/'__pycache__';cache.mkdir();(cache/'a.pyc').write_bytes(b'cache')
    assert source_fingerprint(root)==before
    source.rename(root/'b.py')
    assert source_fingerprint(root)!=before
