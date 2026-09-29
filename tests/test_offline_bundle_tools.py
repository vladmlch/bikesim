import hashlib
import importlib.util
from pathlib import Path
import zipfile
import pytest

ROOT = Path(__file__).resolve().parents[1]


def load_tool(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'tools'/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def wheel(directory, name, version, source=b'VALUE=1\n'):
    path = directory/f'{name}-{version}-py3-none-any.whl'
    with zipfile.ZipFile(path, 'w') as archive:
        archive.writestr(f'{name}-{version}.dist-info/METADATA', f'Name: {name}\nVersion: {version}\n')
        if name == 'bike_sim':
            archive.writestr('bike_sim/__init__.py', source)
    return path


def test_manifest_pins_actual_local_wheel_bytes_and_rejects_duplicates(tmp_path):
    tool = load_tool('refresh_offline_wheels')
    wh = tmp_path/'wheelhouse'; wh.mkdir()
    project = wheel(wh, 'bike_sim', '0.3.0')
    wheel(wh, 'mujoco', '3.12.0')
    tool.refresh(tmp_path)
    content = (tmp_path/'requirements.txt').read_text()
    assert f'bike-sim==0.3.0 --hash=sha256:{hashlib.sha256(project.read_bytes()).hexdigest()}' in content
    wheel(wh, 'bike_sim', '0.2.0')
    with pytest.raises(ValueError, match='duplicate wheel'):
        tool.refresh(tmp_path)


def test_stale_or_changed_source_wheel_is_rejected_before_install(tmp_path, monkeypatch):
    tool = load_tool('install_offline')
    (tmp_path/'pyproject.toml').write_text('[project]\nname="bike-sim"\nversion="0.3.0"\n')
    package = tmp_path/'src/bike_sim'; package.mkdir(parents=True)
    (package/'__init__.py').write_bytes(b'VALUE=1\n')
    monkeypatch.setattr(tool, 'ROOT', tmp_path)
    wh = tmp_path/'wheelhouse'; wh.mkdir()
    old = wheel(wh, 'bike_sim', '0.2.0')
    with pytest.raises(ValueError, match='stale'):
        tool._check_wheel_source(wh)
    old.unlink()
    new = wheel(wh, 'bike_sim', '0.3.0')
    tool._check_wheel_source(wh)
    (package/'__init__.py').write_bytes(b'VALUE=2\n')
    with pytest.raises(ValueError, match='differs from source'):
        tool._check_wheel_source(wh)
