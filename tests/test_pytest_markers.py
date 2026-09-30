import tomllib
from pathlib import Path


def test_slow_marker_is_registered():
    config = tomllib.loads((Path(__file__).resolve().parents[1]/'pyproject.toml').read_text())
    markers = config['tool']['pytest']['ini_options']['markers']
    assert any(m.startswith('slow:') for m in markers)
