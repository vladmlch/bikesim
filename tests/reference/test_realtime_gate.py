"""Full20-second accounted headless measurements; throughput is separate from validity."""
from pathlib import Path

import pytest

from tools.measure_realtime import measure_realtime


@pytest.mark.slow
@pytest.mark.parametrize('track',['rough_uphill_savage.toml','rough_uphill_extreme.toml'])
def test_realtime_factor_for_full_twenty_seconds(track,tmp_path):
    report=measure_realtime(Path('examples/research')/track,
        Path('examples/research/viewer_physics_welded.toml'),20.,out=tmp_path,dt_s=.00125)
    assert report['reason']=='duration',report
    assert report['steps']==16000,report
    assert report['sim_seconds']>=20.-.00125,report
    assert report['factor']>=1.,report
