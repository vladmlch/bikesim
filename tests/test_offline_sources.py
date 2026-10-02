from pathlib import Path
from zipfile import ZipFile
from tools.check_offline_sources import compare_sources


def test_missing_and_changed_modules_are_not_silently_accepted(tmp_path):
    source = tmp_path / 'src'
    (source / 'bike_sim').mkdir(parents=True)
    (source / 'bike_sim/a.py').write_text('VALUE = 1\n')
    wheel = tmp_path / 'fixture.whl'
    with ZipFile(wheel, 'w') as z:
        z.writestr('bike_sim/a.py', 'VALUE = 2\n')
        z.writestr('bike_sim/b.py', 'VALUE = 3\n')
    assert compare_sources(source, wheel) == {
        'missing_source': ['bike_sim/b.py'],
        'missing_wheel': [],
        'different': ['bike_sim/a.py'],
    }
