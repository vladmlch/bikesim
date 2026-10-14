from pathlib import Path
from zipfile import ZipFile
import argparse
import json


def compare_sources(source_root: Path, wheel: Path) -> dict[str, list[str]]:
    local = {p.relative_to(source_root).as_posix(): p
             for p in (source_root / 'bike_sim').rglob('*.py')}
    with ZipFile(wheel) as z:
        packaged = {n for n in z.namelist()
                    if n.startswith('bike_sim/') and n.endswith('.py')}
        return {
            'missing_source': sorted(packaged - local.keys()),
            'missing_wheel': sorted(local.keys() - packaged),
            'different': sorted(n for n in packaged & local.keys()
                                if z.read(n) != local[n].read_bytes()),
        }


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--wheel', type=Path, required=True)
    a = p.parse_args()
    report = compare_sources(a.source, a.wheel)
    print(json.dumps(report, indent=2))
    return int(any(report.values()))


if __name__ == '__main__':
    raise SystemExit(main())
