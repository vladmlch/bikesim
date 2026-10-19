from pathlib import Path
from zipfile import BadZipFile, ZipFile
import argparse
import json
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


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


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser()
    p.add_argument('--bundle', type=Path,
                   help='extracted schema-1 bundle root containing manifest.json')
    p.add_argument('--require-host', action='store_true',
                   help='also require current OS/architecture/Python and wheel tags')
    p.add_argument('--source', type=Path, help='legacy Python-only source root')
    p.add_argument('--wheel', type=Path, help='legacy Python-only bike-sim wheel')
    a = p.parse_args(argv)
    if a.bundle is not None:
        if a.source is not None or a.wheel is not None:
            p.error('--bundle cannot be combined with legacy --source/--wheel')
        from tools.offline_bundle import check_bundle
        try:
            manifest = check_bundle(a.bundle, require_host=a.require_host)
        except (OSError, ValueError, KeyError, TypeError, BadZipFile) as error:
            print(f'offline source check: {error}', file=sys.stderr)
            return 1
        print(json.dumps({'schema_version': 1, 'status': 'payload-consistent',
                          'target': manifest['target'],
                          'native_source_build': manifest['capabilities']['native_source_build'],
                          'installation_qualification': 'pending-T25'},
                         indent=2, sort_keys=True))
        return 0
    if a.source is None or a.wheel is None or a.require_host:
        p.error('select --bundle [--require-host], or both --source and --wheel '
                'for legacy Python comparison')
    report = compare_sources(a.source, a.wheel)
    print(json.dumps(report, indent=2))
    return int(any(report.values()))


if __name__ == '__main__':
    raise SystemExit(main())
