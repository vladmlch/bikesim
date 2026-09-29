#!/usr/bin/env python3
"""Regenerate hash-pinned requirements from the wheels actually in the bundle."""
from email.parser import BytesParser
import hashlib
from pathlib import Path
import sys
import zipfile


def refresh(bundle):
    bundle = Path(bundle)
    packages = {}
    for path in sorted((bundle/'wheelhouse').glob('*.whl')):
        with zipfile.ZipFile(path) as archive:
            metadata = [n for n in archive.namelist() if n.endswith('.dist-info/METADATA')]
            if len(metadata) != 1:
                raise ValueError(f'{path.name}: expected one wheel metadata file')
            fields = BytesParser().parsebytes(archive.read(metadata[0]))
        name = fields['Name'].lower().replace('_', '-')
        if name in packages:
            raise ValueError(f'duplicate wheel for {name}; remove stale versions')
        packages[name] = f'{name}=={fields["Version"]} --hash=sha256:{hashlib.sha256(path.read_bytes()).hexdigest()}'
    if 'bike-sim' not in packages or 'mujoco' not in packages:
        raise ValueError('bundle requires both the updated project and MuJoCo wheels')
    output = '# Generated from the supplied local wheel files; no index or network lookup.\n'
    (bundle/'requirements.txt').write_text(output+'\n'.join(packages[k] for k in sorted(packages))+'\n')


if __name__ == '__main__':
    refresh(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parents[2])
