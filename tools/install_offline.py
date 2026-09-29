#!/usr/bin/env python3
"""Install the slim bundle from hashed local wheels; never resolve from a network.

The supplied scientific packages are reused explicitly, including when this
installer itself runs inside a pre-existing virtual environment. No paths from
that environment are stored in the distributable archive, only in the new venv.
"""
import argparse
import importlib.metadata as md
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tomllib
import zipfile
from email.parser import BytesParser
from packaging.requirements import Requirement

ROOT = Path(__file__).resolve().parents[1]
BUNDLE = ROOT.parent


def _project():
    with (ROOT/'pyproject.toml').open('rb') as stream:
        return tomllib.load(stream)['project']


def _check(requirement):
    req = Requirement(requirement)
    try:
        version = md.version(req.name)
    except md.PackageNotFoundError as exc:
        raise ValueError(f'missing preinstalled package {requirement}') from exc
    if not req.specifier.contains(version, prereleases=True):
        raise ValueError(f'{requirement} is required; found {req.name}=={version}')
    return version


def verify_install():
    project = _project()
    versions = {Requirement(r).name: _check(r) for r in project['dependencies']}
    if md.version(project['name']) != project['version']:
        raise ValueError('installed bike-sim wheel version does not match the source')
    # Verify the dependency closure, including etils[epath], against the actual
    # interpreter search path. uv's environment metadata alone does not describe
    # packages intentionally reused through an inherited-site .pth file.
    pending = [(Requirement(r).name, set(Requirement(r).extras)) for r in project['dependencies']]
    seen = set()
    while pending:
        name, extras = pending.pop()
        key = (name.lower().replace('_', '-'), tuple(sorted(extras)))
        if key in seen:
            continue
        seen.add(key)
        for text in md.requires(name) or ():
            req = Requirement(text)
            if req.marker and not any(req.marker.evaluate({'extra': e}) for e in (extras or {''})):
                continue
            versions[req.name] = _check(str(req))
            pending.append((req.name, set(req.extras)))
    import bike_sim
    import mujoco
    return dict(passed=True, python=sys.version.split()[0], executable=sys.executable,
        project_version=md.version('bike-sim'), package_path=list(bike_sim.__path__),
        versions=dict(sorted(versions.items())), network_used=False,
        scope='Installed project dependency closure, not a claim that unrelated system packages are consistent.')


def _check_wheel_source(wheelhouse):
    wheels = list(wheelhouse.glob('bike_sim-*.whl'))
    if len(wheels) != 1:
        raise ValueError('wheelhouse must contain exactly one rebuilt bike_sim wheel')
    with zipfile.ZipFile(wheels[0]) as archive:
        metadata_name = next(n for n in archive.namelist() if n.endswith('.dist-info/METADATA'))
        metadata = BytesParser().parsebytes(archive.read(metadata_name))
        if metadata['Version'] != _project()['version']:
            raise ValueError('project wheel is stale: run tools/build_offline_wheel.sh and tools/refresh_offline_wheels.py')
        files = {p.relative_to(ROOT/'src').as_posix(): p for p in (ROOT/'src/bike_sim').rglob('*.py')}
        packaged = {n for n in archive.namelist() if n.startswith('bike_sim/') and n.endswith('.py')}
        if set(files) != packaged or any(archive.read(name) != p.read_bytes() for name, p in files.items()):
            raise ValueError('project wheel content differs from source; rebuild it before installing')


def install(destination):
    if sys.version_info[:2] != (3, 13) or sys.platform != 'linux' or platform.machine() not in ('x86_64', 'AMD64'):
        raise ValueError('supplied binary wheels require Linux x86_64 and CPython 3.13')
    for requirement in _project()['dependencies']:
        if Requirement(requirement).name.lower() not in ('mujoco', 'glfw'):
            _check(requirement)
    wheelhouse = BUNDLE/'wheelhouse'
    _check_wheel_source(wheelhouse)
    destination = Path(destination).resolve()
    env = dict(os.environ, UV_OFFLINE='1', PIP_NO_INDEX='1', PIP_DISABLE_PIP_VERSION_CHECK='1')
    uv = shutil.which('uv')
    if not (destination/'pyvenv.cfg').exists():
        command = ([uv, 'venv', '--offline', '--python', sys.executable, '--system-site-packages', str(destination)]
                   if uv else [sys.executable, '-m', 'venv', '--system-site-packages', str(destination)])
        subprocess.run(command, check=True, env=env)
    python = destination/'bin/python'
    child_version = subprocess.check_output([str(python), '-c', 'import sys; print(sys.version_info[:2])'], text=True)
    if child_version.strip() != '(3, 13)':
        raise ValueError('existing target environment is not CPython 3.13')
    site_path = subprocess.check_output([str(python), '-c',
        'import sysconfig; print(sysconfig.get_path("purelib"))'], text=True).strip()
    inherited = sorted({str(Path(p).resolve()) for p in sys.path
                        if p and Path(p).is_dir() and Path(p).name in ('site-packages', 'dist-packages')})
    (Path(site_path)/'bikesim-inherited-scientific-packages.pth').write_text('\n'.join(inherited)+'\n')
    common = ['--no-index', '--no-deps', '--require-hashes', '--only-binary', ':all:',
              '--find-links', str(wheelhouse), '-r', str(BUNDLE/'requirements.txt')]
    command = ([uv, 'pip', 'install', '--offline', '--python', str(python), '--reinstall-package', 'bike-sim', *common]
               if uv else [str(python), '-m', 'pip', 'install', '--force-reinstall', *common])
    subprocess.run(command, check=True, env=env)
    result = subprocess.check_output([str(python), str(Path(__file__).resolve()), '--verify'], text=True, env=env)
    report = json.loads(result)
    (destination/'bikesim-install-report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(f'Installed bike-sim {report["project_version"]} using only local wheels.')
    print(f'Activate: source {destination}/bin/activate')
    print('Commands: bike-sim, bike-ride, bike-research, bike-replay')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', nargs='?', type=Path, default=BUNDLE/'.venv')
    parser.add_argument('--verify', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    try:
        if args.verify:
            print(json.dumps(verify_install(), sort_keys=True, allow_nan=False))
        else:
            install(args.destination)
        return 0
    except (ValueError, OSError, subprocess.CalledProcessError, md.PackageNotFoundError) as exc:
        print(f'OFFLINE INSTALL ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
