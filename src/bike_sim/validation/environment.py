"""Record the actual validation environment without silently changing its lock.

Packaging is already provided by the project's matplotlib dependency. Its PEP
440 implementation avoids guessing how prereleases or compound requirements
compare. No package is installed, upgraded, or modified by this module.
"""
from hashlib import sha256
from importlib import metadata
from pathlib import Path
import platform
import tomllib

from packaging.requirements import Requirement
from packaging.specifiers import SpecifierSet


def source_fingerprint(root: Path) -> str:
    """Hash ordered source paths and bytes; ignore generated interpreter caches."""
    root = Path(root)
    digest = sha256()
    for path in sorted(root.rglob('*.py')):
        relative = path.relative_to(root).as_posix().encode('utf-8')
        payload = path.read_bytes()
        digest.update(len(relative).to_bytes(8, 'big'))
        digest.update(relative)
        digest.update(len(payload).to_bytes(8, 'big'))
        digest.update(payload)
    return digest.hexdigest()


def environment_contract(project_dir=None, *, versions=None, python_version=None):
    """Compare measured direct dependencies with declared and exact locked versions.

    Overrides are for isolated tests only. An absent project/lock is explicitly
    unverified, which also covers a source-less installed wheel deployment.
    """
    root = Path(project_dir) if project_dir is not None else Path(__file__).resolve().parents[3]
    project_path, lock_path = root/'pyproject.toml', root/'uv.lock'
    project_bytes = project_path.read_bytes() if project_path.exists() else None
    lock_bytes = lock_path.read_bytes() if lock_path.exists() else None
    project = tomllib.loads(project_bytes.decode())['project'] if project_bytes else {}
    lock = tomllib.loads(lock_bytes.decode()) if lock_bytes else {}
    python_version = python_version or platform.python_version()
    python_spec = project.get('requires-python')
    python_satisfied = bool(python_spec and SpecifierSet(python_spec).contains(python_version))
    records = {}
    for text in project.get('dependencies', ()):
        requirement = Requirement(text)
        if requirement.marker is not None and not requirement.marker.evaluate():
            continue
        name = requirement.name.lower().replace('_', '-')
        try:
            installed = versions.get(name) if versions is not None else metadata.version(name)
        except metadata.PackageNotFoundError:
            installed = None
        pins = {entry['version'] for entry in lock.get('package', ())
                if entry['name'].lower().replace('_', '-') == name and 'version' in entry}
        # A multi-version universal lock requires marker resolution; never pick
        # one arbitrarily and describe it as an exact environment verification.
        pinned = next(iter(pins)) if len(pins) == 1 else None
        records[name] = {
            'requirement': text,
            'installed_version': installed,
            'locked_version': pinned,
            'declared_satisfied': bool(installed and requirement.specifier.contains(installed)),
            'locked_match': bool(installed and pinned and installed == pinned),
        }
    declared = bool(records) and python_satisfied and all(row['declared_satisfied'] for row in records.values())
    exact = bool(records and lock_bytes) and all(row['locked_match'] for row in records.values())
    return {
        'python_version': python_version,
        'python_requirement': python_spec,
        'python_satisfied': python_satisfied,
        'platform': platform.platform(),
        'packages': records,
        'pyproject_sha256': sha256(project_bytes).hexdigest() if project_bytes else None,
        'uv_lock_sha256': sha256(lock_bytes).hexdigest() if lock_bytes else None,
        'declared_requirements_satisfied': declared,
        'locked_versions_match': exact,
        'locked_environment_verified': declared and exact,
        'scope': 'Direct project dependencies and Python requirement; not transitive wheel integrity.',
    }
