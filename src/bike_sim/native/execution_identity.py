"""Versioned numerical execution identity — data-only schema validation.

This module is the persisted-record contract for the numerical execution
identity carried by schema-2/3 recordings and compared before checked
replay. It is a *data* module: it performs no imports of MuJoCo, NumPy,
threadpoolctl or the native extension, and never opens a pathname carried
inside a record — the schema contains no paths at all. Runtime capture
lives at the boundary in ``bike_sim.native.artifact`` (the actual loaded
provider, measured live) and in the compile-time ``research_build_identity.hpp``
(the qualified provider-policy record baked by CMake from the dependency
probe). Do not conflate the build-tool context (``native_check_context.json``,
schema 2, a *build* record with tooling paths) with this persisted
``identity_schema=2`` record.

Records:

- **legacy** (no ``identity_schema``): the eight ``EXECUTION_FIELDS``. A
  Python record carries ``build_context=None``; a native record carries the
  four-string build context. Read validation of these records is preserved
  indefinitely.
- **v2** (``identity_schema=2``): the eight legacy fields plus
  ``identity_schema``. ``build_context`` is ``{platform, numerics}`` for
  Python and ``{build_type, compiler_id, compiler_version,
  numerical_flags, platform, numerics}`` for native. ``platform`` and
  ``numerics`` describe the *selected* environment identity — the only
  qualified pairings are Darwin+ACCELERATE and Linux+NUMPY_OPENBLAS.

Checked replay compares records field-by-field via
``execution_identity_differences`` so a mismatch names the exact dotted
field instead of an opaque whole-dict inequality. Data inspection and
non-simulating playback validate records without comparing them to the
current runtime.
"""
from __future__ import annotations

import re

# ---------------------------------------------------------------------------
# Field vocabulary
# ---------------------------------------------------------------------------

# The legacy record set — kept identical to artifact.EXECUTION_FIELDS so old
# callers and old recordings keep their contract.
LEGACY_EXECUTION_FIELDS = frozenset({
    'backend', 'runtime_schema', 'python_source_sha256',
    'native_source_sha256', 'extension_sha256', 'mujoco_version',
    'mujoco_library_sha256', 'build_context',
})
# v2 adds exactly one key; runtime_schema stays 1.
IDENTITY_SCHEMA_FIELD = 'identity_schema'
IDENTITY_SCHEMA_VERSION = 2
V2_EXECUTION_FIELDS = LEGACY_EXECUTION_FIELDS | {IDENTITY_SCHEMA_FIELD}

LEGACY_CONTEXT_FIELDS = frozenset({
    'build_type', 'compiler_id', 'compiler_version', 'numerical_flags',
})
NATIVE_CONTEXT_FIELDS = LEGACY_CONTEXT_FIELDS | {'platform', 'numerics'}
ENVIRONMENT_CONTEXT_FIELDS = frozenset({'platform', 'numerics'})

PLATFORM_FIELDS = frozenset({
    'system', 'machine', 'libc', 'os_build', 'python_version',
    'python_soabi',
})
LIBC_FIELDS = frozenset({'name', 'version'})
NUMERICS_FIELDS = frozenset({
    'backend', 'numpy_version', 'blas_integer_bits', 'lapack_integer_bits',
    'cpu_profile', 'threading', 'libraries',
})
THREADING_FIELDS = frozenset({'implementation', 'num_threads'})
LIBRARY_FIELDS = frozenset({'role', 'identity_kind', 'sha256', 'build_id'})

_BACKENDS = frozenset({'python', 'native'})
_PLATFORMS = frozenset({'Darwin', 'Linux'})
# The only qualified (platform, provider) pairings — the provider policy the
# port actually supports. Anything else is unqualified, never reinterpreted.
_PLATFORM_BACKENDS = {'Darwin': 'ACCELERATE', 'Linux': 'NUMPY_OPENBLAS'}
_PROVIDER_BACKENDS = frozenset({'ACCELERATE', 'NUMPY_OPENBLAS'})
# Qualified integer widths per provider (the probe's _NUMERICS_WIDTHS):
# Accelerate keeps legacy 32-bit CBLAS with 64-bit NEWLAPACK; the wheel
# scipy-openblas is ILP64 on both interfaces.
_PROVIDER_WIDTHS = {'ACCELERATE': (32, 64), 'NUMPY_OPENBLAS': (64, 64)}
_PROVIDER_IDENTITY_KINDS = {'ACCELERATE': 'apple-system',
                            'NUMPY_OPENBLAS': 'file'}
_IDENTITY_KINDS = frozenset({'file', 'apple-system'})
_SHA256 = re.compile(r'[0-9a-f]{64}')
# Normalized architecture spelling: lowercase ISA token, no whitespace.
_MACHINE = re.compile(r'[a-z0-9_.+-]+')


def normalize_machine(value: str) -> str:
    """Canonical ISA spelling for ``platform.machine`` records.

    Writers record this normalized form; readers compare it verbatim and
    never re-normalize a stored record.
    """
    machine = str(value).strip().lower()
    return {'amd64': 'x86_64', 'x64': 'x86_64',
            'arm64': 'aarch64'}.get(machine, machine)


# ---------------------------------------------------------------------------
# Validation — pure data; raises ValueError naming the offending field
# ---------------------------------------------------------------------------


def _is_int(value) -> bool:
    return type(value) is int


def _nonempty_string(value) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _require_nonempty(record: dict, key: str, path: str) -> str:
    value = record.get(key)
    if not _nonempty_string(value):
        raise ValueError(
            f'execution identity field {path} must be a nonempty string, '
            f'found {value!r}')
    return value


def _validate_libc(value, path: str) -> None:
    if not isinstance(value, dict) or set(value) != LIBC_FIELDS:
        raise ValueError(
            f'execution identity field {path} must be an object with exactly '
            f'{sorted(LIBC_FIELDS)}')
    for key in sorted(LIBC_FIELDS):
        _require_nonempty(value, key, f'{path}.{key}')


def _validate_platform(value, path: str) -> None:
    if not isinstance(value, dict) or set(value) != PLATFORM_FIELDS:
        raise ValueError(
            f'execution identity field {path} must be an object with exactly '
            f'{sorted(PLATFORM_FIELDS)}')
    system = _require_nonempty(value, 'system', f'{path}.system')
    if system not in _PLATFORMS:
        raise ValueError(
            f'execution identity field {path}.system must be one of '
            f'{sorted(_PLATFORMS)}, found {system!r}')
    machine = _require_nonempty(value, 'machine', f'{path}.machine')
    if _MACHINE.fullmatch(machine) is None:
        raise ValueError(
            f'execution identity field {path}.machine must be a normalized '
            f'architecture token, found {machine!r}')
    _validate_libc(value['libc'], f'{path}.libc')
    _require_nonempty(value, 'os_build', f'{path}.os_build')
    _require_nonempty(value, 'python_version', f'{path}.python_version')
    _require_nonempty(value, 'python_soabi', f'{path}.python_soabi')


def _validate_libraries(value, backend: str, path: str) -> None:
    if not isinstance(value, list) or not value:
        raise ValueError(
            f'execution identity field {path} must be a nonempty list, '
            f'found {value!r}')
    expected_kind = _PROVIDER_IDENTITY_KINDS[backend]
    for index, entry in enumerate(value):
        label = f'{path}[{index}]'
        if not isinstance(entry, dict) or set(entry) != LIBRARY_FIELDS:
            raise ValueError(
                f'execution identity field {label} must be an object with '
                f'exactly {sorted(LIBRARY_FIELDS)}')
        if entry['role'] != 'blas_lapack':
            raise ValueError(
                f'execution identity field {label}.role must be '
                f"'blas_lapack', found {entry['role']!r}")
        kind = entry['identity_kind']
        if kind not in _IDENTITY_KINDS:
            raise ValueError(
                f'execution identity field {label}.identity_kind must be one '
                f'of {sorted(_IDENTITY_KINDS)}, found {kind!r}')
        if kind != expected_kind:
            raise ValueError(
                f'execution identity field {label}.identity_kind is '
                f'{kind!r}, but backend {backend} requires '
                f'{expected_kind!r}')
        sha256, build_id = entry['sha256'], entry['build_id']
        if kind == 'file':
            if not isinstance(sha256, str) or _SHA256.fullmatch(sha256) is None:
                raise ValueError(
                    f'execution identity field {label}.sha256 must be a '
                    f'lowercase 64-hex digest, found {sha256!r}')
            if build_id is not None:
                raise ValueError(
                    f'execution identity field {label}.build_id must be null '
                    f'for a file identity, found {build_id!r}')
        else:  # apple-system — dyld-cache images have no hashable file
            if sha256 is not None:
                raise ValueError(
                    f'execution identity field {label}.sha256 must be null '
                    f'for an apple-system identity, found {sha256!r}')
            if not _nonempty_string(build_id):
                raise ValueError(
                    f'execution identity field {label}.build_id must be a '
                    f'nonempty string for an apple-system identity, found '
                    f'{build_id!r}')


def _validate_numerics(value, path: str) -> None:
    if not isinstance(value, dict) or set(value) != NUMERICS_FIELDS:
        raise ValueError(
            f'execution identity field {path} must be an object with exactly '
            f'{sorted(NUMERICS_FIELDS)}')
    backend = value.get('backend')
    if backend not in _PROVIDER_BACKENDS:
        raise ValueError(
            f'execution identity field {path}.backend must be one of '
            f'{sorted(_PROVIDER_BACKENDS)}, found {backend!r}')
    _require_nonempty(value, 'numpy_version', f'{path}.numpy_version')
    blas_bits = value.get('blas_integer_bits')
    lapack_bits = value.get('lapack_integer_bits')
    need_blas, need_lapack = _PROVIDER_WIDTHS[backend]
    if (not _is_int(blas_bits) or not _is_int(lapack_bits)
            or (blas_bits, lapack_bits) != (need_blas, need_lapack)):
        raise ValueError(
            f'execution identity field {path}: unqualified BLAS/LAPACK '
            f'integer ABI for backend {backend}: '
            f'blas_integer_bits={blas_bits!r} '
            f'lapack_integer_bits={lapack_bits!r}; the qualified profile is '
            f'{need_blas}-bit BLAS with {need_lapack}-bit LAPACK')
    _require_nonempty(value, 'cpu_profile', f'{path}.cpu_profile')
    threading = value.get('threading')
    if not isinstance(threading, dict) or set(threading) != THREADING_FIELDS:
        raise ValueError(
            f'execution identity field {path}.threading must be an object '
            f'with exactly {sorted(THREADING_FIELDS)}')
    _require_nonempty(
        threading, 'implementation', f'{path}.threading.implementation')
    num_threads = threading.get('num_threads')
    if not _is_int(num_threads) or num_threads < 1:
        raise ValueError(
            f'execution identity field {path}.threading.num_threads must be '
            f'a positive integer, found {num_threads!r}')
    _validate_libraries(value['libraries'], backend, f'{path}.libraries')


def _validate_context(value, backend: str, path: str) -> None:
    if backend == 'python':
        expected = ENVIRONMENT_CONTEXT_FIELDS
    else:
        expected = NATIVE_CONTEXT_FIELDS
    if not isinstance(value, dict) or set(value) != expected:
        raise ValueError(
            f'execution identity field {path} must be an object with exactly '
            f'{sorted(expected)}')
    if backend == 'native':
        for key in sorted(LEGACY_CONTEXT_FIELDS):
            _require_nonempty(value, key, f'{path}.{key}')
    _validate_platform(value['platform'], f'{path}.platform')
    _validate_numerics(value['numerics'], f'{path}.numerics')
    platform_backend = _PLATFORM_BACKENDS[value['platform']['system']]
    if value['numerics']['backend'] != platform_backend:
        raise ValueError(
            f'execution identity field {path}.platform.system '
            f'{value["platform"]["system"]!r} cannot host numerics.backend '
            f'{value["numerics"]["backend"]!r} — the qualified pairing is '
            f'{platform_backend!r}')


def _validate_hashes(value) -> None:
    for name in ('python_source_sha256', 'native_source_sha256',
                 'extension_sha256', 'mujoco_library_sha256'):
        expected_null = (value['backend'] == 'python'
                         and name != 'python_source_sha256')
        if expected_null:
            if value[name] is not None:
                raise ValueError(
                    f'execution identity field {name} must be null for the '
                    'python backend')
        elif (not isinstance(value[name], str)
              or _SHA256.fullmatch(value[name]) is None):
            raise ValueError(
                f'execution identity field {name} must be a lowercase '
                'SHA-256 digest')


def _validate_legacy(value) -> None:
    if set(value) != LEGACY_EXECUTION_FIELDS:
        raise ValueError(
            'execution identity record must contain exactly the legacy '
            'identity fields')
    if value['backend'] not in _BACKENDS:
        raise ValueError('unsupported recorded execution backend')
    if not _is_int(value['runtime_schema']) or value['runtime_schema'] != 1:
        raise ValueError('unsupported recorded runtime schema')
    if not _nonempty_string(value['mujoco_version']):
        raise ValueError(
            'execution identity field mujoco_version must be a nonempty '
            'string')
    _validate_hashes(value)
    context = value['build_context']
    if value['backend'] == 'python':
        if context is not None:
            raise ValueError(
                'Python execution.build_context must be null in a legacy '
                'identity record')
    elif (not isinstance(context, dict) or set(context) != LEGACY_CONTEXT_FIELDS
          or any(not _nonempty_string(item) for item in context.values())):
        raise ValueError('native execution.build_context is incomplete')


def _validate_v2(value) -> None:
    if set(value) != V2_EXECUTION_FIELDS:
        raise ValueError(
            'execution identity record must contain exactly the '
            'identity-schema-2 fields')
    if value['backend'] not in _BACKENDS:
        raise ValueError('unsupported recorded execution backend')
    if not _is_int(value['runtime_schema']) or value['runtime_schema'] != 1:
        raise ValueError('unsupported recorded runtime schema')
    if not _nonempty_string(value['mujoco_version']):
        raise ValueError(
            'execution identity field mujoco_version must be a nonempty '
            'string')
    _validate_hashes(value)
    _validate_context(value['build_context'], value['backend'],
                      'build_context')


def validate_execution_identity(value: object) -> dict:
    """Validate a persisted execution identity record; return it unchanged.

    Pure data validation — never imports a package, loads an extension, or
    opens a recorded path. The caller's record is never mutated. Legacy
    records (no ``identity_schema``) keep their historical contract; v2
    records add ``identity_schema=2`` and the platform/numerics context.
    Any other ``identity_schema`` — including booleans masquerading as
    integer versions — is a named failure.
    """
    if not isinstance(value, dict):
        raise ValueError(
            f'execution identity record must be an object, found {value!r}')
    if IDENTITY_SCHEMA_FIELD not in value:
        _validate_legacy(value)
        return value
    schema = value[IDENTITY_SCHEMA_FIELD]
    if not _is_int(schema) or schema != IDENTITY_SCHEMA_VERSION:
        raise ValueError(
            f'unsupported {IDENTITY_SCHEMA_FIELD} {schema!r}; expected '
            f'integer {IDENTITY_SCHEMA_VERSION}')
    _validate_v2(value)
    return value


# ---------------------------------------------------------------------------
# Recursive field-path diff — stable key order, indexed list positions
# ---------------------------------------------------------------------------


def _diff(left, right, prefix='execution'):
    """Stable-order structural diff producing dotted field paths."""
    if isinstance(left, dict) and isinstance(right, dict):
        result = []
        for key in sorted(set(left) | set(right)):
            path = f'{prefix}.{key}'
            if key not in left or key not in right:
                result.append(f'{path}: field is present in only one identity')
            else:
                result.extend(_diff(left[key], right[key], path))
        return result
    if isinstance(left, list) and isinstance(right, list):
        result = []
        for index in range(max(len(left), len(right))):
            path = f'{prefix}[{index}]'
            if index >= len(left) or index >= len(right):
                result.append(f'{path}: field is present in only one identity')
            else:
                result.extend(_diff(left[index], right[index], path))
        return result
    if left != right:
        return [f'{prefix}: recorded={left!r}, current={right!r}']
    return []


def execution_identity_differences(recorded: dict, current: dict) -> list[str]:
    """Dotted-field differences between two validated identity records.

    Both inputs are validated first — an invalid record raises ``ValueError``
    naming the offending field rather than a diff entry. Scalars report
    ``recorded=...``/``current=...``; missing keys and list lengths report the
    field path that exists on only one side.
    """
    return _diff(validate_execution_identity(recorded),
                 validate_execution_identity(current))


def require_execution_compatible(recorded: dict, current: dict) -> None:
    """Fail closed when the recorded and current identities differ.

    Checked-replay gate: a legacy recording against a v2 runtime reports the
    missing ``identity_schema``/context fields rather than projecting them
    away; any provider/platform/numerics drift names its exact field.
    """
    differences = execution_identity_differences(recorded, current)
    if differences:
        raise ValueError(
            'execution identity mismatch: ' + '; '.join(differences))
