"""Application-side native artifact selection and verified import.

``BIKE_NATIVE_BUILD_PATH`` (absolute directory) selects the artifact when
set; otherwise the checkout's ``native/build/release`` is the default. The
import verifies the loaded module file is the selected artifact and rejects
a pre-imported ``bike_native`` module that came from anywhere else
(design section 7). Pinned runtime dependencies are checked before the
extension is loaded.
"""
from collections.abc import Mapping
import importlib
import importlib.machinery
import importlib.metadata
import os
from pathlib import Path
import sys
import tomllib
from types import ModuleType

SELECTOR_ENV = 'BIKE_NATIVE_BUILD_PATH'
EXTENSION_MODULE = 'bike_native'

_REPO_ROOT = Path(__file__).resolve().parents[3]
_DEFAULT_BUILD = _REPO_ROOT / 'native' / 'build' / 'release'
# Mirror of the == pins in pyproject.toml's 'native' dependency group;
# required_dependencies() prefers the live file and falls back to this in
# environments where the checkout layout is absent.
_FALLBACK_DEPENDENCIES = {'mujoco': '3.12.0', 'numpy': '2.5.2'}


def selected_build_dir(environ: Mapping[str, str] | None = None) -> Path:
    """Absolute artifact directory from the application selector or default."""
    env = os.environ if environ is None else environ
    raw = env.get(SELECTOR_ENV)
    if raw is None:
        return _DEFAULT_BUILD
    candidate = Path(raw)
    if not candidate.is_absolute():
        raise ValueError(f'{SELECTOR_ENV} must be an absolute path, got {raw!r}')
    return candidate.resolve()


def extension_artifacts(build_dir: Path) -> list[Path]:
    """Every importable bike_native binary in the selected directory."""
    return [
        candidate
        for suffix in importlib.machinery.EXTENSION_SUFFIXES
        if (candidate := build_dir / f'{EXTENSION_MODULE}{suffix}').is_file()
    ]


def required_dependencies() -> dict[str, str]:
    """The == pins of pyproject's 'native' dependency group."""
    pyproject = _REPO_ROOT / 'pyproject.toml'
    try:
        document = tomllib.loads(pyproject.read_text(encoding='utf-8'))
    except (OSError, tomllib.TOMLDecodeError):
        return dict(_FALLBACK_DEPENDENCIES)
    entries = document.get('dependency-groups', {}).get('native', [])
    pins = {}
    for entry in entries:
        name, separator, version = str(entry).partition('==')
        if separator:
            pins[name.strip()] = version.strip()
    return pins or dict(_FALLBACK_DEPENDENCIES)


def check_dependencies(declared: Mapping[str, str] | None = None, *,
                       installed: Mapping[str, str] | None = None) -> dict[str, str]:
    """Reject a runtime environment whose pinned native deps do not match.

    ``declared`` maps package name to required version (defaults to the
    pinned 'native' dependency group); ``installed`` maps name to the actual
    version (defaults to importlib.metadata). A missing package is a
    mismatch, and every failure names the package.
    """
    required = required_dependencies() if declared is None else dict(declared)
    if installed is None:
        resolved = {}
        for name in required:
            try:
                resolved[name] = importlib.metadata.version(name)
            except importlib.metadata.PackageNotFoundError:
                resolved[name] = None
        actual_map = resolved
    else:
        actual_map = dict(installed)
    for name, expected in required.items():
        actual = actual_map.get(name)
        if actual != expected:
            raise ValueError(
                f'native dependency version mismatch: {name} requires '
                f'{expected!r}, installed {actual!r}')
    return actual_map


def _module_path(module: ModuleType) -> Path | None:
    location = getattr(module, '__file__', None)
    return Path(location).resolve() if isinstance(location, str) else None


def load_native_extension(build_dir: Path | None = None) -> ModuleType:
    """Import ``bike_native`` only from the selected build directory."""
    if build_dir is None:
        build = selected_build_dir()
    else:
        build = Path(build_dir)
        if not build.is_absolute():
            raise ValueError(
                f'build_dir must be an absolute path, got {build_dir!r}')
        build = build.resolve()
    artifacts = extension_artifacts(build)
    if len(artifacts) != 1:
        if not artifacts:
            raise RuntimeError(
                f'no {EXTENSION_MODULE} extension in the selected build '
                f'directory {build}; build the Release artifact with: '
                'uv run --frozen --group native cmake -S native -B '
                'native/build/release -DCMAKE_BUILD_TYPE=Release && '
                'uv run --frozen --group native cmake --build '
                'native/build/release -j4')
        raise RuntimeError(
            f'expected exactly one {EXTENSION_MODULE} extension in {build}, '
            f'found {len(artifacts)}')
    expected = artifacts[0].resolve()
    check_dependencies()

    imported = sys.modules.get(EXTENSION_MODULE)
    if imported is not None:
        actual = _module_path(imported)
        if actual != expected:
            raise RuntimeError(
                f'pre-imported {EXTENSION_MODULE} module does not match '
                f'selected build: selected={build}, expected={expected}, '
                f'imported={actual}')
        return imported

    sys.path[:] = [
        entry
        for entry in sys.path
        if not entry or Path(entry).resolve() != build
    ]
    sys.path.insert(0, str(build))
    module = importlib.import_module(EXTENSION_MODULE)
    actual = _module_path(module)
    if actual != expected:
        raise RuntimeError(
            f'native artifact mismatch: selected={build}, '
            f'expected={expected}, imported={actual}')
    return module


EXECUTION_FIELDS = frozenset({
    'backend', 'runtime_schema', 'python_source_sha256', 'native_source_sha256',
    'extension_sha256', 'mujoco_version', 'mujoco_library_sha256', 'build_context',
})
BUILD_CONTEXT_FIELDS = frozenset({'build_type', 'compiler_id', 'compiler_version', 'numerical_flags'})


def _file_digest(path):
    from hashlib import sha256
    digest = sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def native_source_fingerprint(root=None):
    """Same sorted path/content manifest as native/CMakeLists.txt.

    Generated headers, build trees, binaries and third-party reference archives
    are excluded. This digest binds a binary's compile-time stamp to its source
    checkout, independent of the checkout's absolute path.
    """
    from hashlib import sha256
    root = _REPO_ROOT/'native' if root is None else Path(root)
    inputs = {root/'CMakeLists.txt'}
    for directory, extensions in (('src', {'.cpp', '.hpp', '.h', '.in'}),
                                  ('cmake', {'.cmake', '.in'})):
        inputs.update(path for path in (root/directory).rglob('*')
                      if path.is_file() and path.suffix in extensions)
    records = (f'{path.relative_to(root).as_posix()}={_file_digest(path)}\n'
               for path in sorted(inputs, key=lambda p: p.relative_to(root).as_posix()))
    return sha256(''.join(records).encode('utf-8')).hexdigest()


def validate_execution(value):
    """Validate data only: no imports, recorded paths, or policy factories.

    Delegates to the shared versioned-identity validator: legacy records keep
    their historical exact-field contract, ``identity_schema=2`` records are
    checked against the extended schema, and every other version fails
    closed. This function never imports a package or follows a recorded path.
    """
    from bike_sim.native.execution_identity import validate_execution_identity
    return validate_execution_identity(value)


# ---------------------------------------------------------------------------
# Runtime environment identity — measured at call boundaries only (recording
# start/save, replay checks, environment construction); never inside a
# physics step. Every fact is live evidence: kernel/loader inspection,
# provider self-queries and file digests, not an assumed configuration table.
# Tooling-only hostnames, timestamps, roots, absolute paths and PIDs are
# excluded from the persisted record.
# ---------------------------------------------------------------------------

_ACCELERATE_IMAGE_PREFIX = '/System/Library/Frameworks/Accelerate.framework/'
_ACCELERATE_UMBRELLA = _ACCELERATE_IMAGE_PREFIX + 'Versions/A/Accelerate'
_ACCELERATE_REQUIRED = (
    _ACCELERATE_IMAGE_PREFIX + ('Versions/A/Frameworks/vecLib.framework/'
                                'Versions/A/libBLAS.dylib'),
    _ACCELERATE_IMAGE_PREFIX + ('Versions/A/Frameworks/vecLib.framework/'
                                'Versions/A/libLAPACK.dylib'),
    _ACCELERATE_UMBRELLA)


def _check_output(argv) -> str:
    """A short local tool query for a platform fact — the answer is measured
    evidence, never recorded text that is being trusted."""
    import subprocess
    try:
        result = subprocess.run(argv, check=True, capture_output=True,
                                text=True, timeout=30)
    except (OSError, subprocess.SubprocessError) as error:
        raise ValueError(
            f'cannot establish the platform identity: {argv[0]} failed: '
            f'{error}') from error
    value = result.stdout.strip()
    if not value:
        raise ValueError(
            f'cannot establish the platform identity: {argv[0]} reported '
            'empty output')
    return value


def _darwin_os_build() -> str:
    """The actual macOS build identifier from the canonical SystemVersion
    record — the same source ``sw_vers -buildVersion`` reports."""
    import plistlib
    path = Path('/System/Library/CoreServices/SystemVersion.plist')
    try:
        with path.open('rb') as stream:
            document = plistlib.load(stream)
    except (OSError, ValueError) as error:
        raise ValueError(
            f'cannot establish the macOS build identity: {error}') from error
    build = document.get('ProductBuildVersion')
    if not isinstance(build, str) or not build.strip():
        raise ValueError(
            'cannot establish the macOS build identity: no '
            'ProductBuildVersion in the SystemVersion record')
    return build


def _darwin_sysctl(name: str) -> str:
    """A single sysctlbyname string fact via libSystem — used for the CPU
    profile so no shell-out is needed for every provenance capture."""
    import ctypes
    lib = ctypes.CDLL(None)
    size = ctypes.c_size_t(0)
    if (lib.sysctlbyname(name.encode(), None, ctypes.byref(size), None, 0) != 0
            or size.value == 0):
        raise ValueError(
            f'sysctl {name!r} is unavailable — cannot establish the '
            'platform identity')
    buffer = ctypes.create_string_buffer(size.value)
    if lib.sysctlbyname(name.encode(), buffer, ctypes.byref(size), None, 0) != 0:
        raise ValueError(
            f'sysctl {name!r} failed — cannot establish the platform identity')
    value = buffer.value.decode('utf-8', 'replace').strip()
    if not value:
        raise ValueError(
            f'sysctl {name!r} reported an empty value — cannot establish '
            'the platform identity')
    return value


def _darwin_sysctl_int(name: str) -> int:
    """An integer sysctlbyname fact (``hw.*`` counters are binary OIDs, not
    strings)."""
    import ctypes
    lib = ctypes.CDLL(None)
    size = ctypes.c_size_t(0)
    if (lib.sysctlbyname(name.encode(), None, ctypes.byref(size), None, 0) != 0
            or size.value not in (4, 8)):
        raise ValueError(
            f'sysctl {name!r} is unavailable — cannot establish the '
            'platform identity')
    buffer = ctypes.create_string_buffer(size.value)
    if lib.sysctlbyname(name.encode(), buffer, ctypes.byref(size), None, 0) != 0:
        raise ValueError(
            f'sysctl {name!r} failed — cannot establish the platform identity')
    return int.from_bytes(buffer.raw[:size.value], 'little')


def _dyld_images() -> dict:
    """Loaded Mach-O image names -> mach_header addresses, via libSystem's
    public ``_dyld_*`` API — the only way to name the dyld-cache images
    NumPy actually bound, since they do not exist as files on disk."""
    import ctypes
    lib = ctypes.CDLL(None)
    lib._dyld_image_count.restype = ctypes.c_uint32
    lib._dyld_get_image_name.restype = ctypes.c_char_p
    lib._dyld_get_image_name.argtypes = [ctypes.c_uint32]
    lib._dyld_get_image_header.restype = ctypes.c_void_p
    lib._dyld_get_image_header.argtypes = [ctypes.c_uint32]
    images = {}
    for index in range(lib._dyld_image_count()):
        raw = lib._dyld_get_image_name(index)
        if raw:
            header = lib._dyld_get_image_header(index)
            images[os.fsdecode(raw)] = int(header) if header else 0
    return images


def _macho_uuid(header_address) -> str | None:
    """LC_UUID of a loaded Mach-O image — a read-only walk of the load
    commands at the data-object address ``_dyld_get_image_header`` reports."""
    import ctypes
    if not header_address:
        return None
    magic = ctypes.c_uint32.from_address(header_address).value
    if magic != 0xFEEDFACF:  # MH_MAGIC_64 — in-process images are thin
        return None
    ncmds = ctypes.c_uint32.from_address(header_address + 16).value
    offset = header_address + 32  # sizeof(struct mach_header_64)
    for _ in range(ncmds):
        command = ctypes.c_uint32.from_address(offset).value
        size = ctypes.c_uint32.from_address(offset + 4).value
        if size < 8:
            return None  # corrupt command table — never keep walking
        if command == 0x1B:  # LC_UUID
            raw = ctypes.string_at(offset + 8, 16).hex()
            return (f'{raw[0:8]}-{raw[8:12]}-{raw[12:16]}-'
                    f'{raw[16:20]}-{raw[20:32]}')
        offset += size
    return None


def _linux_mapped_files():
    """(path, deleted) pairs currently mapped, per the kernel's own loader
    view in /proc/self/maps — the actually-loaded images, not configured
    search paths."""
    try:
        lines = Path('/proc/self/maps').read_text(encoding='utf-8').splitlines()
    except OSError as error:
        raise ValueError(
            f'cannot inspect the loaded provider images: {error}') from error
    mapped = set()
    for line in lines:
        fields = line.split(None, 5)
        if len(fields) != 6:
            continue
        name = fields[5]
        if name.endswith(' (deleted)'):
            mapped.add((name[:-len(' (deleted)')], True))
        elif name.startswith('/'):
            mapped.add((name, False))
    return mapped


def _numpy_build_dependency(numpy, name) -> str | None:
    """NumPy's own build record for a dependency — what the wheel was built
    against, as the wheel itself reports it."""
    config = getattr(numpy, '__config__', None)
    record = getattr(config, 'CONFIG', None) if config is not None else None
    if not isinstance(record, dict):
        return None
    dependencies = record.get('Build Dependencies')
    entry = dependencies.get(name) if isinstance(dependencies, dict) else None
    result = entry.get('name') if isinstance(entry, dict) else None
    return result if isinstance(result, str) else None


def _numpy_openblas_provider(numpy) -> Path:
    """The wheel's ILP64 scipy-openblas actually mapped into this process —
    resolved to a real file inside the installed NumPy distribution tree."""
    site = Path(numpy.__file__).resolve().parents[1]
    matches = {}
    for raw, deleted in _linux_mapped_files():
        name = Path(raw).name
        if not (name.startswith('libscipy_openblas') and name.endswith('.so')):
            continue
        try:
            resolved = Path(raw).resolve(strict=not deleted)
            relative = resolved.relative_to(site)
        except (OSError, RuntimeError, ValueError):
            continue
        # The provider must belong to the NumPy distribution records —
        # a system/foreign libscipy_openblas elsewhere in site-packages is
        # not the qualified wheel image.
        if not relative.parts or relative.parts[0] not in ('numpy', 'numpy.libs'):
            continue
        matches[resolved] = deleted
    if len(matches) != 1:
        raise ValueError(
            'unqualified numerical provider: expected exactly one loaded '
            f'scipy-openblas image inside the NumPy distribution {site}, '
            f'found {sorted(str(path) for path in matches) or "none"}')
    provider, deleted = next(iter(matches.items()))
    if deleted or not provider.is_file():
        raise ValueError(
            f'unqualified numerical provider: the loaded image {provider} '
            'no longer exists as a file — a replaced provider cannot be '
            'identified by its former path')
    return provider


def _openblas_query(handle, name, restype):
    """One provider self-query. Names here are fixed literals from the
    qualified scipy-openblas export contract, never data-derived strings."""
    import ctypes
    function = getattr(handle, name, None)
    if function is None:
        raise ValueError(
            f'the loaded provider does not export {name} — cannot '
            'establish its identity')
    function.restype = restype
    function.argtypes = []
    return function()


def _openblas_numerics(numpy):
    """Measured NUMPY_OPENBLAS record plus the provider file path (the path
    itself is diagnostic evidence — it never enters the persisted record)."""
    import ctypes
    for label in ('blas', 'lapack'):
        reported = _numpy_build_dependency(numpy, label)
        if reported != 'scipy-openblas':
            raise ValueError(
                f'unqualified numerical provider: NumPy reports its {label} '
                f'build dependency as {reported!r}, not scipy-openblas — '
                'the wheel profile cannot be assumed')
    # Force the provider mapping before asking the kernel what is loaded.
    _ = numpy.arange(64., dtype=numpy.float64).reshape(8, 8) @ numpy.eye(8)
    provider = _numpy_openblas_provider(numpy)
    handle = ctypes.CDLL(str(provider))
    config = _openblas_query(handle, 'scipy_openblas_get_config64_',
                             ctypes.c_char_p)
    if config is None:
        raise ValueError(
            'unqualified numerical provider: openblas_get_config returned '
            'NULL — the provider cannot describe its own build')
    config_text = config.decode('utf-8', 'replace')
    if 'USE64BITINT' not in config_text:
        raise ValueError(
            f'unqualified numerical provider: the loaded OpenBLAS reports '
            f'{config_text!r} — without USE64BITINT the 64-bit integer ABI '
            'claim has no support')
    core = _openblas_query(handle, 'scipy_openblas_get_corename64_',
                           ctypes.c_char_p)
    cpu_profile = core.decode('utf-8', 'replace').strip() if core else ''
    if not cpu_profile:
        raise ValueError(
            'unqualified numerical provider: the CPU profile the provider '
            'dispatches to could not be established')
    threads = _openblas_query(handle, 'scipy_openblas_get_num_threads64_',
                              ctypes.c_int)
    if not isinstance(threads, int) or threads < 1:
        raise ValueError(
            f'unqualified numerical provider: the effective thread count '
            f'could not be established ({threads!r})')
    parallel = _openblas_query(handle, 'scipy_openblas_get_parallel64_',
                               ctypes.c_int)
    implementation = {0: 'openblas-serial', 1: 'openblas-pthreads',
                      2: 'openblas-openmp'}.get(parallel)
    if implementation is None:
        raise ValueError(
            f'unqualified numerical provider: unknown OpenBLAS threading '
            f'mode {parallel!r}')
    record = {'backend': 'NUMPY_OPENBLAS',
              'numpy_version': str(numpy.__version__),
              'blas_integer_bits': 64, 'lapack_integer_bits': 64,
              'cpu_profile': cpu_profile,
              'threading': {'implementation': implementation,
                            'num_threads': threads},
              'libraries': [{'role': 'blas_lapack', 'identity_kind': 'file',
                             'sha256': _file_digest(provider),
                             'build_id': None}]}
    return record, provider


def _accelerate_threading() -> dict:
    """The Accelerate threading bound — the same quantity
    ``openblas_get_num_threads`` reports on Linux: the maximum width the
    provider may engage, not an instantaneous sample of parked GCD workers
    (which varies run to run and can never anchor an equality check).

    ``VECLIB_MAXIMUM_THREADS`` caps the vecLib pool; without it the bound is
    the machine's performance-core count (``hw.perflevel0.physicalcpu``),
    which is what GCD concedes to compute work. A missing or nonpositive
    bound fails closed rather than guessing a policy.
    """
    cap = os.environ.get('VECLIB_MAXIMUM_THREADS')
    try:
        maximum = _darwin_sysctl_int('hw.perflevel0.physicalcpu')
    except ValueError:
        maximum = _darwin_sysctl_int('hw.physicalcpu')
    if maximum < 1:
        raise ValueError(
            'unqualified numerical provider: the Accelerate threading bound '
            'could not be established')
    if cap is not None:
        try:
            bounded = int(cap)
        except ValueError as error:
            raise ValueError(
                f'unqualified numerical provider: VECLIB_MAXIMUM_THREADS '
                f'{cap!r} is not an integer') from error
        if bounded < 1:
            raise ValueError(
                f'unqualified numerical provider: VECLIB_MAXIMUM_THREADS '
                f'{cap!r} is not positive')
        maximum = min(maximum, bounded)
    return {'implementation': 'accelerate', 'num_threads': maximum}


def _accelerate_numerics(numpy):
    """Measured ACCELERATE record plus the loaded umbrella image UUID (an
    apple-system image has no file to hash — its build_id binds the actual
    macOS build, the loaded provider UUID and the selected SDK)."""
    for label in ('blas', 'lapack'):
        reported = _numpy_build_dependency(numpy, label)
        if reported != 'accelerate':
            raise ValueError(
                f'unqualified numerical provider: NumPy reports its {label} '
                f'build dependency as {reported!r}, not accelerate — the '
                'wheel profile cannot be assumed')
    _ = numpy.arange(64., dtype=numpy.float64).reshape(8, 8) @ numpy.eye(8)
    images = _dyld_images()
    loaded = sorted(name for name in images
                    if name.startswith(_ACCELERATE_IMAGE_PREFIX))
    for required in _ACCELERATE_REQUIRED:
        if required not in images:
            raise ValueError(
                f'unqualified numerical provider: the Accelerate image '
                f'{required} is not loaded — loaded Accelerate images: '
                f'{loaded}')
    uuid = _macho_uuid(images[_ACCELERATE_UMBRELLA])
    if not uuid:
        raise ValueError(
            'unqualified numerical provider: could not read the LC_UUID of '
            'the loaded Accelerate image — the apple-system identity has '
            'no provider evidence')
    sdk_build = _check_output(
        ['xcrun', '--sdk', 'macosx', '--show-sdk-build-version'])
    build_id = (f'macos-{_darwin_os_build()}+accelerate-{uuid}'
                f'+sdk-{sdk_build}')
    record = {'backend': 'ACCELERATE',
              'numpy_version': str(numpy.__version__),
              'blas_integer_bits': 32, 'lapack_integer_bits': 64,
              'cpu_profile': _darwin_sysctl('machdep.cpu.brand_string'),
              'threading': _accelerate_threading(),
              'libraries': [{'role': 'blas_lapack',
                             'identity_kind': 'apple-system',
                             'sha256': None, 'build_id': build_id}]}
    return record, uuid


def _numerics_evidence():
    """(record, evidence) — the measured numerical-provider identity plus a
    verification-side artifact (provider Path on Linux, umbrella UUID on
    Darwin) used only for the native compile-time cross-check."""
    import platform as _platform
    import numpy
    system = _platform.system()
    if system == 'Linux':
        return _openblas_numerics(numpy)
    if system == 'Darwin':
        return _accelerate_numerics(numpy)
    raise ValueError(
        f'numerical identity has no qualified provider profile for '
        f'platform {system!r}')


def _platform_record() -> dict:
    """Interpreter/OS/libc identity of this process — all live facts."""
    import platform as _platform
    import sysconfig
    from bike_sim.native.execution_identity import normalize_machine
    system = _platform.system()
    if system == 'Darwin':
        build = _darwin_os_build()
        libc = {'name': 'libSystem', 'version': build}
        os_build = build
    elif system == 'Linux':
        name, version = _platform.libc_ver()
        if not name.strip() or not version.strip():
            raise ValueError(
                'cannot establish the Linux libc identity of this '
                'interpreter')
        libc = {'name': name, 'version': version}
        os_build = _platform.release()
    else:
        raise ValueError(
            f'numerical identity has no qualified platform profile for '
            f'{system!r}')
    machine = _platform.machine()
    if not isinstance(machine, str) or not machine.strip():
        raise ValueError('cannot establish the machine architecture')
    python_soabi = sysconfig.get_config_var('SOABI')
    if not isinstance(python_soabi, str) or not python_soabi.strip():
        raise ValueError('cannot establish the interpreter SOABI')
    return {'system': system, 'machine': normalize_machine(machine),
            'libc': libc, 'os_build': os_build,
            'python_version': _platform.python_version(),
            'python_soabi': python_soabi}


def _require_provider_consistent(policy: dict, numerics: dict, evidence,
                                 images) -> None:
    """Compile-time qualified provider selection vs the provider actually in
    use — runs at provenance boundaries, before any native physics advance.

    ``policy`` is the baked research_build_identity.hpp provider record;
    ``numerics``/``evidence`` are the measured runtime record; ``images`` is
    the extension's own loader view of the provider actually loaded.
    """
    def _fail(reason):
        raise ValueError(
            'numerics provider selection differs from the qualified build: '
            + reason)

    if not isinstance(policy, dict):
        _fail('the extension exposes no provider policy record')
    if policy.get('backend') != numerics['backend']:
        _fail(f"runtime measured {numerics['backend']!r} but the build "
              f"qualified {policy.get('backend')!r}")
    for field in ('blas_integer_bits', 'lapack_integer_bits'):
        if policy.get(field) != numerics[field]:
            _fail(f'{field} is {numerics[field]!r} but the build qualified '
                  f'{policy.get(field)!r}')
    libraries = policy.get('libraries')
    if not isinstance(libraries, list) or not libraries:
        _fail('the provider policy has no library records')
    if not isinstance(images, list):
        _fail('the extension reported no loaded provider images')
    loaded = {entry.get('path'): entry for entry in images
              if isinstance(entry, dict)}
    recorded = numerics['libraries'][0]
    if numerics['backend'] == 'NUMPY_OPENBLAS':
        qualified = [entry for entry in libraries
                     if entry.get('identity_kind') == 'file']
        if len(qualified) != 1:
            _fail('the policy does not name exactly one wheel-file provider')
        expected = Path(str(qualified[0].get('path') or '')).resolve()
        if str(expected) not in loaded:
            _fail(f'the qualified provider {expected} is not the loaded '
                  f'image {sorted(loaded)}')
        if str(Path(evidence).resolve()) != str(expected):
            _fail(f'NumPy mapped {evidence} — not the qualified provider '
                  f'{expected}')
        if qualified[0].get('sha256') != recorded['sha256']:
            _fail('the loaded provider file content differs from the '
                  'qualified digest')
    else:
        names = set(loaded)
        for entry in libraries:
            path = entry.get('path')
            if path not in names:
                _fail(f'qualified provider image {path} is not loaded')
            if entry.get('build_id') != recorded['build_id']:
                _fail(f'provider {path} identity drifted from the qualified '
                      'build')
        umbrella = loaded.get(_ACCELERATE_UMBRELLA)
        if not isinstance(umbrella, dict) or umbrella.get('macho_uuid') != evidence:
            _fail('the loaded Accelerate image identity does not match the '
                  'runtime measurement')


def execution_provenance(backend: str, *, artifact=None) -> dict:
    """Capture source, environment and actual loaded binary/library content
    at a boundary.

    The record is identity_schema=2: the legacy fields plus the measured
    platform and numerical-provider context. Native creation separately
    checks the compiled source stamp and cross-checks the compile-time
    qualified provider selection against the provider actually loaded now.
    Re-reading current digests at save detects subsequent edits without
    silently rewriting the recorded startup identity.
    """
    import json
    import mujoco
    from bike_sim.native.execution_identity import IDENTITY_SCHEMA_VERSION
    from bike_sim.validation.environment import source_fingerprint
    if backend not in ('python', 'native'):
        raise ValueError('backend must be python or native')
    numerics, evidence = _numerics_evidence()
    context = {'platform': _platform_record(), 'numerics': numerics}
    result = dict(backend=backend, identity_schema=IDENTITY_SCHEMA_VERSION,
                  runtime_schema=1,
                  python_source_sha256=source_fingerprint(Path(__file__).resolve().parents[1]),
                  native_source_sha256=None, extension_sha256=None,
                  mujoco_version=mujoco.mj_versionString(),
                  mujoco_library_sha256=None, build_context=context)
    if backend == 'native':
        module = load_native_extension() if artifact is None else artifact
        location = _module_path(module)
        if location is None or not location.is_file():
            raise ValueError('native provenance requires the loaded extension file')
        identity = module.runtime_identity()
        library = Path(identity['mujoco_library_path']).resolve(strict=True)
        if identity['mujoco_version'] != result['mujoco_version']:
            raise ValueError('native and Python have different loaded MuJoCo versions')
        compiled = json.loads(identity['build_context_json'])
        if set(compiled) != BUILD_CONTEXT_FIELDS:
            raise ValueError(
                'the extension build context does not match the qualified '
                'schema')
        _require_provider_consistent(
            json.loads(identity['provider_policy_json']), numerics, evidence,
            identity['numerics_provider_images'])
        context.update((key, compiled[key]) for key in BUILD_CONTEXT_FIELDS)
        result.update(native_source_sha256=native_source_fingerprint(),
                      extension_sha256=_file_digest(location),
                      mujoco_library_sha256=_file_digest(library))
    return validate_execution(result)
