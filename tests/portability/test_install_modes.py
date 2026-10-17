"""Non-building installation/loader controls for T23.

Fresh processes isolate import caches. Source/wheel-layout fixtures and fake
extension modules exercise selection, diagnostics and no-build behavior; they
do not claim that a wheel was built/installed or a native binary was qualified.
Real editable/wheel installation and Linux/macOS imports remain release gates.
"""
from __future__ import annotations

import importlib.machinery
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import sysconfig
import textwrap

import pytest

from tools import dev


ROOT = Path(__file__).resolve().parents[2]
ARTIFACT_SOURCE = ROOT / "src/bike_sim/native/artifact.py"


def _package_at(path: Path) -> None:
    native = path / "bike_sim/native"
    native.mkdir(parents=True)
    (native.parent / "__init__.py").write_text("", encoding="utf-8")
    (native / "__init__.py").write_text("", encoding="utf-8")
    shutil.copyfile(ARTIFACT_SOURCE, native / "artifact.py")


@pytest.fixture
def source_tree(tmp_path: Path) -> Path:
    root = tmp_path / "source tree"
    _package_at(root / "src")
    (root / "native").mkdir()
    (root / "native/CMakeLists.txt").write_text("# fixture source\n", encoding="utf-8")
    (root / "pyproject.toml").write_text(
        '[dependency-groups]\nnative = ["mujoco==3.12.0", "numpy==2.5.2"]\n',
        encoding="utf-8",
    )
    return root


def _child(source: Path, code: str, *, cwd: Path, arguments=()) -> subprocess.CompletedProcess:
    environment = dict(os.environ)
    for key in ("BIKE_NATIVE_BUILD_PATH", "NATIVE_TEST_BUILD_PATH", "NATIVE_TEST_BUILD_DIR",
                "NATIVE_TEST_PROVENANCE_PATH", "PYTHONPATH"):
        environment.pop(key, None)
    program = f"import sys\nsys.path.insert(0, {str(source)!r})\n" + textwrap.dedent(code)
    return subprocess.run(
        [sys.executable, "-I", "-B", "-c", program, *map(str, arguments)],
        cwd=cwd, env=environment, text=True, capture_output=True, timeout=30, check=False,
    )


_ARTIFACT_SETUP = """
import json
import os
from pathlib import Path
import subprocess
import types
def forbidden(*args, **kwargs):
    raise AssertionError('application import launched a subprocess/build')
subprocess.run = forbidden
from bike_sim.native import artifact
artifact.importlib.metadata.version = lambda name: {'mujoco': '3.12.0', 'numpy': '2.5.2'}[name]
build = Path(sys.argv[1])
expected = build / ('bike_native' + artifact.importlib.machinery.EXTENSION_SUFFIXES[0])
module = types.ModuleType('bike_native')
module.__file__ = str(expected)
module.runtime_identity = lambda: {'native_source_sha256': artifact.native_source_fingerprint()}
# Only the extension import is intercepted: the loader's diagnostic path still
# needs real lazy imports (sysconfig's _sysconfigdata) to work.
_real_import_module = artifact.importlib.import_module
def _import_module(name, package=None):
    if name == 'bike_native':
        return module
    return _real_import_module(name, package)
artifact.importlib.import_module = _import_module
"""


_LOAD_RESULT = """
try:
    loaded = artifact.load_native_extension(build)
except Exception as error:
    print(json.dumps({'status': 'error', 'type': type(error).__name__, 'message': str(error),
                      'cause': type(error.__cause__).__name__ if error.__cause__ else None,
                      'cause_message': str(error.__cause__) if error.__cause__ else None,
                      'cached': getattr(sys.modules.get('bike_native'), '__file__', None)}))
else:
    print(json.dumps({'status': 'pass', 'file': loaded.__file__}))
"""


def _artifact_result(source_tree: Path, tmp_path: Path, extra: str = "", *, count=1) -> dict:
    build = tmp_path / "selected build"
    build.mkdir()
    for suffix in importlib.machinery.EXTENSION_SUFFIXES[:count]:
        (build / f"bike_native{suffix}").write_bytes(b"selection fixture, not a native binary")
    result = _child(source_tree / "src", _ARTIFACT_SETUP + extra + _LOAD_RESULT,
                    cwd=tmp_path, arguments=[build])
    assert result.returncode == 0, result.stderr
    return json.loads(result.stdout)


def test_source_editable_layout_uses_only_selected_artifact(source_tree, tmp_path):
    result = _artifact_result(source_tree, tmp_path)
    assert result["status"] == "pass"
    assert Path(result["file"]).parent == tmp_path / "selected build"


def test_cached_correct_source_artifact_is_a_valid_control(source_tree, tmp_path):
    result = _artifact_result(source_tree, tmp_path, "sys.modules['bike_native'] = module\n")
    assert result["status"] == "pass"


def test_default_source_selection_ignores_working_directory(source_tree, tmp_path):
    result = _child(source_tree / "src", """
        from bike_sim.native import artifact
        print(artifact.selected_build_dir())
    """, cwd=ROOT)
    assert result.returncode == 0, result.stderr
    assert Path(result.stdout.strip()) == source_tree / "native/build/release"


def test_missing_artifact_never_launches_a_build(source_tree, tmp_path):
    result = _artifact_result(source_tree, tmp_path, count=0)
    assert result["type"] == "RuntimeError"
    for term in ("no bike_native extension", "python=", "soabi=", "platform=",
                 "tools/dev.py", "configure", "build"):
        assert term in result["message"]


def test_two_importable_suffixes_fail_before_import(source_tree, tmp_path):
    assert len(importlib.machinery.EXTENSION_SUFFIXES) >= 2
    result = _artifact_result(source_tree, tmp_path, count=2)
    assert result["type"] == "RuntimeError"
    assert "exactly one" in result["message"]
    assert "found 2" in result["message"]


@pytest.mark.parametrize("selector", ["argument", "environment"])
def test_relative_native_selector_is_rejected_in_fresh_process(source_tree, tmp_path, selector):
    statement = ("artifact.load_native_extension(Path('native/build'))" if selector == "argument"
                 else "artifact.selected_build_dir({'BIKE_NATIVE_BUILD_PATH': 'native/build'})")
    result = _child(source_tree / "src", f"""
        from pathlib import Path
        from bike_sim.native import artifact
        try:
            {statement}
        except ValueError as error:
            print(error)
        else:
            raise AssertionError('relative path accepted')
    """, cwd=tmp_path)
    assert result.returncode == 0, result.stderr
    assert "absolute" in result.stdout


def test_wrong_cached_module_is_not_unloaded_or_replaced(source_tree, tmp_path):
    result = _artifact_result(source_tree, tmp_path, """
module.__file__ = str(build.parent / 'other build' / expected.name)
sys.modules['bike_native'] = module
def _no_import(name, package=None):
    if name == 'bike_native':
        forbidden()
    return _real_import_module(name, package)
artifact.importlib.import_module = _no_import
""")
    assert result["type"] == "RuntimeError"
    assert "pre-imported" in result["message"]
    assert "other build" in result["cached"]
    assert "fresh Python process" in result["message"]


@pytest.mark.parametrize("cached", [False, True])
def test_stale_compiled_source_stamp_is_rejected(source_tree, tmp_path, cached):
    extra = "module.runtime_identity = lambda: {'native_source_sha256': '0' * 64}\n"
    if cached:
        extra += "sys.modules['bike_native'] = module\n"
    result = _artifact_result(source_tree, tmp_path, extra)
    assert result["type"] == "ValueError"
    assert "not built from the current native source" in result["message"]


def test_wrong_mujoco_fails_before_import(source_tree, tmp_path):
    result = _artifact_result(source_tree, tmp_path, """
artifact.importlib.metadata.version = lambda name: '3.11.0' if name == 'mujoco' else '2.5.2'
def _no_import(name, package=None):
    if name == 'bike_native':
        forbidden()
    return _real_import_module(name, package)
artifact.importlib.import_module = _no_import
""")
    assert result["type"] == "ValueError"
    assert "mujoco" in result["message"]
    assert "3.12.0" in result["message"]
    assert "3.11.0" in result["message"]


@pytest.mark.parametrize("error_type", ["ImportError", "OSError"])
def test_dynamic_loader_diagnostic_keeps_original_cause(source_tree, tmp_path, error_type):
    result = _artifact_result(source_tree, tmp_path, f"""
def fail_import(name, package=None):
    if name == 'bike_native':
        raise {error_type}('libprovider.so: undefined symbol: scipy_dgelsd_64_')
    return _real_import_module(name, package)
artifact.importlib.import_module = fail_import
""")
    assert result["type"] == "RuntimeError"
    assert result["cause"] == error_type
    assert result["cause_message"] in result["message"]
    for term in ("selected=", "artifact=", "python=", "soabi=", "platform=", "doctor"):
        assert term in result["message"]


def test_removed_compiled_provider_keeps_identity_failure_context(source_tree, tmp_path):
    result = _artifact_result(source_tree, tmp_path, """
def fail_identity():
    raise RuntimeError('qualified numerics provider image is not loaded: /removed/provider.so')
module.runtime_identity = fail_identity
""")
    assert result["type"] == "RuntimeError"
    assert result["cause"] == "RuntimeError"
    assert "/removed/provider.so" in result["message"]
    assert "selected=" in result["message"] and "doctor" in result["message"]


@pytest.mark.parametrize("explicit_build", [False, True])
def test_python_only_wheel_layout_never_guesses_a_checkout(source_tree, tmp_path, explicit_build):
    wheel_site = tmp_path / "wheel/site-packages"
    _package_at(wheel_site)
    build_argument = "Path(sys.argv[1])" if explicit_build else ""
    result = _child(wheel_site, f"""
        from pathlib import Path
        import subprocess
        def forbidden(*args, **kwargs):
            raise AssertionError('wheel import launched a build')
        subprocess.run = forbidden
        from bike_sim.native import artifact
        try:
            artifact.load_native_extension({build_argument})
        except RuntimeError as error:
            print(error)
        else:
            raise AssertionError('wheel silently borrowed a checkout')
    """, cwd=source_tree, arguments=[source_tree / "native/build/release"])
    assert result.returncode == 0, result.stderr
    assert "Python-only" in result.stdout
    assert "editable uv environment" in result.stdout
    assert "py3-none-any" in result.stdout


@pytest.fixture
def wheel_layout(tmp_path):
    """Python payload layout only: no packaging/build backend is invoked."""
    site = tmp_path / "wheel/site-packages"
    shutil.copytree(ROOT / "src/bike_sim", site / "bike_sim",
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "*.so", "*.dylib"))
    return site


def test_wheel_python_reference_backend_does_not_request_native(wheel_layout, tmp_path):
    result = _child(wheel_layout, """
        import subprocess
        # mujoco's package init probes Rosetta translation via sysctl on macOS;
        # that is runtime setup, not a build — import it before the tripwire.
        import mujoco
        def forbidden(*args, **kwargs):
            raise AssertionError('reference route launched a build')
        subprocess.run = forbidden
        from bike_sim.sim.backend import require_backend
        assert require_backend('python', None, None) is None
        assert 'bike_native' not in sys.modules
        print('reference route passed')
    """, cwd=tmp_path)
    assert result.returncode == 0, result.stderr
    assert "reference route passed" in result.stdout


@pytest.mark.parametrize("module", ["ride", "research"])
def test_wheel_help_needs_no_native_binary(wheel_layout, tmp_path, module):
    result = _child(wheel_layout, f"""
        import runpy
        sys.argv = ['bike-{module}', '--help']
        try:
            runpy.run_module('bike_sim.cli.{module}', run_name='__main__')
        except SystemExit as error:
            assert error.code == 0
        assert 'bike_native' not in sys.modules
    """, cwd=tmp_path)
    assert result.returncode == 0, result.stderr
    assert "usage:" in result.stdout.lower()


@pytest.mark.parametrize("removed", [False, True])
def test_removed_mapped_numerical_provider_is_not_reused(source_tree, tmp_path, removed):
    site = tmp_path / "scientific/site-packages"
    numpy = site / "numpy"
    provider = site / "numpy.libs/libscipy_openblas_fixture.so"
    numpy.mkdir(parents=True)
    provider.parent.mkdir()
    if not removed:
        provider.write_bytes(b"provider identity fixture")
    result = _child(source_tree / "src", """
        from pathlib import Path
        from types import SimpleNamespace
        from bike_sim.native import artifact
        numpy = SimpleNamespace(__file__=str(Path(sys.argv[1]) / '__init__.py'))
        provider = Path(sys.argv[2])
        removed = sys.argv[3] == 'True'
        artifact._linux_mapped_files = lambda: [(str(provider), removed)]
        try:
            actual = artifact._numpy_openblas_provider(numpy)
        except ValueError as error:
            assert removed
            assert 'no longer exists' in str(error)
            print('removed provider rejected')
        else:
            assert not removed and actual == provider
            print('existing provider accepted')
    """, cwd=tmp_path, arguments=[numpy, provider, removed])
    assert result.returncode == 0, result.stderr
    assert ("rejected" if removed else "accepted") in result.stdout


def test_reference_loader_adds_context_without_replacing_import_error(source_tree, tmp_path):
    build = tmp_path / "selected build"
    build.mkdir()
    (build / f"bike_native{importlib.machinery.EXTENSION_SUFFIXES[0]}").write_bytes(b"fixture")
    result = _child(source_tree / "src", f"""
        import os
        sys.path.insert(0, {str(ROOT / 'tests/reference')!r})
        import native_loader
        os.environ['NATIVE_TEST_BUILD_PATH'] = sys.argv[1]
        native_loader.verify_sanitizer_runtime = lambda: None
        real_import = native_loader.importlib.import_module
        def fail(name, package=None):
            if name == 'bike_native':
                raise ImportError('missing native-linked library')
            return real_import(name, package)
        native_loader.importlib.import_module = fail
        try:
            native_loader.load_native()
        except RuntimeError as error:
            assert isinstance(error.__cause__, ImportError)
            assert 'missing native-linked library' in str(error)
            assert 'soabi=' in str(error) and 'doctor' in str(error)
        else:
            raise AssertionError('loader failure was ignored')
    """, cwd=tmp_path, arguments=[build])
    assert result.returncode == 0, result.stderr


def _configured_build(root: Path) -> tuple[Path, dict]:
    build = root / "native/build/release"
    build.mkdir(parents=True)
    (build / "CMakeCache.txt").write_text(
        f"CMAKE_HOME_DIRECTORY:INTERNAL={root / 'native'}\n"
        f"CMAKE_CACHEFILE_DIR:INTERNAL={build}\n"
        f"Python_EXECUTABLE:FILEPATH={sys.executable}\n", encoding="utf-8",
    )
    dependency = root / "selected-library"
    dependency.write_bytes(b"recorded dependency")
    context = {
        "schema_version": 2,
        "python_executable": sys.executable,
        "python": {"executable": sys.executable, "version": platform.python_version(),
                   "soabi": sysconfig.get_config_var("SOABI")},
        "source_directory": str(root / "native"), "build_directory": str(build),
        "platform": {"system": platform.system(), "machine": platform.machine()},
        "library_paths": [str(dependency)], "required_files": [str(dependency)],
    }
    return build, context


def test_install_context_matches_source_build_interpreter_and_dependencies(tmp_path):
    build, context = _configured_build(tmp_path)
    dev._validate_install_context(tmp_path, build, context)


@pytest.mark.parametrize("field", ["source_directory", "build_directory", "soabi", "python", "provider"])
def test_doctor_rejects_stale_install_context(tmp_path, field):
    build, context = _configured_build(tmp_path)
    if field in {"source_directory", "build_directory"}:
        context[field] = str(tmp_path / "old checkout")
    elif field == "soabi":
        context["python"]["soabi"] = "cpython-wrong-abi"
    elif field == "python":
        context["python"]["executable"] = str(tmp_path / "another-venv/bin/python")
    else:
        Path(context["library_paths"][0]).unlink()
    with pytest.raises(ValueError):
        dev._validate_install_context(tmp_path, build, context)


def test_old_context_requests_reconfigure_without_changing_its_schema(tmp_path):
    build, context = _configured_build(tmp_path)
    del context["python"]
    with pytest.raises(ValueError, match="reconfigure"):
        dev._validate_install_context(tmp_path, build, context)
    assert context["schema_version"] == 2


def test_copied_build_cache_is_rejected_before_build_process(tmp_path, monkeypatch, capsys):
    root = tmp_path / "source"
    root.mkdir()
    original, _ = _configured_build(root)
    moved = root / "native/build/copied"
    shutil.copytree(original, moved)
    monkeypatch.setattr(dev, "_require_project_interpreter", lambda root: sys.executable)
    monkeypatch.delenv("UV_PROJECT_ENVIRONMENT", raising=False)
    def forbidden(*args, **kwargs):
        raise AssertionError("compiler spawned for a copied build cache")
    monkeypatch.setattr(dev, "_run", forbidden)
    assert dev.main(["build", "--build", str(moved)], root=root) == 2
    assert "relocated build cache" in capsys.readouterr().err


def test_two_venvs_sharing_base_python_are_distinct(tmp_path):
    first = tmp_path / "first/bin/python"
    second = tmp_path / "second/bin/python"
    first.parent.mkdir(parents=True)
    second.parent.mkdir(parents=True)
    first.symlink_to(sys.executable)
    second.symlink_to(sys.executable)
    alias = first.with_name("python3")
    alias.symlink_to(first)
    assert dev._same_interpreter(str(first), str(alias))
    assert not dev._same_interpreter(str(first), str(second))


@pytest.mark.parametrize("custom", [False, True])
def test_project_interpreter_must_belong_to_selected_venv(tmp_path, monkeypatch, custom):
    selected = tmp_path / "custom venv" if custom else tmp_path / ".venv"
    executable = selected / "bin/python"
    executable.parent.mkdir(parents=True)
    executable.symlink_to(sys.executable)
    if custom:
        monkeypatch.setenv("UV_PROJECT_ENVIRONMENT", str(selected))
    else:
        monkeypatch.delenv("UV_PROJECT_ENVIRONMENT", raising=False)
    monkeypatch.setattr(dev.sys, "prefix", str(selected))
    monkeypatch.setattr(dev.sys, "executable", str(executable))
    assert dev._require_project_interpreter(tmp_path) == str(executable)
    monkeypatch.setattr(dev.sys, "prefix", str(tmp_path / "different venv"))
    with pytest.raises(ValueError, match="selected uv environment"):
        dev._require_project_interpreter(tmp_path)


@pytest.mark.parametrize("custom", [False, True])
def test_all_wrappers_preserve_one_absolute_uv_environment(tmp_path, monkeypatch, custom):
    selected = tmp_path / "external venv" if custom else tmp_path / ".venv"
    if custom:
        monkeypatch.setenv("UV_PROJECT_ENVIRONMENT", str(selected))
    else:
        monkeypatch.delenv("UV_PROJECT_ENVIRONMENT", raising=False)
    monkeypatch.setenv("UV_OFFLINE", "1")
    monkeypatch.setattr(dev, "_require_project_interpreter", lambda root: str(selected / "bin/python"))
    monkeypatch.setattr(dev, "_check_build_cache", lambda *args, **kwargs: None)
    (tmp_path / "uv.lock").write_text("version = 1\n", encoding="utf-8")
    calls = []
    def record(argv, *, cwd, env):
        calls.append((argv, cwd, env))
        return 0
    monkeypatch.setattr(dev, "_run", record)
    for argv in (["bootstrap"],
                 ["configure", "--build", "native/build/release", "--config", "Release"],
                 ["build", "--build", "native/build/release"],
                 ["test", "--build", "native/build/release", "--profile", "quick"]):
        assert dev.main(argv, root=tmp_path) == 0
    assert len(calls) == 4
    for _, cwd, environment in calls:
        assert cwd == tmp_path
        assert environment["UV_PROJECT_ENVIRONMENT"] == str(selected)
        assert environment["UV_OFFLINE"] == "1"
    assert f"-DPython_EXECUTABLE={selected / 'bin/python'}" in calls[1][0]


def test_relative_custom_environment_fails_before_any_command(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv("UV_PROJECT_ENVIRONMENT", "relative-venv")
    def forbidden(*args, **kwargs):
        raise AssertionError("command launched with ambiguous environment")
    monkeypatch.setattr(dev, "_run", forbidden)
    assert dev.main(["configure", "--build", "build", "--config", "Release"], root=tmp_path) == 2
    assert "UV_PROJECT_ENVIRONMENT must be an absolute" in capsys.readouterr().err


@pytest.mark.parametrize("native", [False, True])
def test_doctor_probe_does_not_invoke_configure_time_abi_compiler(source_tree, tmp_path, monkeypatch, native):
    (source_tree / "tools").mkdir()
    (source_tree / dev.DEPENDENCY_PROBE).write_text("# fixture\n", encoding="utf-8")
    build = None
    if native:
        build, context = _configured_build(source_tree)
        (build / "native_check_context.json").write_text(json.dumps(context), encoding="utf-8")
        (build / f"bike_native{importlib.machinery.EXTENSION_SUFFIXES[0]}").write_bytes(b"fixture")
    else:
        (source_tree / "selected-library").write_bytes(b"fixture")
    monkeypatch.delenv("UV_PROJECT_ENVIRONMENT", raising=False)
    calls = []
    def capture(argv, **kwargs):
        calls.append(argv)
        assert argv[1] == "-c"
        # Execute the actual doctor child body with package/DSO fixtures.
        # Every process launch from that body is forbidden, so routing back
        # through a compiler-based probe cannot masquerade as read-only.
        setup = """
import ctypes
from pathlib import Path
import subprocess
import types
def forbidden(*args, **kwargs):
    raise AssertionError('doctor child launched a compiler or subprocess')
subprocess.run = forbidden
subprocess.Popen = forbidden
from bike_sim.native import artifact
artifact.check_dependencies = lambda: {'mujoco': '3.12.0', 'numpy': '2.5.2'}
artifact._numerics_evidence = lambda: ({'backend': 'NUMPY_OPENBLAS'}, None)
mujoco = types.ModuleType('mujoco')
mujoco.__file__ = str(Path(sys.argv[1]) / 'mujoco/__init__.py')
mujoco.mj_versionString = lambda: '3.12.0'
sys.modules['mujoco'] = mujoco
dependencies = types.ModuleType('tools.native_dependencies')
dependencies.select_mujoco_library = lambda *args: Path(sys.argv[1]) / 'selected-library'
dependencies.discover_numpy_provider = forbidden
dependencies.probe_dependencies = forbidden
tools = types.ModuleType('tools')
tools.__path__ = []
sys.modules['tools'] = tools
sys.modules['tools.native_dependencies'] = dependencies
ctypes.CDLL = lambda path: types.SimpleNamespace(
    _mjPRIVATE_setTlsLogHandler=object(), _mjPRIVATE_getGlobalLogHandler=object(), mj_makeRawData=object())
if sys.argv[2]:
    module = types.ModuleType('bike_native')
    module.__file__ = str(Path(sys.argv[2]) / ('bike_native' + artifact.importlib.machinery.EXTENSION_SUFFIXES[0]))
    module.runtime_identity = lambda: {
        'native_source_sha256': artifact.native_source_fingerprint(),
        'mujoco_library_path': str(Path(sys.argv[1]) / 'selected-library')}
    sys.modules['bike_native'] = module
    artifact.execution_provenance = lambda backend, *, artifact: {
        'build_context': {'numerics': {'backend': 'NUMPY_OPENBLAS'}},
        'native_source_sha256': module.runtime_identity()['native_source_sha256'],
        'extension_sha256': 'f' * 64}
"""
        result = _child(source_tree / "src", setup + f"\nexec({argv[2]!r})\n",
                        cwd=tmp_path, arguments=argv[3:])
        return {"returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr}
    monkeypatch.setattr(dev, "_capture", capture)
    check, measured = dev._doctor_dependency_probe(source_tree, build)
    assert check["status"] == "pass", check["evidence"]
    assert measured["mujoco"]["version"] == "3.12.0"
    assert measured["numerics"]["backend"] == "NUMPY_OPENBLAS"
    if native:
        assert measured["native"]["compiled_source_sha256"] == measured["native"]["source_sha256"]
        assert Path(measured["native"]["artifact"]).parent == build
    assert len(calls) == 1


def test_doctor_capture_preserves_installer_environment(tmp_path, monkeypatch):
    selected = tmp_path / "installer venv"
    monkeypatch.setenv("UV_PROJECT_ENVIRONMENT", str(selected))
    monkeypatch.setenv("UV_OFFLINE", "1")
    def run(argv, **kwargs):
        assert kwargs["cwd"] == tmp_path
        assert kwargs["env"]["UV_PROJECT_ENVIRONMENT"] == str(selected)
        assert kwargs["env"]["UV_OFFLINE"] == "1"
        return subprocess.CompletedProcess(argv, 0, stdout="{}", stderr="")
    monkeypatch.setattr(dev.subprocess, "run", run)
    assert dev._capture([sys.executable, "-c", "pass"], cwd=tmp_path)["returncode"] == 0


@pytest.mark.parametrize("system,variable", [("darwin", "DYLD_INSERT_LIBRARIES"), ("linux", "LD_PRELOAD")])
def test_doctor_reinjects_selected_sanitizer_for_fresh_python(tmp_path, monkeypatch, system, variable):
    runtime = tmp_path / "selected runtime"
    runtime.write_bytes(b"runtime selection fixture")
    monkeypatch.setattr(dev.sys, "platform", system)
    monkeypatch.delenv("UV_PROJECT_ENVIRONMENT", raising=False)
    monkeypatch.setenv("NATIVE_TEST_SANITIZER_RUNTIME", str(runtime))
    monkeypatch.delenv(variable, raising=False)
    environment = dev._doctor_probe_environment(tmp_path, None)
    assert environment[variable] == str(runtime)
    monkeypatch.setenv(variable, "/retained/other-runtime")
    environment = dev._doctor_probe_environment(tmp_path, None)
    assert environment[variable] == f"{runtime}{os.pathsep}/retained/other-runtime"
    monkeypatch.setenv(variable, str(runtime))
    assert dev._doctor_probe_environment(tmp_path, None)[variable] == str(runtime)


def test_doctor_rejects_missing_selected_sanitizer_before_probe(tmp_path, monkeypatch):
    monkeypatch.delenv("UV_PROJECT_ENVIRONMENT", raising=False)
    monkeypatch.setenv("NATIVE_TEST_SANITIZER_RUNTIME", str(tmp_path / "missing-runtime"))
    with pytest.raises(ValueError, match="sanitizer runtime"):
        dev._doctor_probe_environment(tmp_path, None)
