"""Numerics provider identity: the probe's output, the declarations generator
and the Python-free root graph's configure-time probe.

native/cmake/BikeProviderIdentity.cmake generates numerics_provider_policy_json
and required_provider_images for both graphs. The pure tests drive it through
``cmake -P`` with fixture records, so they run on any host that has CMake. The
Darwin tests configure real projects: the probe compiles and runs the runtime's
discovery code against Accelerate, and the root graph's identity must equal the
legacy graph's on the same machine.
"""
from __future__ import annotations

import importlib.util
import json
import os
import platform
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from interpreter_shim import interpreter_shim_env, squashed

REPO_ROOT = Path(__file__).resolve().parents[2]
MODULE = REPO_ROOT / "native" / "cmake" / "BikeProviderIdentity.cmake"
NUMERICS_LOCK = REPO_ROOT / "native" / "numerics.lock.json"
BUILD_ID_VECTORS = REPO_ROOT / "tests" / "fixtures" / "portability" / "build_id_vectors.json"

ACCELERATE_DIR = "/System/Library/Frameworks/Accelerate.framework/Versions/A"
BLAS_IMAGE = f"{ACCELERATE_DIR}/Frameworks/vecLib.framework/Versions/A/libBLAS.dylib"
LAPACK_IMAGE = f"{ACCELERATE_DIR}/Frameworks/vecLib.framework/Versions/A/libLAPACK.dylib"
UMBRELLA_IMAGE = f"{ACCELERATE_DIR}/Accelerate"
BLAS_UUID = "31104d4f-4a7e-3fa7-9e18-55937b2011f2"
UMBRELLA_UUID = "a697e7b9-8ff4-356c-a507-64b38af1ccaa"
BUILD_ID = "macos-26A434+accelerate-a697e7b9-8ff4-356c-a507-64b38af1ccaa+sdk-26A425"
RECORD_KEY = "darwin-arm64"
SHA256 = "ab" * 32


def test_committed_provider_records_qualify_every_bound_operation():
    """Generated records must cover the adapter surface on both required lanes.

    This gate deliberately rejects a legacy four-op lock; records are
    regenerated with the real per-platform probe, never completed by a test.
    """
    from tools.native_dependencies import load_numerics_lock
    records = load_numerics_lock(NUMERICS_LOCK)["records"]
    operations = {
        "ddot", "dgemv", "dgemm", "dgelsd", "dgesv", "dsyevd",
        "ddot_ilp64", "zdotu_sub",
    }
    for key in ("darwin-arm64", "linux-aarch64"):
        assert set(records[key]["numerics"]["symbols"]) == operations
    darwin = records["darwin-arm64"]["numerics"]
    assert darwin["blas_integer_bits"] == 32
    assert darwin["symbols"]["ddot"] == "cblas_ddot"
    assert darwin["symbols"]["ddot_ilp64"] == "cblas_ddot$NEWLAPACK$ILP64"
    linux = records["linux-aarch64"]["numerics"]
    assert linux["blas_integer_bits"] == 64
    assert linux["symbols"]["ddot"] == linux["symbols"]["ddot_ilp64"]


def _cmake_quote(text: str) -> str:
    """A CMake quoted argument that carries ``text`` verbatim, tabs and newlines included."""
    escaped = (text.replace("\\", "\\\\").replace('"', '\\"')
               .replace("\t", "\\t").replace("\n", "\\n"))
    return f'"{escaped}"'


def _cmake(tmp_path: Path, body: str) -> subprocess.CompletedProcess:
    """Run ``body`` in CMake script mode, with the identity module included."""
    cmake = shutil.which("cmake")
    if cmake is None:
        pytest.skip("cmake is not installed")
    script = tmp_path / "identity-harness.cmake"
    script.write_text(f'include("{MODULE}")\n{body}\n', encoding="utf-8")
    return subprocess.run(
        [cmake, "-P", str(script)], capture_output=True, text=True, timeout=300)


def _accelerate_record() -> dict:
    """The committed darwin-arm64 facts the declarations read: role,
    identity_kind and sha256 per provider, in record order."""
    return {"records": {RECORD_KEY: {"numerics": {
        "backend": "ACCELERATE",
        "blas_integer_bits": 32,
        "lapack_integer_bits": 64,
        "library_paths": [
            {"path": BLAS_IMAGE, "role": "blas",
             "identity_kind": "apple-system", "sha256": None},
            {"path": LAPACK_IMAGE, "role": "lapack",
             "identity_kind": "apple-system", "sha256": None},
        ],
    }}}}


def _openblas_record() -> dict:
    """A Linux wheel-provider entry: a file identity with a SHA-256 and no build_id."""
    return {"records": {"linux-x86_64": {"numerics": {
        "backend": "NUMPY_OPENBLAS",
        "blas_integer_bits": 64,
        "lapack_integer_bits": 64,
        "library_paths": [
            {"path": "numpy.libs/libscipy_openblas64_.so", "role": "blas_lapack",
             "identity_kind": "file", "sha256": SHA256},
        ],
    }}}}


def _declarations(tmp_path: Path, record: dict, key: str, *, backend: str,
                  bits: tuple[int, int], paths: list[str], build_ids: list[str]) -> str:
    """Generate the declarations through bike_provider_identity_declarations()."""
    record_file = tmp_path / "numerics.lock.json"
    record_file.write_text(json.dumps(record), encoding="utf-8")
    out = tmp_path / "declarations.txt"
    path_args = " ".join(_cmake_quote(path) for path in paths)
    id_args = " ".join(_cmake_quote(build_id) for build_id in build_ids)
    result = _cmake(tmp_path, f"""
file(READ "{record_file}" _record_json)
set(_paths {path_args})
set(_ids {id_args})
bike_provider_identity_declarations(_decl
  RECORD_VAR _record_json RECORD_KEY "{key}"
  BACKEND "{backend}" BLAS_BITS {bits[0]} LAPACK_BITS {bits[1]}
  PROVIDER_PATHS_VAR _paths BUILD_IDS_VAR _ids)
file(WRITE "{out}" "${{_decl}}")
""")
    assert result.returncode == 0, result.stderr
    return out.read_text(encoding="utf-8")


def _parse_declarations(text: str) -> tuple[dict, list[str]]:
    """The policy object and the image list, as the runtime reads them back."""
    policy = re.search(
        r'numerics_provider_policy_json\[\] = R"identity\((.*?)\)identity";', text)
    images = re.search(r'required_provider_images\[\] = "(.*)";', text)
    assert policy and images, text
    # The header carries newline separators as the two characters \n.
    return json.loads(policy.group(1)), images.group(1).split("\\n")


def test_probe_text_yields_the_reported_uuid(tmp_path):
    """The umbrella's LC_UUID is read from the probe line that names it, not
    from the neighbouring image lines."""
    probe = f"{BLAS_IMAGE}\t{BLAS_UUID}\n{UMBRELLA_IMAGE}\t{UMBRELLA_UUID}\n"
    out = tmp_path / "uuid.txt"
    result = _cmake(tmp_path, f"""
bike_provider_probe_image_uuid(_uuid IMAGE "{UMBRELLA_IMAGE}" PROBE {_cmake_quote(probe)})
file(WRITE "{out}" "${{_uuid}}")
""")
    assert result.returncode == 0, result.stderr
    assert out.read_text(encoding="utf-8") == UMBRELLA_UUID


def test_probe_text_fails_for_an_image_it_did_not_report(tmp_path):
    """A required image missing from the probe output is a named failure, not a
    build_id taken from some other image."""
    result = _cmake(tmp_path, f"""
bike_provider_probe_image_uuid(_uuid IMAGE "{UMBRELLA_IMAGE}"
  PROBE {_cmake_quote(f"{BLAS_IMAGE}\t{BLAS_UUID}\n")})
""")
    assert result.returncode != 0
    assert "did not report the required image" in squashed(result.stderr)


def test_probe_text_fails_for_an_image_without_an_lc_uuid(tmp_path):
    """A '-' uuid means the image has no LC_UUID; the apple-system identity
    must not be formed from it."""
    result = _cmake(tmp_path, f"""
bike_provider_probe_image_uuid(_uuid IMAGE "{UMBRELLA_IMAGE}"
  PROBE {_cmake_quote(f"{UMBRELLA_IMAGE}\t-\n")})
""")
    assert result.returncode != 0
    assert "has no LC_UUID" in squashed(result.stderr)


@pytest.mark.parametrize("uuid", [
    "a697e7b9-8ff4-356c-a507-64b38af1cca",     # last group one digit short
    "A697E7B9-8FF4-356C-A507-64B38AF1CCAA",    # the probe prints lowercase only
    "a697e7b98ff4356ca50764b38af1ccaa",        # no separators
])
def test_probe_text_rejects_a_malformed_uuid(tmp_path, uuid):
    result = _cmake(tmp_path, f"""
bike_provider_probe_image_uuid(_uuid IMAGE "{UMBRELLA_IMAGE}"
  PROBE {_cmake_quote(f"{UMBRELLA_IMAGE}\t{uuid}\n")})
""")
    assert result.returncode != 0
    assert "malformed LC_UUID" in squashed(result.stderr)


def test_declarations_reproduce_the_accelerate_identity(tmp_path):
    """The Accelerate policy takes role, identity_kind and sha256 from the
    record, the path from the bound list and build_id from the probe; the
    required images are the bound providers plus the umbrella."""
    text = _declarations(
        tmp_path, _accelerate_record(), RECORD_KEY, backend="ACCELERATE",
        bits=(32, 64), paths=[BLAS_IMAGE, LAPACK_IMAGE],
        build_ids=[BUILD_ID, BUILD_ID])
    policy, images = _parse_declarations(text)
    assert policy == {
        "backend": "ACCELERATE",
        "blas_integer_bits": 32,
        "lapack_integer_bits": 64,
        "libraries": [
            {"path": BLAS_IMAGE, "role": "blas",
             "identity_kind": "apple-system", "sha256": None,
             "build_id": BUILD_ID},
            {"path": LAPACK_IMAGE, "role": "lapack",
             "identity_kind": "apple-system", "sha256": None,
             "build_id": BUILD_ID},
        ],
    }
    assert images == [BLAS_IMAGE, LAPACK_IMAGE, UMBRELLA_IMAGE]


def test_declarations_write_a_missing_build_id_as_null(tmp_path):
    """The <none> marker means the identity has no build_id, so the policy
    carries null. This is the one-provider file identity, the case where an
    empty element would vanish from the CMake list and misalign the count."""
    provider = "/site/numpy.libs/libscipy_openblas64_.so"
    text = _declarations(
        tmp_path, _openblas_record(), "linux-x86_64", backend="NUMPY_OPENBLAS",
        bits=(64, 64), paths=[provider], build_ids=["<none>"])
    policy, images = _parse_declarations(text)
    assert policy["libraries"] == [{
        "path": provider, "role": "blas_lapack", "identity_kind": "file",
        "sha256": SHA256, "build_id": None,
    }]
    assert images == [provider]


def test_declarations_reject_a_build_id_count_that_does_not_match(tmp_path):
    """One build_id for two bound providers must fail, not pair the first build
    id with both providers."""
    record_file = tmp_path / "numerics.lock.json"
    record_file.write_text(json.dumps(_accelerate_record()), encoding="utf-8")
    result = _cmake(tmp_path, f"""
file(READ "{record_file}" _record_json)
set(_paths {_cmake_quote(BLAS_IMAGE)} {_cmake_quote(LAPACK_IMAGE)})
set(_ids {_cmake_quote(BUILD_ID)})
bike_provider_identity_declarations(_decl
  RECORD_VAR _record_json RECORD_KEY "{RECORD_KEY}"
  BACKEND ACCELERATE BLAS_BITS 32 LAPACK_BITS 64
  PROVIDER_PATHS_VAR _paths BUILD_IDS_VAR _ids)
""")
    assert result.returncode != 0
    assert "has 1 build ids for 2 bound providers" in squashed(result.stderr)


def test_declarations_reject_a_record_that_lists_other_providers(tmp_path):
    record = _accelerate_record()
    record["records"][RECORD_KEY]["numerics"]["library_paths"].pop()
    record_file = tmp_path / "numerics.lock.json"
    record_file.write_text(json.dumps(record), encoding="utf-8")
    result = _cmake(tmp_path, f"""
file(READ "{record_file}" _record_json)
set(_paths {_cmake_quote(BLAS_IMAGE)} {_cmake_quote(LAPACK_IMAGE)})
set(_ids {_cmake_quote(BUILD_ID)} {_cmake_quote(BUILD_ID)})
bike_provider_identity_declarations(_decl
  RECORD_VAR _record_json RECORD_KEY "{RECORD_KEY}"
  BACKEND ACCELERATE BLAS_BITS 32 LAPACK_BITS 64
  PROVIDER_PATHS_VAR _paths BUILD_IDS_VAR _ids)
""")
    assert result.returncode != 0
    assert "lists 1 providers, BikeNumerics bound 2" in squashed(result.stderr)


def _scratch_project(tmp_path: Path, body: str) -> subprocess.CompletedProcess:
    """Configure a minimal CXX project that includes the module on Darwin."""
    cmake = shutil.which("cmake")
    if cmake is None:
        pytest.skip("cmake is not installed")
    project = tmp_path / "probe-project"
    project.mkdir()
    (project / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.24)\n"
        "project(provider_identity_probe_check CXX)\n"
        'set(BIKE_TARGET_SYSTEM "Darwin")\n'
        f'include("{MODULE}")\n'
        f"{body}\n",
        encoding="utf-8")
    return subprocess.run(
        [cmake, "-S", str(project), "-B", str(tmp_path / "probe-build")],
        capture_output=True, text=True, timeout=600)


darwin_only = pytest.mark.skipif(
    platform.system() != "Darwin", reason="Accelerate and LC_UUID are Darwin facts")


@darwin_only
def test_probe_reports_the_loaded_accelerate_images(tmp_path):
    """The configure-time probe compiles the runtime's discovery code, links
    Accelerate, and reports the umbrella with a well-formed LC_UUID."""
    result = _scratch_project(tmp_path, f"""
set(_required "{UMBRELLA_IMAGE}")
bike_provider_probe_loaded_images(_probe REQUIRED_VAR _required)
bike_provider_probe_image_uuid(_uuid IMAGE "{UMBRELLA_IMAGE}" PROBE "${{_probe}}")
message(STATUS "PROBE_UUID=${{_uuid}}")
""")
    assert result.returncode == 0, result.stderr
    assert re.search(r"PROBE_UUID=[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}",
                     result.stdout)


@darwin_only
def test_probe_fails_configure_when_a_required_image_is_not_loaded(tmp_path):
    """The discovery's own check runs in the probe: a required image the
    process did not load fails configure, and no identity is written."""
    missing = f"{ACCELERATE_DIR}/Frameworks/NotAProvider.framework/NotAProvider"
    result = _scratch_project(tmp_path, f"""
set(_required "{missing}")
bike_provider_probe_loaded_images(_probe REQUIRED_VAR _required)
""")
    assert result.returncode != 0
    assert f"qualified numerics provider image is not loaded: {missing}" in squashed(
        result.stderr)


def _build_id_cases() -> list[dict]:
    return json.loads(BUILD_ID_VECTORS.read_text(encoding="utf-8"))["cases"]


def test_build_id_vectors_are_present_and_cover_both_outcomes():
    """The shared vectors must keep both an accepted and a rejected case, or the
    two spellings below could agree by never being asked anything hard."""
    cases = _build_id_cases()
    assert len({case["name"] for case in cases}) == len(cases)
    assert any(case["expected"] is not None for case in cases)
    assert any(case["expected"] is None for case in cases)


@pytest.mark.parametrize("case", _build_id_cases(), ids=lambda case: case["name"])
def test_python_build_id_matches_the_shared_vectors(case):
    """The Python spelling (used by the probe checker and the runtime record)
    returns the vector's expected string, or rejects the component."""
    from bike_sim.native.provider_build_id import accelerate_build_id

    arguments = (case["macos_build"], case["accelerate_uuid"], case["sdk_build"])
    if case["expected"] is None:
        with pytest.raises(ValueError, match="build_id component"):
            accelerate_build_id(*arguments)
    else:
        assert accelerate_build_id(*arguments) == case["expected"]


def test_cmake_build_id_matches_the_shared_vectors(tmp_path):
    """The CMake spelling, run over the same vectors in one `cmake -P` script.
    A rejected component must come back as the empty string, never a value."""
    result_file = tmp_path / "cmake-build-id.txt"
    result = _cmake(tmp_path, f"""
file(READ "{BUILD_ID_VECTORS}" _vectors)
string(JSON _count LENGTH "${{_vectors}}" cases)
math(EXPR _last "${{_count}} - 1")
set(_failures "")
foreach(_i RANGE ${{_last}})
  string(JSON _name GET "${{_vectors}}" cases ${{_i}} name)
  string(JSON _macos GET "${{_vectors}}" cases ${{_i}} macos_build)
  string(JSON _uuid GET "${{_vectors}}" cases ${{_i}} accelerate_uuid)
  string(JSON _sdk GET "${{_vectors}}" cases ${{_i}} sdk_build)
  string(JSON _expected_type TYPE "${{_vectors}}" cases ${{_i}} expected)
  bike_accelerate_build_id(_actual
    MACOS_BUILD "${{_macos}}" SDK_BUILD "${{_sdk}}" UUID "${{_uuid}}")
  if(_expected_type STREQUAL "NULL")
    if(NOT _actual STREQUAL "")
      list(APPEND _failures "${{_name}}: rejected input produced '${{_actual}}'")
    endif()
  else()
    string(JSON _expected GET "${{_vectors}}" cases ${{_i}} expected)
    if(NOT _actual STREQUAL _expected)
      list(APPEND _failures "${{_name}}: '${{_actual}}' != '${{_expected}}'")
    endif()
  endif()
endforeach()
file(WRITE "{result_file}" "count=${{_count}}\\nfailures=${{_failures}}\\n")
""")
    assert result.returncode == 0, result.stderr
    report = result_file.read_text(encoding="utf-8")
    assert f"count={len(_build_id_cases())}\n" in report, report
    assert "failures=\n" in report, report


def test_the_build_id_spelling_has_one_home_per_language():
    """The formula's literal separator sequence appears only in the two
    spellings. A third copy in any consumer would escape the vectors."""
    literal = "+accelerate-"
    python_home = REPO_ROOT / "src" / "bike_sim" / "native" / "provider_build_id.py"
    cmake_home = REPO_ROOT / "native" / "cmake" / "BikeProviderIdentity.cmake"
    scanned = [
        *sorted((REPO_ROOT / "src").rglob("*.py")),
        *sorted((REPO_ROOT / "tools").rglob("*.py")),
        *sorted((REPO_ROOT / "native" / "cmake").glob("*.cmake")),
        *sorted((REPO_ROOT / "native").glob("CMakeLists.txt")),
        REPO_ROOT / "CMakeLists.txt",
    ]
    homes = sorted(
        path.relative_to(REPO_ROOT).as_posix() for path in scanned
        if path.is_file() and literal in path.read_text(encoding="utf-8"))
    assert homes == sorted(
        path.relative_to(REPO_ROOT).as_posix() for path in (python_home, cmake_home)), homes


def _real_environment_available() -> bool:
    return all(
        importlib.util.find_spec(name) is not None
        for name in ("numpy", "threadpoolctl", "mujoco", "nanobind")
    )


def _configure_root_without_python(tmp_path: Path) -> Path:
    """Configure the Python-free root graph with no Python or uv reachable.

    The PATH holds only cmake and ninja, plus logging stubs for the interpreters
    and uv that exit 97. A configure that starts one fails, and the stubs record
    the attempt, so the test checks the absence of Python and not just its output.
    """
    cmake = shutil.which("cmake")
    ninja = shutil.which("ninja")
    if cmake is None or ninja is None:
        pytest.skip("cmake and ninja are required for the root configure")
    if not (REPO_ROOT / ".deps").is_dir():
        pytest.skip("the locked .deps tree is not acquired")
    env, calls = interpreter_shim_env(tmp_path, tools=(cmake, ninja))
    build = tmp_path / "root-build"
    result = subprocess.run(
        [cmake, "-S", str(REPO_ROOT), "-B", str(build), "-G", "Ninja",
         "-DCMAKE_BUILD_TYPE=Release", "-DBIKE_BUILD_PYTHON_BINDINGS=OFF",
         "-DBIKE_BUILD_GUI=OFF", f"-DBIKE_DEPENDENCY_ROOT={REPO_ROOT / '.deps'}"],
        capture_output=True, text=True, timeout=900, env=env)
    assert result.returncode == 0, result.stderr
    assert not calls.exists(), calls.read_text(encoding="utf-8")
    return build


def _configure_legacy_graph(tmp_path: Path) -> Path:
    """Configure the legacy graph (cmake -S native) with this test's interpreter."""
    cmake = shutil.which("cmake")
    if cmake is None:
        pytest.skip("cmake is not installed")
    env = dict(os.environ)
    env["PATH"] = os.pathsep.join([str(Path(sys.executable).parent), env.get("PATH", "")])
    build = tmp_path / "legacy-build"
    result = subprocess.run(
        [cmake, "-S", str(REPO_ROOT / "native"), "-B", str(build),
         f"-DBIKE_NUMERICS_RECORD={NUMERICS_LOCK}"],
        capture_output=True, text=True, timeout=900, env=env)
    assert result.returncode == 0, result.stderr
    return build


def _identity_lines(build: Path) -> tuple[str, str]:
    """The two declarations this task generates, from a configured build tree."""
    headers = list(build.rglob("research_build_identity.hpp"))
    assert len(headers) == 1, headers
    lines = headers[0].read_text(encoding="utf-8").splitlines()
    policy = [line for line in lines if "numerics_provider_policy_json[]" in line]
    images = [line for line in lines if "required_provider_images[]" in line]
    assert len(policy) == 1 and len(images) == 1, lines
    return policy[0], images[0]


@darwin_only
@pytest.mark.skipif(not _real_environment_available(),
                    reason="the legacy graph configure needs the pinned scientific environment")
def test_root_identity_equals_the_legacy_graph_on_this_host(tmp_path):
    """The Python-free root graph and the legacy graph (live probe) generate the
    same policy and image declarations on the same machine. Only the root's
    build_id comes from its own probe, so equality is the proof that the probe
    reads the same provider the live measurement read."""
    root = _configure_root_without_python(tmp_path)
    legacy = _configure_legacy_graph(tmp_path)
    assert _identity_lines(root) == _identity_lines(legacy)
