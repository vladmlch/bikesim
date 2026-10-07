"""native_bench CLI/state contract regressions (native-safety plan tasks R2/R3).

Fixtures are generated per test — a slide-joint chain sized to the case plus
a BIKEST02 state.bin — so no case depends on the tracked artifacts alone.
The executable comes from the selected build (the same
NATIVE_TEST_BUILD_PATH / NATIVE_TEST_BUILD_DIR resolution used by the native
extension tests), which keeps the ASan/UBSan build's native_bench reachable
from this suite.

Bundle fixtures are written through tools/proto_native_bench/state_format.py
serialize_state: an 80-byte little-endian header (magic BIKEST02, version,
flags, nq/nv/na/nu/nbody, payload_bytes, model_crc64, state_crc64) followed by
the binary64 payload. Python-side coverage of the same format lives in
test_native_state_bundle.py; this file exercises the C++ consumer through the
native_bench CLI, including the matched-workload conformance report.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import struct
import subprocess
import sys

import mujoco
import numpy as np
import pytest

from native_loader import load_native, selected_build
from tools.proto_native_bench import state_format


REPO_ROOT = Path(__file__).resolve().parents[2]
TRACKED_ARTIFACTS = REPO_ROOT / 'tools' / 'proto_native_bench' / 'artifacts'
BENCH_PY = REPO_ROOT / 'tools' / 'proto_native_bench' / 'bench.py'

LEGACY_STATE_MAGIC = 0xBEA0
MAX_COUNT = 10_000_000

# BIKEST02 header field offsets (struct '<8s2I8Q', 80 bytes).
HDR_VERSION = 8
HDR_FLAGS = 12
HDR_DIM = {'nq': 16, 'nv': 24, 'na': 32, 'nu': 40, 'nbody': 48}
HDR_PAYLOAD_BYTES = 56
HDR_MODEL_CRC = 64
HDR_STATE_CRC = 72


@pytest.fixture(scope='module')
def bench_executable() -> Path:
    """Select the configured build's native_bench, like every test_native_* module."""
    load_native()  # the selected build must be complete, extension included
    executable = selected_build(os.environ) / 'native_bench'
    if not (executable.is_file() and os.access(executable, os.X_OK)):
        raise RuntimeError(f'native_bench is not built in the selected build: {executable}')
    return executable


def _bench_env() -> dict[str, str]:
    """Subprocess environment; inject the selected sanitizer runtime if set."""
    environment = os.environ.copy()
    runtime_value = environment.get('NATIVE_TEST_SANITIZER_RUNTIME')
    if runtime_value is not None:
        runtime = Path(runtime_value)
        if not runtime.is_absolute() or not runtime.is_file():
            raise RuntimeError(
                'selected sanitizer runtime is not an absolute file: '
                f'{runtime_value!r}'
            )
        preload = 'DYLD_INSERT_LIBRARIES' if sys.platform == 'darwin' else 'LD_PRELOAD'
        existing = environment.get(preload, '')
        environment[preload] = os.pathsep.join(
            [part for part in (existing, str(runtime)) if part]
        )
    return environment


def _run_bench(bench: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(bench), *args],
        cwd=REPO_ROOT,
        env=_bench_env(),
        text=True,
        capture_output=True,
        check=False,
        timeout=120,
    )


def _run_bench_py(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(BENCH_PY), *args],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=False,
        timeout=120,
    )


def _chain_model(dofs: int, actuators: int = 1, welded: int = 0) -> mujoco.MjModel:
    """Deterministic chain: nq = nv = dofs, nu = actuators, nbody = dofs + welded + 1.

    actuators may exceed dofs (motors stack on joints cyclically), which is
    the nu > nv case the explicit actuator->velocity mapping must handle.
    """
    bodies = ''.join(
        f'<body name="link{i}" pos="{i} 0 0">'
        f'<joint name="joint{i}" type="slide" axis="1 0 0"/>'
        f'<geom type="sphere" size="0.1" mass="0.5"/></body>'
        for i in range(dofs)
    ) + ''.join(
        f'<body name="weld{i}" pos="0 0 {-1 - i}"><geom type="box" size="0.1 0.1 0.1"/></body>'
        for i in range(welded)
    )
    motors = ''.join(
        f'<motor name="motor{i}" joint="joint{i % dofs}"/>'
        for i in range(actuators if dofs else 0)
        # dofs == 0 cannot carry motors; actuators > dofs wraps joints
    )
    actuator_xml = f'<actuator>{motors}</actuator>' if motors else ''
    return mujoco.MjModel.from_xml_string(
        f'<mujoco><worldbody>{bodies}</worldbody>{actuator_xml}</mujoco>'
    )


def _history_model() -> mujoco.MjModel:
    """Model whose mjData allocation the engine boundary must reject.

    nsample>0 gives nhistory>0; a nonpositive timestep then trips the guarded
    makeData precondition — the case that would reach MuJoCo's fatal
    'history buffers require positive timestep' without the E1 check.
    """
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="link">'
        '<joint name="j0" type="slide"/>'
        '<geom type="sphere" size="0.1" mass="0.5"/></body></worldbody>'
        '<sensor><jointpos joint="j0" nsample="3"/></sensor></mujoco>'
    )
    model.opt.timestep = -0.5
    return model


def _state_arrays(
    model: mujoco.MjModel, *, nonfinite: bool = False
) -> dict[str, np.ndarray]:
    """Deterministic payload arrays for `model`'s dimensions — no MjData
    needed, so models whose data allocation would fail can still produce
    fixtures."""
    nq, nv, na, nu, nbody = (
        model.nq, model.nv, model.na, model.nu, model.nbody
    )
    qpos = np.linspace(0.01, 0.02, nq) if nq else np.zeros(0)
    if nonfinite:
        if nq:
            qpos[0] = np.nan
        else:
            qpos = np.array([np.nan])
    return {
        'time': np.array([np.nan if (nonfinite and not nq) else 0.25]),
        'qpos': qpos,
        'qvel': np.linspace(-0.01, 0.01, nv) if nv else np.zeros(0),
        'act': np.zeros(na),
        'ctrl': np.linspace(0.1, 0.2, nu) if nu else np.zeros(0),
        'qfrc_applied': np.linspace(0.001, 0.002, nv) if nv else np.zeros(0),
        'xfrc_applied': np.zeros((nbody, 6)),
        'qacc_warmstart': np.linspace(0.0, 0.004, nv) if nv else np.zeros(0),
    }


def _manifest_stub(model: mujoco.MjModel) -> str:
    """Minimal manifest for synthetic bundles — real generation provenance
    is covered by test_native_state_bundle through write_bundle."""
    return json.dumps({
        'format': 1,
        'model': {
            'nq': model.nq, 'nv': model.nv, 'na': model.na,
            'nu': model.nu, 'nbody': model.nbody,
            'timestep': float(model.opt.timestep),
        },
    })


def _write_bundle(
    root: Path,
    model: mujoco.MjModel,
    *,
    state: bytes | None = None,
    nonfinite: bool = False,
) -> Path:
    """Concrete bundle directory: model.mjb + state.bin + state.npz +
    manifest.json, all self-consistent unless `state`/`nonfinite` corrupt it."""
    root.mkdir(parents=True, exist_ok=True)
    mujoco.mj_saveModel(model, str(root / 'model.mjb'))
    model_bytes = (root / 'model.mjb').read_bytes()
    arrays = _state_arrays(model, nonfinite=nonfinite)
    if state is None:
        state = state_format.serialize_state(arrays, model_bytes)
    (root / 'state.bin').write_bytes(state)
    np.savez(root / 'state.npz', **arrays)
    (root / 'manifest.json').write_text(_manifest_stub(model))
    return root


def _generated_bundle(root: Path, model: mujoco.MjModel) -> Path:
    """A real versioned bundle through the production writer + pointer."""
    data = mujoco.MjData(model)
    if model.nq:
        data.qpos[:] = np.linspace(0.01, 0.02, model.nq)
    if model.nv:
        data.qvel[:] = np.linspace(-0.01, 0.01, model.nv)
        data.qfrc_applied[:] = np.linspace(0.001, 0.002, model.nv)
        data.qacc_warmstart[:] = np.linspace(0.0, 0.004, model.nv)
    if model.nu:
        data.ctrl[:] = np.linspace(0.1, 0.2, model.nu)
    return state_format.write_bundle(
        root, model, data, name='test-bundle',
        source_track='test', source_config='test', warmup_steps=0,
    )


def _rewrite_state(bundle: Path, transform) -> None:
    state_path = bundle / 'state.bin'
    state_path.write_bytes(transform(state_path.read_bytes()))


def _patch_u64(raw: bytes, offset: int, value: int) -> bytes:
    patched = bytearray(raw)
    struct.pack_into('<Q', patched, offset, value)
    return bytes(patched)


def _patch_u32(raw: bytes, offset: int, value: int) -> bytes:
    patched = bytearray(raw)
    struct.pack_into('<I', patched, offset, value)
    return bytes(patched)


def _legacy_state_bytes(model: mujoco.MjModel) -> bytes:
    """Retired 0xBEA0 payload — kept only to prove the reader rejects it."""
    nq, nv, na, nu, nbody = (
        model.nq, model.nv, model.na, model.nu, model.nbody
    )
    payload = struct.pack('<d', 0.25)
    payload += np.linspace(0.01, 0.02, nq).tobytes()
    payload += np.linspace(-0.01, 0.01, nv).tobytes()
    payload += np.zeros(na).tobytes()
    payload += np.linspace(0.1, 0.2, nu).tobytes()
    payload += np.linspace(0.001, 0.002, nv).tobytes()
    payload += np.zeros(6 * nbody).tobytes()
    return struct.pack('<6i', LEGACY_STATE_MAGIC, nq, nv, na, nu, nbody) + payload


# ---------------------------------------------------------------------------
# CLI contract
# ---------------------------------------------------------------------------

def test_help_prints_usage(bench_executable: Path) -> None:
    result = _run_bench(bench_executable, '--help')
    assert result.returncode == 0
    assert 'usage' in result.stdout.lower()


def test_selftest_prints_crc64_check_vector(bench_executable: Path) -> None:
    """CRC64-ECMA('123456789') == 0x6C40DF5F0B497347 — cross-checks the
    native implementation against the published parameter set."""
    result = _run_bench(bench_executable, '--selftest')
    assert result.returncode == 0
    assert '0x6c40df5f0b497347' in result.stdout


def test_mode_is_required(bench_executable: Path) -> None:
    result = _run_bench(bench_executable)
    assert result.returncode == 1
    assert 'mode' in result.stderr


@pytest.mark.parametrize('mode', ['Glue', 'spin', '0', '--mode'])
def test_unknown_mode_is_rejected(
    bench_executable: Path, tmp_path: Path, mode: str
) -> None:
    result = _run_bench(bench_executable, mode, '--artifacts', str(tmp_path))
    assert result.returncode == 1
    assert result.stderr


@pytest.mark.parametrize(
    'steps', ['abc', '1x', '1.5', '0', '-3', '+5', str(MAX_COUNT + 1),
              '99999999999999999999']
)
def test_malformed_steps_are_rejected(
    bench_executable: Path, tmp_path: Path, steps: str
) -> None:
    result = _run_bench(
        bench_executable, 'bare', steps, '--artifacts', str(tmp_path)
    )
    assert result.returncode == 1
    assert 'steps' in result.stderr


@pytest.mark.parametrize(
    'rewind', ['0', '-1', 'abc', '1x', '1.5', '+2', str(MAX_COUNT + 1)]
)
def test_invalid_rewind_is_rejected(
    bench_executable: Path, tmp_path: Path, rewind: str
) -> None:
    result = _run_bench(
        bench_executable, 'glue', '2', rewind, '--artifacts', str(tmp_path)
    )
    assert result.returncode == 1
    assert 'rewind' in result.stderr


def test_missing_artifacts_flag_is_rejected(bench_executable: Path) -> None:
    result = _run_bench(bench_executable, 'bare', '5', '2')
    assert result.returncode == 1
    assert 'artifacts' in result.stderr


def test_artifacts_requires_a_directory(bench_executable: Path) -> None:
    result = _run_bench(bench_executable, 'bare', '--artifacts')
    assert result.returncode == 1
    assert 'artifacts' in result.stderr


def test_artifacts_must_be_a_directory(
    bench_executable: Path, tmp_path: Path
) -> None:
    not_a_dir = tmp_path / 'plain.bin'
    not_a_dir.write_bytes(b'x')
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(not_a_dir))
    assert result.returncode == 1
    assert 'model.mjb' in result.stderr


def test_unknown_option_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    result = _run_bench(
        bench_executable, 'bare', '--speed', '--artifacts', str(tmp_path)
    )
    assert result.returncode == 1
    assert 'option' in result.stderr or 'unknown' in result.stderr


def test_duplicate_artifacts_flag_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    result = _run_bench(
        bench_executable, 'bare', '--artifacts', str(tmp_path),
        '--artifacts', str(tmp_path),
    )
    assert result.returncode == 1


def test_extra_positional_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    result = _run_bench(
        bench_executable, 'bare', '1', '2', '3', '--artifacts', str(tmp_path)
    )
    assert result.returncode == 1


# ---------------------------------------------------------------------------
# Artifact root / pointer / bundle-member validation
# ---------------------------------------------------------------------------

def test_missing_state_bin_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    mujoco.mj_saveModel(_chain_model(2), str(tmp_path / 'model.mjb'))
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(tmp_path))
    assert result.returncode == 1
    assert 'state.bin' in result.stderr


def test_missing_state_npz_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _write_bundle(tmp_path, _chain_model(2))
    (bundle / 'state.npz').unlink()
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'state.npz' in result.stderr


def test_missing_manifest_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _write_bundle(tmp_path, _chain_model(2))
    (bundle / 'manifest.json').unlink()
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'manifest.json' in result.stderr


def test_missing_model_mjb_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    (tmp_path / 'state.bin').write_bytes(b'x')
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(tmp_path))
    assert result.returncode == 1
    assert 'model.mjb' in result.stderr


def test_pointer_root_resolves_to_bundle(
    bench_executable: Path, tmp_path: Path
) -> None:
    """--artifacts may name the artifacts ROOT: current.json is resolved once."""
    _generated_bundle(tmp_path / 'artifacts', _chain_model(3))
    result = _run_bench(
        bench_executable, 'bare', '3', '2',
        '--artifacts', str(tmp_path / 'artifacts'),
    )
    assert result.returncode == 0, result.stderr
    assert 'mode=bare' in result.stdout


def test_concrete_bundle_dir_resolves_directly(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _generated_bundle(tmp_path / 'artifacts', _chain_model(3))
    result = _run_bench(
        bench_executable, 'glue', '3', '2', '--artifacts', str(bundle)
    )
    assert result.returncode == 0, result.stderr
    assert 'mode=glue' in result.stdout


def test_malformed_pointer_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    """A half-written current.json must fail loudly, never fall back to a
    guess — interrupted publication is part of the contract."""
    bundle = _generated_bundle(tmp_path / 'artifacts', _chain_model(2))
    (tmp_path / 'artifacts' / 'current.json').write_text('{not json')
    result = _run_bench(
        bench_executable, 'bare', '--artifacts', str(tmp_path / 'artifacts')
    )
    assert result.returncode == 1
    assert 'current.json' in result.stderr


def test_pointer_to_missing_bundle_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _generated_bundle(tmp_path / 'artifacts', _chain_model(2))
    (tmp_path / 'artifacts' / 'current.json').write_text(
        json.dumps({'format': 1, 'bundle': 'deleted-generation'})
    )
    result = _run_bench(
        bench_executable, 'bare', '--artifacts', str(tmp_path / 'artifacts')
    )
    assert result.returncode == 1
    assert 'missing bundle' in result.stderr


def test_pointer_name_cannot_escape_root(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _generated_bundle(tmp_path / 'artifacts', _chain_model(2))
    for hostile in ('../escape', '/abs', 'a/b'):
        (tmp_path / 'artifacts' / 'current.json').write_text(
            json.dumps({'format': 1, 'bundle': hostile})
        )
        result = _run_bench(
            bench_executable, 'bare', '--artifacts',
            str(tmp_path / 'artifacts'),
        )
        assert result.returncode == 1, hostile
        assert 'bundle name' in result.stderr, hostile


def test_corrupt_model_mjb_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    model = _chain_model(2)
    bundle = _write_bundle(tmp_path, model)
    raw = bytearray((bundle / 'model.mjb').read_bytes())
    # The MJB leading int (54321) guards the size block; corrupting it must be
    # a clean load failure through the engine boundary, not an abort.
    struct.pack_into('<i', raw, 0, 42)
    (bundle / 'model.mjb').write_bytes(bytes(raw))
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1


def test_corrupt_model_dimensions_are_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    model = _chain_model(2)
    bundle = _write_bundle(tmp_path, model)
    raw = bytearray((bundle / 'model.mjb').read_bytes())
    # mjtSize nv slot in the MJB size block: 5-i32 header, nv is sizes[1].
    struct.pack_into('<q', raw, 20 + 8, 9_999_999)
    (bundle / 'model.mjb').write_bytes(bytes(raw))
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1


def test_make_data_failure_is_recoverable(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _write_bundle(tmp_path, _history_model())
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'timestep' in result.stderr or 'history' in result.stderr


# ---------------------------------------------------------------------------
# state.bin validation
# ---------------------------------------------------------------------------

def test_legacy_state_format_is_rejected_with_hint(
    bench_executable: Path, tmp_path: Path
) -> None:
    """A 0xBEA0 file must be rejected with a regeneration instruction."""
    bundle = _write_bundle(
        tmp_path, _chain_model(2), state=_legacy_state_bytes(_chain_model(2))
    )
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'retired' in result.stderr or 'regenerate' in result.stderr


def test_bad_state_magic_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _write_bundle(tmp_path, _chain_model(2))
    _rewrite_state(bundle, lambda raw: b'\x00' * 8 + raw[8:])
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'format' in result.stderr or 'magic' in result.stderr


def test_unknown_state_version_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _write_bundle(tmp_path, _chain_model(2))
    _rewrite_state(bundle, lambda raw: _patch_u32(raw, HDR_VERSION, 7))
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'version' in result.stderr


def test_unknown_state_flags_are_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _write_bundle(tmp_path, _chain_model(2))
    _rewrite_state(bundle, lambda raw: _patch_u32(raw, HDR_FLAGS, 1))
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'flags' in result.stderr


def test_truncated_state_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _write_bundle(tmp_path, _chain_model(2))
    _rewrite_state(bundle, lambda raw: raw[:-16])
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1


def test_truncated_state_header_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _write_bundle(tmp_path, _chain_model(2))
    _rewrite_state(bundle, lambda raw: raw[:20])
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'truncated' in result.stderr


def test_trailing_state_bytes_are_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _write_bundle(tmp_path, _chain_model(2))
    _rewrite_state(bundle, lambda raw: raw + b'\x00' * 8)
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'expected' in result.stderr


def test_huge_header_count_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    """u64-max nq overflows the signed mjtSize domain — rejected before any
    allocation, never fed to a sized buffer."""
    bundle = _write_bundle(tmp_path, _chain_model(2))
    _rewrite_state(
        bundle, lambda raw: _patch_u64(raw, HDR_DIM['nq'], 0xFFFFFFFFFFFFFFFF)
    )
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert (
        'domain' in result.stderr
        or 'does not match' in result.stderr
        or 'mismatch' in result.stderr
    )


def test_excessive_header_count_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    """A huge positive count mismatches the model and must be rejected before
    any buffer is sized — never fed to an allocation."""
    bundle = _write_bundle(tmp_path, _chain_model(2))
    _rewrite_state(
        bundle, lambda raw: _patch_u64(raw, HDR_DIM['nq'], 2_000_000_000)
    )
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'does not match' in result.stderr or 'mismatch' in result.stderr


@pytest.mark.parametrize('label', ['nq', 'nv', 'na', 'nu', 'nbody'])
def test_state_model_dimension_mismatch_is_rejected(
    bench_executable: Path, tmp_path: Path, label: str
) -> None:
    model = _chain_model(4, actuators=2)
    bundle = _write_bundle(tmp_path, model)
    _rewrite_state(
        bundle,
        lambda raw: _patch_u64(
            raw, HDR_DIM[label], getattr(model, label) + 1
        ),
    )
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'does not match' in result.stderr or 'mismatch' in result.stderr


def test_declared_payload_length_lie_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    """payload_bytes is independently mandatory: patching only the declared
    field (file length still correct) must still fail."""
    bundle = _write_bundle(tmp_path, _chain_model(2))
    _rewrite_state(
        bundle, lambda raw: _patch_u64(raw, HDR_PAYLOAD_BYTES, 8)
    )
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'expected' in result.stderr or 'does not match' in result.stderr


def test_payload_checksum_corruption_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    """Flipping one payload byte breaks state_crc64 — corruption caught
    before decode, not by a downstream finiteness accident."""
    bundle = _write_bundle(tmp_path, _chain_model(2))

    def flip(raw: bytes) -> bytes:
        patched = bytearray(raw)
        patched[-1] ^= 0xFF
        return bytes(patched)

    _rewrite_state(bundle, flip)
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'checksum' in result.stderr


def test_declared_state_checksum_lie_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _write_bundle(tmp_path, _chain_model(2))
    _rewrite_state(
        bundle, lambda raw: _patch_u64(raw, HDR_STATE_CRC, 0)
    )
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'checksum' in result.stderr


def test_state_from_other_generation_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    """Mixed-generation mixing: same dims, different model bytes — the
    model_crc64 binds state.bin to its own model.mjb."""
    model = _chain_model(3)
    bundle = _write_bundle(tmp_path / 'main', model)
    other_model = _chain_model(3)
    other_model.body_mass[1] = 0.9  # same dims, different model bytes
    other = _write_bundle(tmp_path / 'other', other_model)
    (bundle / 'state.bin').write_bytes((other / 'state.bin').read_bytes())
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'checksum' in result.stderr


def test_state_from_other_model_dims_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    other = _write_bundle(tmp_path / 'other', _chain_model(5))
    bundle = _write_bundle(tmp_path / 'main', _chain_model(3))
    (bundle / 'state.bin').write_bytes((other / 'state.bin').read_bytes())
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1


def test_nonfinite_state_scalar_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _write_bundle(tmp_path, _chain_model(2), nonfinite=True)
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'finite' in result.stderr


# ---------------------------------------------------------------------------
# Workload domains
# ---------------------------------------------------------------------------

def test_bare_accepts_empty_model(
    bench_executable: Path, tmp_path: Path
) -> None:
    """nv=0/nu=0/nbody=1 — bare steps an empty model if the engine permits it."""
    model = _chain_model(0, actuators=0)
    assert model.nv == 0 and model.nu == 0 and model.nbody == 1
    bundle = _write_bundle(tmp_path, model)
    result = _run_bench(bench_executable, 'bare', '3', '2', '--artifacts', str(bundle))
    assert result.returncode == 0, result.stderr
    assert 'mode=bare' in result.stdout
    assert 'nv=0' in result.stdout


@pytest.mark.parametrize('mode', ['glue', 'heavy'])
def test_non_bare_modes_reject_empty_model(
    bench_executable: Path, tmp_path: Path, mode: str
) -> None:
    bundle = _write_bundle(tmp_path, _chain_model(0, actuators=0))
    result = _run_bench(bench_executable, mode, '2', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'requires' in result.stderr


def test_glue_rejects_no_actuators(
    bench_executable: Path, tmp_path: Path
) -> None:
    model = _chain_model(4, actuators=0)
    assert model.nu == 0
    bundle = _write_bundle(tmp_path, model)
    result = _run_bench(bench_executable, 'glue', '2', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'nu' in result.stderr


@pytest.mark.parametrize('dofs', [6, 9])
def test_heavy_rejects_too_few_attachment_bodies(
    bench_executable: Path, tmp_path: Path, dofs: int
) -> None:
    """nbody = dofs + 1 < 11 — the heavy attachment list cannot resolve."""
    bundle = _write_bundle(tmp_path, _chain_model(dofs))
    result = _run_bench(bench_executable, 'heavy', '2', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'nbody' in result.stderr


def test_heavy_rejects_narrow_model(
    bench_executable: Path, tmp_path: Path
) -> None:
    """nbody >= 11 but nv < 3: welded shells add bodies without dofs."""
    model = _chain_model(1, welded=11)
    assert model.nbody == 13 and model.nv == 1
    bundle = _write_bundle(tmp_path, model)
    result = _run_bench(bench_executable, 'heavy', '2', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'nv' in result.stderr


# ---------------------------------------------------------------------------
# Valid controls — dimension sweep, plus the fixed-buffer ASan cases
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('nv', [0, 1, 33, 65, 128])
def test_bare_dimension_sweep(
    bench_executable: Path, tmp_path: Path, nv: int
) -> None:
    bundle = _write_bundle(tmp_path / f'nv{nv}', _chain_model(nv))
    result = _run_bench(
        bench_executable, 'bare', '3', '2', '--artifacts', str(bundle)
    )
    assert result.returncode == 0, result.stderr
    assert f'nv={nv}' in result.stdout


@pytest.mark.parametrize('nv', [1, 33, 65, 128])
def test_glue_dimension_sweep(
    bench_executable: Path, tmp_path: Path, nv: int
) -> None:
    """nv=65 exceeds the prototype's fixed 64-element Jacobian cap; nv=33
    crosses the 3*nv shared-buffer reuse defect. Both must succeed."""
    bundle = _write_bundle(tmp_path / f'nv{nv}', _chain_model(nv))
    result = _run_bench(
        bench_executable, 'glue', '3', '2', '--artifacts', str(bundle)
    )
    assert result.returncode == 0, result.stderr
    assert f'nv={nv}' in result.stdout


def test_glue_with_nu_greater_than_nv(
    bench_executable: Path, tmp_path: Path
) -> None:
    """nu > nv: the actuator->velocity mapping must go through the
    engine-projected qfrc_actuator (length nv), never qvel[:nu]."""
    model = _chain_model(2, actuators=4)
    assert model.nu > model.nv
    bundle = _write_bundle(tmp_path, model)
    result = _run_bench(
        bench_executable, 'glue', '3', '2', '--artifacts', str(bundle)
    )
    assert result.returncode == 0, result.stderr
    assert 'mode=glue' in result.stdout
    assert 'nan' not in result.stdout


def test_heavy_nv33_paired_jacobians(
    bench_executable: Path, tmp_path: Path
) -> None:
    """ASan control: nv=33 makes the prototype's 64-double shared Jacobian
    buffer overflow; model-sized separate buffers must not."""
    bundle = _write_bundle(tmp_path / 'nv33', _chain_model(33))
    result = _run_bench(
        bench_executable, 'heavy', '3', '2', '--artifacts', str(bundle)
    )
    assert result.returncode == 0, result.stderr
    assert 'nv=33' in result.stdout
    assert 'nan' not in result.stdout


def test_heavy_boundary_model(
    bench_executable: Path, tmp_path: Path
) -> None:
    """nbody == 11 is the smallest supported heavy model."""
    model = _chain_model(10)
    assert model.nbody == 11
    bundle = _write_bundle(tmp_path, model)
    result = _run_bench(
        bench_executable, 'heavy', '3', '2', '--artifacts', str(bundle)
    )
    assert result.returncode == 0, result.stderr


def test_operation_mix_mode_runs(
    bench_executable: Path, tmp_path: Path
) -> None:
    """operation_mix keeps its R2 approximation — it runs, reports its own
    claim, and is NOT among the parity-checked workloads."""
    bundle = _write_bundle(tmp_path, _chain_model(12))
    emit_path = tmp_path / 'mix.json'
    result = _run_bench(
        bench_executable, 'operation_mix', '4', '2',
        '--artifacts', str(bundle), '--emit-json', str(emit_path),
    )
    assert result.returncode == 0, result.stderr
    assert 'mode=operation_mix' in result.stdout
    assert 'nan' not in result.stdout
    report = json.loads(emit_path.read_text())
    assert report['comparison'] == 'operation_mix'
    assert report['workload']['claim'] == 'operation_mix'


@pytest.mark.parametrize('mode,args', [
    ('bare', ('10', '5')),
    ('glue', ('5', '2')),
    ('heavy', ('3', '2')),
    ('operation_mix', ('4', '2')),
])
def test_tracked_artifacts_smoke(
    bench_executable: Path, mode: str, args: tuple[str, str]
) -> None:
    assert TRACKED_ARTIFACTS.is_dir()
    result = _run_bench(
        bench_executable, mode, *args, '--artifacts', str(TRACKED_ARTIFACTS)
    )
    assert result.returncode == 0, result.stderr
    assert f'mode={mode}' in result.stdout
    assert 'RTF=' in result.stdout
    assert 'nan' not in result.stdout


def test_rewind_boundaries(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _write_bundle(tmp_path, _chain_model(3))
    # rewind == steps: restore runs exactly once; rewind > steps is fine.
    result = _run_bench(
        bench_executable, 'glue', '4', '4', '--artifacts', str(bundle)
    )
    assert result.returncode == 0, result.stderr
    result = _run_bench(
        bench_executable, 'glue', '4', '99', '--artifacts', str(bundle)
    )
    assert result.returncode == 0, result.stderr


# ---------------------------------------------------------------------------
# Emitted JSON report + paired Python/C++ matched-workload conformance
# ---------------------------------------------------------------------------

def _emit_report(
    bench: Path, mode: str, bundle: Path, tmp_path: Path
) -> dict:
    emit_path = tmp_path / f'native_{mode}.json'
    result = _run_bench(
        bench, mode, '6', '2', '--artifacts', str(bundle),
        '--emit-json', str(emit_path),
    )
    assert result.returncode == 0, result.stderr
    return json.loads(emit_path.read_text())


def _emit_report_py(mode: str, bundle: Path, tmp_path: Path) -> dict:
    emit_path = tmp_path / f'python_{mode}.json'
    result = _run_bench_py(
        mode, '6', '2', '--artifacts', str(bundle),
        '--emit-json', str(emit_path),
    )
    assert result.returncode == 0, result.stderr
    return json.loads(emit_path.read_text())


def test_emit_json_report_shape(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _write_bundle(tmp_path / 'bundle', _chain_model(12))
    report = _emit_report(bench_executable, 'glue', bundle, tmp_path)
    assert report['format'] == 1
    assert report['mode'] == 'glue'
    assert report['comparison'] == 'matched'
    assert report['workload']['seed'] == '0x42f0e1eba9ea3693'
    counts = report['operation_counts']
    assert counts['engine_steps'] > 0 and counts['forwards'] > 0
    assert counts['jacobian_calls'] > 0 and counts['name_lookups'] > 0
    checksums = report['stage_checksums']
    for key in (
        'trajectory_time', 'forces', 'energy', 'jacobians', 'channels',
        'name_ids',
    ):
        assert key in checksums
    assert np.isfinite(report['checksum'])
    assert report['provenance']['model_crc64'].startswith('0x')


def test_emit_json_unwritable_path_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _write_bundle(tmp_path / 'bundle', _chain_model(2))
    missing_dir = tmp_path / 'no-such-dir' / 'report.json'
    result = _run_bench(
        bench_executable, 'glue', '2', '2', '--artifacts', str(bundle),
        '--emit-json', str(missing_dir),
    )
    assert result.returncode == 1


@pytest.mark.parametrize('mode', ['bare', 'glue', 'heavy'])
def test_matched_workload_python_native_conformance(
    bench_executable: Path, tmp_path: Path, mode: str
) -> None:
    """The matched workloads must produce the same operation counts, the same
    stage checksums, and solver outputs inside rtol=atol=1e-12 in both
    languages. Exact-integer stages (name_ids) must be byte-equal; float
    stages may differ only by platform libm noise bounded by the solver
    tolerance."""
    model = _chain_model(12)
    bundle = _generated_bundle(tmp_path / 'artifacts', model)
    native = _emit_report(bench_executable, mode, bundle, tmp_path)
    python = _emit_report_py(mode, bundle, tmp_path)

    assert native['comparison'] == 'matched'
    assert python['comparison'] == 'matched'
    assert native['workload'] == python['workload']
    assert native['operation_counts'] == python['operation_counts']
    assert (
        native['provenance']['model_crc64']
        == python['provenance']['model_crc64']
    )
    assert (
        native['provenance']['state_crc64']
        == python['provenance']['state_crc64']
    )

    native_ck = native['stage_checksums']
    python_ck = python['stage_checksums']
    assert set(native_ck) == set(python_ck)
    # Scalar stages that must be byte-equal.
    assert native_ck['name_ids'] == python_ck['name_ids']
    assert native_ck['trajectory_time'] == python_ck['trajectory_time']
    for key, native_value in native_ck.items():
        python_value = python_ck[key]
        assert np.isfinite(native_value)
        assert np.isfinite(python_value)
        assert np.isclose(
            native_value, python_value, rtol=1e-12, atol=1e-12
        ), f'stage_checksums[{key}]: {native_value!r} vs {python_value!r}'

    native_solve = np.asarray(native['solve_outputs'], dtype=np.float64)
    python_solve = np.asarray(python['solve_outputs'], dtype=np.float64)
    assert native_solve.shape == python_solve.shape
    assert np.isfinite(native_solve).all()
    assert np.allclose(
        native_solve, python_solve, rtol=1e-12, atol=1e-12
    )

    assert np.isfinite(native['checksum'])
    assert np.isfinite(python['checksum'])
    assert np.isclose(
        native['checksum'], python['checksum'], rtol=1e-12, atol=1e-12
    )
