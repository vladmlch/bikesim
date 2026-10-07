"""native_bench CLI/state contract regressions (native-safety plan task R2).

Fixtures are generated per test — a slide-joint chain sized to the case plus
a recorded state snapshot in the legacy 0xBEA0 layout — so no case depends on
the tracked artifacts alone. The executable comes from the selected build
(the same NATIVE_TEST_BUILD_PATH / NATIVE_TEST_BUILD_DIR resolution used by
the native extension tests), which keeps the ASan/UBSan build's native_bench
reachable from this suite.
"""

from __future__ import annotations

import os
from pathlib import Path
import struct
import subprocess
import sys

import mujoco
import numpy as np
import pytest

from native_loader import load_native, selected_build


REPO_ROOT = Path(__file__).resolve().parents[2]
TRACKED_ARTIFACTS = REPO_ROOT / 'tools' / 'proto_native_bench' / 'artifacts'

STATE_MAGIC = 0xBEA0
MAX_COUNT = 10_000_000


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


def _chain_model(dofs: int, actuators: int = 1, welded: int = 0) -> mujoco.MjModel:
    """Deterministic chain: nq = nv = dofs, nu = actuators, nbody = dofs + welded + 1."""
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
        f'<motor name="motor{i}" joint="joint{i}"/>'
        for i in range(min(actuators, dofs))
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


def _state_bytes(model: mujoco.MjModel, *, nonfinite: bool = False) -> bytes:
    """Legacy 0xBEA0 payload for `model`'s dimensions — no MjData needed, so
    models whose data allocation would fail can still produce fixtures."""
    nq, nv, na, nu, nbody = (
        model.nq, model.nv, model.na, model.nu, model.nbody
    )
    time = np.nan if (nonfinite and nq == 0) else 0.25
    qpos = np.linspace(0.01, 0.02, nq) if nq else np.zeros(0)
    if nonfinite and nq:
        qpos[0] = np.nan
    qvel = np.linspace(-0.01, 0.01, nv) if nv else np.zeros(0)
    act = np.zeros(na)
    ctrl = np.linspace(0.1, 0.2, nu) if nu else np.zeros(0)
    qfrc = np.linspace(0.001, 0.002, nv) if nv else np.zeros(0)
    xfrc = np.zeros(6 * nbody)
    header = struct.pack('<6i', STATE_MAGIC, nq, nv, na, nu, nbody)
    payload = struct.pack('<d', float(time))
    for array in (qpos, qvel, act, ctrl, qfrc, xfrc):
        payload += np.asarray(array, dtype=np.float64).tobytes()
    return header + payload


def _write_bundle(
    root: Path, model: mujoco.MjModel, *, state: bytes | None = None
) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    mujoco.mj_saveModel(model, str(root / 'model.mjb'))
    (root / 'state.bin').write_bytes(
        _state_bytes(model) if state is None else state
    )
    return root


def _patch_header(state: bytes, field_index: int, value: int) -> bytes:
    """Rewrite one i32 header field (1=nq, 2=nv, 3=na, 4=nu, 5=nbody)."""
    patched = bytearray(state)
    struct.pack_into('<i', patched, 4 * field_index, value)
    return bytes(patched)


# ---------------------------------------------------------------------------
# CLI contract
# ---------------------------------------------------------------------------

def test_help_prints_usage(bench_executable: Path) -> None:
    result = _run_bench(bench_executable, '--help')
    assert result.returncode == 0
    assert 'usage' in result.stdout.lower()


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
# Artifact / model / state validation
# ---------------------------------------------------------------------------

def test_missing_state_bin_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    mujoco.mj_saveModel(_chain_model(2), str(tmp_path / 'model.mjb'))
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(tmp_path))
    assert result.returncode == 1
    assert 'state.bin' in result.stderr


def test_missing_model_mjb_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    model = _chain_model(2)
    (tmp_path / 'state.bin').write_bytes(_state_bytes(model))
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(tmp_path))
    assert result.returncode == 1
    assert 'model.mjb' in result.stderr


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


def test_bad_state_magic_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    model = _chain_model(2)
    state = bytearray(_state_bytes(model))
    struct.pack_into('<i', state, 0, 0)
    bundle = _write_bundle(tmp_path, model, state=bytes(state))
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1


def test_truncated_state_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    model = _chain_model(2)
    bundle = _write_bundle(tmp_path, model, state=_state_bytes(model)[:-16])
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1


def test_truncated_state_header_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    model = _chain_model(2)
    bundle = _write_bundle(tmp_path, model, state=_state_bytes(model)[:20])
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1


def test_trailing_state_bytes_are_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    model = _chain_model(2)
    bundle = _write_bundle(
        tmp_path, model, state=_state_bytes(model) + b'\x00' * 8
    )
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1


def test_negative_header_count_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    model = _chain_model(2)
    bundle = _write_bundle(
        tmp_path, model, state=_patch_header(_state_bytes(model), 1, -1)
    )
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'negative' in result.stderr


def test_excessive_header_count_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    """A huge positive count mismatches the model and must be rejected before
    any buffer is sized — never fed to an allocation."""
    model = _chain_model(2)
    bundle = _write_bundle(
        tmp_path,
        model,
        state=_patch_header(_state_bytes(model), 1, 2_000_000_000),
    )
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'does not match' in result.stderr or 'mismatch' in result.stderr


@pytest.mark.parametrize(
    'field_index,label',
    [(1, 'nq'), (2, 'nv'), (3, 'na'), (4, 'nu'), (5, 'nbody')],
)
def test_state_model_dimension_mismatch_is_rejected(
    bench_executable: Path, tmp_path: Path, field_index: int, label: str
) -> None:
    model = _chain_model(4, actuators=2)
    mismatched = _patch_header(
        _state_bytes(model), field_index,
        getattr(model, label) + 1,
    )
    bundle = _write_bundle(tmp_path, model, state=mismatched)
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1
    assert 'does not match' in result.stderr or 'mismatch' in result.stderr


def test_state_from_other_model_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    bundle = _write_bundle(
        tmp_path, _chain_model(3), state=_state_bytes(_chain_model(5))
    )
    result = _run_bench(bench_executable, 'bare', '--artifacts', str(bundle))
    assert result.returncode == 1


def test_nonfinite_state_scalar_is_rejected(
    bench_executable: Path, tmp_path: Path
) -> None:
    model = _chain_model(2)
    bundle = _write_bundle(
        tmp_path, model, state=_state_bytes(model, nonfinite=True)
    )
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


@pytest.mark.parametrize('mode,args', [
    ('bare', ('10', '5')),
    ('glue', ('5', '2')),
    ('heavy', ('3', '2')),
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
        bench_executable, 'glue', '4', '100', '--artifacts', str(bundle)
    )
    assert result.returncode == 0, result.stderr
