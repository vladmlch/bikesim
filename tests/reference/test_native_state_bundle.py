"""BIKEST02 state-bundle format tests (native-safety plan task R3).

Python-side coverage of tools/proto_native_bench/state_format.py: the writer
(write_bundle + atomic current.json publication), the reader (read_state),
and resolve_bundle. The C++ consumer path is exercised through the
native_bench CLI in test_native_benchmark.py.

Layout under test (80-byte little-endian header '<8s2I8Q'):
    offset  0  magic8         "BIKEST02"
    offset  8  version        u32 = 2
    offset 12  flags          u32 = 0
    offset 16  nq, nv, na, nu, nbody   u64 each
    offset 56  payload_bytes  u64
    offset 64  model_crc64    u64
    offset 72  state_crc64    u64
Payload: binary64 in order time(1), qpos(nq), qvel(nv), act(na), ctrl(nu),
qfrc_applied(nv), xfrc_applied(6*nbody flat), qacc_warmstart(nv).
"""

from __future__ import annotations

import json
from pathlib import Path
import struct
import tempfile

import mujoco
import numpy as np
import pytest

from native_loader import load_native
from tools.proto_native_bench import state_format as sf


@pytest.fixture(scope='module', autouse=True)
def _selected_build_ready() -> None:
    """Anchor to the selected native build like every test_native_* module —
    the format under test is what that build's native_bench consumes."""
    load_native()


MAGIC = b'BIKEST02'
HDR_VERSION = 8
HDR_FLAGS = 12
HDR_DIM = {'nq': 16, 'nv': 24, 'na': 32, 'nu': 40, 'nbody': 48}
HDR_PAYLOAD_BYTES = 56
HDR_MODEL_CRC = 64
HDR_STATE_CRC = 72


def _chain_model(dofs: int = 3, actuators: int = 2) -> mujoco.MjModel:
    bodies = ''.join(
        f'<body name="link{i}" pos="{i} 0 0">'
        f'<joint name="joint{i}" type="slide" axis="1 0 0"/>'
        f'<geom type="sphere" size="0.1" mass="0.5"/></body>'
        for i in range(dofs)
    )
    motors = ''.join(
        f'<motor name="motor{i}" joint="joint{i % dofs}"/>'
        for i in range(actuators if dofs else 0)
    )
    actuator_xml = f'<actuator>{motors}</actuator>' if motors else ''
    return mujoco.MjModel.from_xml_string(
        f'<mujoco><worldbody>{bodies}</worldbody>{actuator_xml}</mujoco>'
    )


def _data(model: mujoco.MjModel) -> mujoco.MjData:
    """Deterministic, finite snapshot content."""
    data = mujoco.MjData(model)
    if model.nq:
        data.qpos[:] = np.linspace(0.05, 0.06, model.nq)
    if model.nv:
        data.qvel[:] = np.linspace(-0.02, 0.02, model.nv)
        data.qfrc_applied[:] = np.linspace(0.001, 0.002, model.nv)
        data.qacc_warmstart[:] = np.linspace(0.0, 0.003, model.nv)
    if model.nu:
        data.ctrl[:] = np.linspace(0.1, 0.3, model.nu)
    data.xfrc_applied[:] = np.linspace(
        -0.01, 0.01, 6 * model.nbody
    ).reshape(model.nbody, 6)
    data.time = 0.125
    return data


def _bundle(root, model=None) -> tuple:
    model = _chain_model() if model is None else model
    data = _data(model)
    bundle = sf.write_bundle(
        root, model, data, name='gen',
        source_track='test-track', source_config='test-config',
        warmup_steps=3,
    )
    return bundle, model, data


def _model_bytes(tmp_path, model) -> bytes:
    path = tmp_path / 'm.mjb'
    mujoco.mj_saveModel(model, str(path))
    return path.read_bytes()


def _arrays(model) -> dict[str, np.ndarray]:
    return {
        'time': np.array([0.125]),
        'qpos': np.linspace(0.01, 0.02, model.nq),
        'qvel': np.linspace(-0.01, 0.01, model.nv),
        'act': np.zeros(model.na),
        'ctrl': np.linspace(0.1, 0.2, model.nu),
        'qfrc_applied': np.linspace(0.001, 0.002, model.nv),
        'xfrc_applied': np.zeros((model.nbody, 6)),
        'qacc_warmstart': np.linspace(0.0, 0.004, model.nv),
    }


def _patch(raw: bytes, offset: int, value: int, fmt: str = '<Q') -> bytes:
    patched = bytearray(raw)
    struct.pack_into(fmt, patched, offset, value)
    return bytes(patched)


def _read(bundle) -> dict[str, np.ndarray]:
    return sf.read_state(bundle / 'state.bin', bundle / 'model.mjb')


# ---------------------------------------------------------------------------
# CRC64 check vector + header shape
# ---------------------------------------------------------------------------

def test_crc64_ecma_check_vector() -> None:
    """CRC64-ECMA('123456789') == 0x6C40DF5F0B497347 — the published check
    vector for poly 0x42F0E1EBA9EA3693, init 0, no reflection, no xorout.
    The C++ side asserts the same vector via `native_bench --selftest`."""
    assert sf.crc64_ecma(b'123456789') == 0x6C40DF5F0B497347
    assert sf.crc64_ecma(b'') == 0


def test_header_is_exactly_80_bytes() -> None:
    assert sf.HEADER_BYTES == 80
    assert sf.HEADER.size == 80


# ---------------------------------------------------------------------------
# Roundtrip + NPZ/BIN byte equality
# ---------------------------------------------------------------------------

def test_bundle_npz_matches_bin() -> None:
    """state.npz and state.bin must be byte-equal for every array, warmstart
    included — the two representations can never disagree."""
    with tempfile.TemporaryDirectory() as tmp:
        bundle, _model, _data_ = _bundle(tmp)
        state = _read(bundle)
        with np.load(bundle / 'state.npz') as archive:
            for key in state:
                np.testing.assert_array_equal(state[key], archive[key])
                assert state[key].tobytes() == archive[key].tobytes()


def test_read_state_roundtrip_values() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        bundle, model, data = _bundle(tmp)
        state = _read(bundle)
        assert state['time'] == pytest.approx(0.125)
        np.testing.assert_array_equal(state['qpos'], data.qpos)
        np.testing.assert_array_equal(state['qvel'], data.qvel)
        np.testing.assert_array_equal(state['act'], data.act)
        np.testing.assert_array_equal(state['ctrl'], data.ctrl)
        np.testing.assert_array_equal(
            state['qfrc_applied'], data.qfrc_applied
        )
        np.testing.assert_array_equal(
            state['xfrc_applied'], data.xfrc_applied
        )
        np.testing.assert_array_equal(
            state['qacc_warmstart'], data.qacc_warmstart
        )
        assert state['xfrc_applied'].shape == (model.nbody, 6)
        assert state['qacc_warmstart'].shape == (model.nv,)


def test_write_bundle_publishes_pointer() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        bundle, _m, _d = _bundle(root)
        assert bundle.name == 'gen'
        pointer = json.loads((root / 'current.json').read_text())
        assert pointer == {'format': 1, 'bundle': 'gen'}
        assert sf.resolve_bundle(root) == bundle
        # Second generation: a new dir is created and the pointer moves.
        bundle2 = sf.write_bundle(root, _m, _d)
        assert bundle2 != bundle
        pointer2 = json.loads((root / 'current.json').read_text())
        assert pointer2['bundle'] == bundle2.name
        assert sf.resolve_bundle(root) == bundle2


def test_bundle_name_collision_is_rejected() -> None:
    """Bundles are immutable — a name reuse attempt fails rather than
    overwriting a published generation."""
    with tempfile.TemporaryDirectory() as tmp:
        _bundle(tmp)
        with pytest.raises(FileExistsError, match='immutable'):
            _bundle(tmp)


def test_manifest_records_provenance() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        bundle, model, _d = _bundle(tmp)
        manifest = json.loads((bundle / 'manifest.json').read_text())
        assert manifest['format'] == 1
        dims = manifest['model']
        for key in ('nq', 'nv', 'na', 'nu', 'nbody'):
            assert dims[key] == getattr(model, key)
        assert dims['timestep'] == pytest.approx(float(model.opt.timestep))
        assert manifest['source'] == {
            'track': 'test-track',
            'physics_config': 'test-config',
            'warmup_steps': 3,
        }
        checksums = manifest['checksums']
        assert checksums['model_mjb_crc64'] == (
            f"0x{sf.crc64_ecma((bundle / 'model.mjb').read_bytes()):016x}"
        )
        assert checksums['state_payload_crc64'] == (
            f"0x{sf.crc64_ecma((bundle / 'state.bin').read_bytes()[80:]):016x}"
        )
        assert 'workload' in manifest
        assert manifest['workload']['regularization'] == pytest.approx(1e-6)


# ---------------------------------------------------------------------------
# Header field validation
# ---------------------------------------------------------------------------

def _corrupt(tmp_path, transform, *, model=None):
    """A valid bundle whose state.bin is then mutated; returns the paths
    read_state needs."""
    model = _chain_model() if model is None else model
    bundle, _m, _d = _bundle(tmp_path / 'gen-root', model)
    state_path = bundle / 'state.bin'
    state_path.write_bytes(transform(state_path.read_bytes()))
    return state_path, bundle / 'model.mjb'


def test_legacy_magic_is_rejected() -> None:
    """A retired 0xBEA0 file gets a regeneration instruction, not a silent
    mis-parse."""
    with tempfile.TemporaryDirectory() as tmp:
        model = _chain_model()
        mjb = Path(tmp) / 'model.mjb'
        mujoco.mj_saveModel(model, str(mjb))
        legacy_path = Path(tmp) / 'state.bin'
        legacy_path.write_bytes(
            struct.pack('<6i', 0xBEA0, model.nq, model.nv, model.na,
                        model.nu, model.nbody)
            + b'\x00' * (8 * (1 + model.nq + 2 * model.nv + model.na
                              + model.nu + 6 * model.nbody))
        )
        with pytest.raises(sf.StateFormatError, match='retired|regenerate'):
            sf.read_state(legacy_path, mjb)


def test_bad_magic_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        state_path, model_path = _corrupt(
            Path(tmp),
            lambda raw: b'BIKEST03' + raw[8:],
        )
        with pytest.raises(sf.StateFormatError, match='format'):
            sf.read_state(state_path, model_path)


def test_unknown_version_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        state_path, model_path = _corrupt(
            Path(tmp),
            lambda raw: _patch(raw, HDR_VERSION, 7, '<I'),
        )
        with pytest.raises(sf.StateFormatError, match='version'):
            sf.read_state(state_path, model_path)


def test_unknown_flags_are_rejected() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        state_path, model_path = _corrupt(
            Path(tmp),
            lambda raw: _patch(raw, HDR_FLAGS, 0x80000000, '<I'),
        )
        with pytest.raises(sf.StateFormatError, match='flags'):
            sf.read_state(state_path, model_path)


@pytest.mark.parametrize('field', ['nq', 'nv', 'na', 'nu', 'nbody'])
def test_dimension_mismatch_is_rejected(field: str) -> None:
    with tempfile.TemporaryDirectory() as tmp:
        model = _chain_model()
        state_path, model_path = _corrupt(
            Path(tmp),
            lambda raw: _patch(
                raw, HDR_DIM[field], getattr(model, field) + 1
            ),
        )
        with pytest.raises(sf.StateFormatError, match='does not match'):
            sf.read_state(state_path, model_path)


@pytest.mark.parametrize('field', ['nq', 'nv', 'na', 'nu', 'nbody'])
def test_out_of_domain_dimension_is_rejected(field: str) -> None:
    """u64 fields beyond the signed mjtSize domain are rejected before the
    model comparison and before any allocation."""
    with tempfile.TemporaryDirectory() as tmp:
        state_path, model_path = _corrupt(
            Path(tmp),
            lambda raw: _patch(raw, HDR_DIM[field], 1 << 63),
        )
        with pytest.raises(sf.StateFormatError, match='domain|does not match'):
            sf.read_state(state_path, model_path)


def test_declared_payload_length_mismatch_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        state_path, model_path = _corrupt(
            Path(tmp),
            lambda raw: _patch(raw, HDR_PAYLOAD_BYTES, 8),
        )
        with pytest.raises(sf.StateFormatError, match='payload'):
            sf.read_state(state_path, model_path)


def test_truncated_header_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        state_path, model_path = _corrupt(
            Path(tmp), lambda raw: raw[:20]
        )
        with pytest.raises(sf.StateFormatError, match='truncated'):
            sf.read_state(state_path, model_path)


def test_truncated_payload_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        state_path, model_path = _corrupt(
            Path(tmp), lambda raw: raw[:-16]
        )
        with pytest.raises(sf.StateFormatError, match='payload|expected'):
            sf.read_state(state_path, model_path)


def test_trailing_bytes_are_rejected() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        state_path, model_path = _corrupt(
            Path(tmp), lambda raw: raw + b'\x00' * 8
        )
        with pytest.raises(sf.StateFormatError, match='expected'):
            sf.read_state(state_path, model_path)


def test_payload_checksum_corruption_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        def flip(raw: bytes) -> bytes:
            patched = bytearray(raw)
            patched[-1] ^= 0xFF
            return bytes(patched)
        state_path, model_path = _corrupt(
            Path(tmp), flip
        )
        with pytest.raises(sf.StateFormatError, match='checksum'):
            sf.read_state(state_path, model_path)


def test_model_checksum_corruption_is_rejected() -> None:
    """Declared model_crc64 no longer matching model.mjb = mixed-generation
    or corrupted member."""
    with tempfile.TemporaryDirectory() as tmp:
        state_path, model_path = _corrupt(
            Path(tmp),
            lambda raw: _patch(raw, HDR_MODEL_CRC, 0xDEADBEEF),
        )
        with pytest.raises(sf.StateFormatError, match='checksum'):
            sf.read_state(state_path, model_path)


def test_mixed_generation_is_rejected() -> None:
    """state.bin from generation A read against model.mjb from generation B:
    same dims but different bytes — the model crc must catch the mix."""
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        model_a = _chain_model()
        bundle_a, _ma, _da = _bundle(root / 'a', model_a)
        model_b = _chain_model()
        model_b.body_mass[1] = 0.9  # same dims, different model bytes
        bundle_b, _mb, _db = _bundle(root / 'b', model_b)
        with pytest.raises(sf.StateFormatError, match='checksum'):
            sf.read_state(
                bundle_a / 'state.bin', bundle_b / 'model.mjb'
            )


def test_nonfinite_stored_scalar_is_rejected() -> None:
    """A NaN smuggled inside a self-consistent file (crc valid for the NaN
    bytes) is caught by the per-section finiteness check during decode."""
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        model = _chain_model()
        mjb = root / 'model.mjb'
        mujoco.mj_saveModel(model, str(mjb))
        arrays = _arrays(model)
        arrays['qpos'] = arrays['qpos'].copy()
        arrays['qpos'][0] = np.nan
        state_path = root / 'state.bin'
        state_path.write_bytes(
            sf.serialize_state(arrays, mjb.read_bytes())
        )
        with pytest.raises(sf.StateFormatError, match='non-finite|finite'):
            sf.read_state(state_path, mjb)


# ---------------------------------------------------------------------------
# resolve_bundle — pointer and concrete-directory resolution
# ---------------------------------------------------------------------------

def test_resolve_concrete_bundle_dir() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        bundle, _m, _d = _bundle(tmp)
        assert sf.resolve_bundle(bundle) == bundle


def test_resolve_root_uses_pointer() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        bundle, _m, _d = _bundle(tmp)
        assert sf.resolve_bundle(tmp) == bundle


def test_resolve_malformed_pointer_is_rejected() -> None:
    """Interrupted pointer publication: current.json exists but is not a
    valid pointer document — fail loudly rather than guess."""
    with tempfile.TemporaryDirectory() as tmp:
        _bundle(tmp)
        root = Path(tmp)
        (root / 'current.json').write_text('{not json')
        with pytest.raises(sf.StateFormatError, match='current.json'):
            sf.resolve_bundle(root)


def test_resolve_pointer_to_missing_bundle_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        _bundle(tmp)
        root = Path(tmp)
        (root / 'current.json').write_text(
            json.dumps({'format': 1, 'bundle': 'gone'})
        )
        with pytest.raises(sf.StateFormatError, match='missing bundle'):
            sf.resolve_bundle(root)


@pytest.mark.parametrize('hostile', ['../escape', '/abs', 'a/b', '..', '.'])
def test_resolve_pointer_name_cannot_escape(
    hostile: str,
) -> None:
    with tempfile.TemporaryDirectory() as tmp:
        _bundle(tmp)
        root = Path(tmp)
        (root / 'current.json').write_text(
            json.dumps({'format': 1, 'bundle': hostile})
        )
        with pytest.raises(sf.StateFormatError, match='bundle name'):
            sf.resolve_bundle(root)


@pytest.mark.parametrize('missing', sf.BUNDLE_MEMBERS)
def test_resolve_incomplete_bundle_is_rejected(missing: str) -> None:
    with tempfile.TemporaryDirectory() as tmp:
        bundle, _m, _d = _bundle(tmp)
        (bundle / missing).unlink()
        with pytest.raises(sf.StateFormatError, match=missing):
            sf.resolve_bundle(bundle)


def test_resolve_missing_dir_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp) / 'nonexistent'
        with pytest.raises(sf.StateFormatError, match='model.mjb'):
            sf.resolve_bundle(root)


# ---------------------------------------------------------------------------
# Writer-side snapshot validation
# ---------------------------------------------------------------------------

def test_write_bundle_rejects_nonfinite_snapshot() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        model = _chain_model()
        data = _data(model)
        data.qpos[0] = np.nan
        with pytest.raises(sf.StateFormatError, match='non-finite'):
            sf.write_bundle(tmp, model, data)
        # The failed write must not publish a pointer or leave a bundle.
        root = Path(tmp)
        assert not (root / 'current.json').exists()
        assert not any(p.is_dir() for p in root.iterdir())


def test_serialize_rejects_mismatched_qfrc_extent() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        model = _chain_model()
        arrays = _arrays(model)
        arrays['qfrc_applied'] = np.zeros(model.nv + 1)
        with pytest.raises(sf.StateFormatError, match='qfrc_applied'):
            sf.serialize_state(arrays, _model_bytes(
                Path(tmp), model))


def test_serialize_rejects_bad_xfrc_extent() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        model = _chain_model()
        arrays = _arrays(model)
        arrays['xfrc_applied'] = np.zeros(7)  # not a multiple of 6
        with pytest.raises(sf.StateFormatError, match='xfrc_applied'):
            sf.serialize_state(arrays, _model_bytes(
                Path(tmp), model))
