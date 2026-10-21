"""The opt-in hook preserves assertion semantics and indexes raw oracle bytes."""
import json

import numpy as np
import pytest

import _golden
from _bits import assert_bitwise_equal


def test_capture_writes_expected_per_call(tmp_path, monkeypatch, request):
    monkeypatch.setenv("BIKE_GOLDEN_CAPTURE", str(tmp_path))
    assert_bitwise_equal(np.array([1.0, -0.0]), np.array([1.0, -0.0]), "pair")
    assert_bitwise_equal(np.int64(7), np.int64(7))
    meta_path, bin_path = _golden.node_files(tmp_path, request.node.nodeid)
    meta = json.loads(meta_path.read_text())
    assert [m["index"] for m in meta] == [0, 1]
    assert meta[0]["dtype"] == "<f8" and meta[0]["shape"] == [2]
    assert meta[0]["label"] == "pair" and meta[1]["offset"] == 16
    assert meta[1]["dtype"] == "<i8" and meta[1]["shape"] == []
    assert bin_path.read_bytes() == np.array([1.0, -0.0]).tobytes() + np.int64(7).tobytes()


def test_close_kind_records_tolerances(tmp_path, monkeypatch, request):
    monkeypatch.setenv("BIKE_GOLDEN_CAPTURE", str(tmp_path))
    _golden.record(np.array([0.5]), "speed", kind="close", rtol=1e-6, atol=1e-12)
    meta = json.loads(_golden.node_files(tmp_path, request.node.nodeid)[0].read_text())
    assert meta[0]["kind"] == "close" and meta[0]["rtol"] == 1e-6 and meta[0]["atol"] == 1e-12


def test_index_zero_truncates_an_earlier_capture(tmp_path, monkeypatch, request):
    monkeypatch.setenv("BIKE_GOLDEN_CAPTURE", str(tmp_path))
    meta_path, bin_path = _golden.node_files(tmp_path, request.node.nodeid)
    meta_path.parent.mkdir(parents=True)
    meta_path.write_text('[{"index": 0}, {"index": 1}, {"index": 2}]\n')
    bin_path.write_bytes(b"\0" * 64)
    _golden.record(np.array([3.0]))
    assert len(json.loads(meta_path.read_text())) == 1
    assert bin_path.read_bytes() == np.array([3.0]).tobytes()


def test_capture_off_writes_nothing(tmp_path, monkeypatch):
    monkeypatch.delenv("BIKE_GOLDEN_CAPTURE", raising=False)
    assert_bitwise_equal(np.array([2.0]), np.array([2.0]))
    assert not any(tmp_path.iterdir())


def test_node_path_is_filesystem_safe_and_bounded():
    node = "tests/reference/test_x.py::test_y[" + "a/b (c) " * 40 + "]"
    file_key, test_key = _golden.node_path(node).split("/")
    assert file_key == "tests_reference_test_x.py"
    assert len(test_key) == 149 and all(c.isalnum() or c in "._-" for c in test_key)
    assert test_key.endswith("-edc0213d")
    assert _golden.node_path("tests/a.py::t[😀]") == "tests_a.py/t___"


def test_dotted_parameter_keeps_its_full_key(tmp_path):
    meta, payload = _golden.node_files(tmp_path, "tests/a.py::t[1.5]")
    assert meta.name == "t_1.5_.json" and payload.name == "t_1.5_.bin"


def test_phase_suffix_is_stripped_only_at_the_end(monkeypatch):
    monkeypatch.setenv("PYTEST_CURRENT_TEST", "tests/a.py::t[x (call) y] (call)")
    assert _golden._current_node() == "tests/a.py::t[x (call) y]"


def test_failed_assertion_still_fails_and_records_expected(tmp_path, monkeypatch, request):
    monkeypatch.setenv("BIKE_GOLDEN_CAPTURE", str(tmp_path))
    with pytest.raises(AssertionError, match="bitwise mismatch"):
        assert_bitwise_equal(np.array([1.]), np.array([-0.]), "deliberate mismatch")
    _, binary = _golden.node_files(tmp_path, request.node.nodeid)
    assert binary.read_bytes() == np.array([-0.]).tobytes()


def test_noncontiguous_input_records_c_order_without_changing_shape(tmp_path, monkeypatch, request):
    monkeypatch.setenv("BIKE_GOLDEN_CAPTURE", str(tmp_path))
    array = np.arange(12, dtype=np.float64).reshape(3, 4).T
    _golden.record(array)
    sidecar, payload = _golden.node_files(tmp_path, request.node.nodeid)
    assert json.loads(sidecar.read_text())[0]["shape"] == [4, 3]
    assert payload.read_bytes() == array.tobytes(order="C")


@pytest.mark.parametrize("kwargs", [
    {"kind": "unknown"}, {"rtol": 0.},
    {"kind": "close", "rtol": 0.},
    {"kind": "close", "rtol": -1., "atol": 0.},
    {"kind": "close", "rtol": float("nan"), "atol": 0.},
])
def test_bad_kind_or_tolerance_leaves_no_payload(tmp_path, monkeypatch, kwargs):
    monkeypatch.setenv("BIKE_GOLDEN_CAPTURE", str(tmp_path))
    with pytest.raises(ValueError):
        _golden.record(np.array([1.]), **kwargs)
    assert not any(tmp_path.iterdir())


def test_object_payload_is_rejected(tmp_path, monkeypatch):
    monkeypatch.setenv("BIKE_GOLDEN_CAPTURE", str(tmp_path))
    with pytest.raises(ValueError, match="pickle-free"):
        _golden.record(np.array([object()], dtype=object))
    assert not any(tmp_path.iterdir())


def test_sanitized_name_collision_is_rejected(tmp_path, monkeypatch):
    monkeypatch.setenv("BIKE_GOLDEN_CAPTURE", str(tmp_path))
    monkeypatch.setenv("PYTEST_CURRENT_TEST", "tests/a.py::t[a/b] (call)")
    _golden.record(np.array([1.]))
    monkeypatch.setenv("PYTEST_CURRENT_TEST", "tests/a.py::t[a b] (call)")
    with pytest.raises(ValueError, match="collision"):
        _golden.record(np.array([2.]))
