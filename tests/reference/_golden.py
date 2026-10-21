"""Freeze oracle comparisons for ADR-0003 (completion plan 3.01).

Capture is opt-in. Each pytest node owns one JSON index and one C-order
binary payload. A new capture process truncates the pair at the first call;
do not commit a capture unless the entire Python file passed on that lane.
No oracle execution or fixture synthesis happens when this module imports.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import platform
import re
from pathlib import Path

import numpy as np

_counters: dict[tuple[str, str], int] = {}
_owners: dict[str, str] = {}
_UNSAFE = re.compile(r"[^A-Za-z0-9._-]")
_PHASE = re.compile(r" \((?:setup|call|teardown)\)$")


def platform_key() -> str:
    return f"{platform.system().lower()}-{platform.machine()}"


def _key(part: str) -> str:
    safe = _UNSAFE.sub("_", part)
    if len(safe) > 150:
        safe = safe[:140] + "-" + hashlib.sha256(part.encode("utf-8")).hexdigest()[:8]
    if not safe or safe in (".", ".."):
        raise ValueError("golden node needs nonempty safe file and test components")
    return safe


def node_path(node: str) -> str:
    file_part, separator, test_part = node.partition("::")
    if not separator:
        raise ValueError("golden node must include a pytest file and test separated by ::")
    return f"{_key(file_part)}/{_key(test_part)}"


def node_files(root, node: str) -> tuple[Path, Path]:
    base = Path(root) / platform_key() / node_path(node)
    # with_suffix would erase a dotted parameter such as "t_1.5_".
    return base.parent / (base.name + ".json"), base.parent / (base.name + ".bin")


def _current_node() -> str:
    return _PHASE.sub("", os.environ.get("PYTEST_CURRENT_TEST", ""))


def record(value, label: str = "", *, kind: str = "bitwise", rtol=None, atol=None) -> None:
    root = os.environ.get("BIKE_GOLDEN_CAPTURE")
    if not root:
        return
    if kind not in ("bitwise", "close"):
        raise ValueError(f"unknown golden comparison kind {kind!r}")
    if kind == "close":
        if rtol is None or atol is None:
            raise ValueError("golden close comparison requires both rtol and atol")
        rtol, atol = float(rtol), float(atol)
        if not math.isfinite(rtol) or not math.isfinite(atol) or min(rtol, atol) < 0:
            raise ValueError("golden tolerances must be finite and nonnegative")
    elif rtol is not None or atol is not None:
        raise ValueError("golden bitwise comparison cannot carry tolerances")
    node = _current_node()
    if not node:
        raise RuntimeError("golden capture outside a pytest node")
    array = np.asarray(value)
    if array.dtype.hasobject or array.dtype.fields is not None or array.dtype.subdtype is not None:
        raise ValueError("golden capture requires a primitive pickle-free dtype")
    if kind == "close" and array.dtype.str != "<f8":
        raise ValueError("golden close comparisons require little-endian float64")
    # tobytes(order='C') preserves a scalar's shape []; ascontiguousarray
    # would promote it to [1], silently changing the original assertion.
    payload = array.tobytes(order="C")
    meta_path, bin_path = node_files(root, node)
    key = (str(Path(root).resolve()), node)
    path_key = str(meta_path.resolve())
    owner = _owners.get(path_key)
    if owner is not None and owner != node:
        raise ValueError(f"golden node path collision: {owner!r} and {node!r}")
    index = _counters.get(key, 0)
    meta_path.parent.mkdir(parents=True, exist_ok=True)
    meta = [] if index == 0 else json.loads(meta_path.read_text(encoding="utf-8"))
    if len(meta) != index or any(entry["node"] != node for entry in meta):
        raise ValueError("golden capture index was modified during this process")
    offset = 0 if index == 0 else bin_path.stat().st_size
    if index and offset != sum(entry["nbytes"] for entry in meta):
        raise ValueError("golden capture payload was modified during this process")
    meta.append({"index": index, "node": node, "label": str(label), "kind": kind,
                 "dtype": array.dtype.str, "shape": list(array.shape),
                 "offset": offset, "nbytes": len(payload), "rtol": rtol, "atol": atol})
    encoded = json.dumps(meta, indent=1, sort_keys=True, allow_nan=False) + "\n"
    with bin_path.open("wb" if index == 0 else "ab") as output:
        output.write(payload)
    temporary = meta_path.with_name(meta_path.name + ".tmp")
    temporary.write_text(encoded, encoding="utf-8")
    temporary.replace(meta_path)
    # Publish the counter only once both files were successfully written.
    _owners[path_key] = node
    _counters[key] = index + 1
