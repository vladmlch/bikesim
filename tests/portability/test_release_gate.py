"""Contract tests for the release aggregation in
tools.portability_qualification (T30, Python-only).

These tests never import the native extension and never invoke
``tools/run_tests.sh``: they exercise ``evaluate_release`` on in-memory
release records plus tiny evidence files under ``tmp_path``, and the
required-gate set generated from the checked-in toolchain matrix. The
release record schema is documented in the tool's module docstring.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

from tools import portability_qualification
from tools.portability_qualification import (
    _missing_gates,
    evaluate_release,
    release_requirements,
    required_release_rows,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
MATRIX_DOC = REPO_ROOT / "docs" / "development" / "toolchain-matrix.json"

_COMMIT = "a" * 40
_STALE_COMMIT = "b" * 40


# ---------------------------------------------------------------------------
# Builders — a well-formed release record / requirements unless overridden.
# ---------------------------------------------------------------------------


def _record(**overrides):
    """A fully bound release record for the Linux native gate."""
    record = {
        "gate": "native",
        "platform": "Linux",
        "machine": "x86_64",
        "profile": "native",
        "toolchain": "clang++",
        "status": "pass",
        "source_commit": _COMMIT,
        "command": ["bash", "tools/run_tests.sh", "native"],
        "returncode": 0,
        "counts": {"collected": 40, "executed": 40},
        "artifact": {
            "identity": "native/build/linux-clang-release/"
            "bike_native.cpython-313.so",
            "sha256": "d" * 64,
        },
        "evidence": "linux-native.json",
        "evidence_sha256": "c" * 64,
    }
    record.update(overrides)
    return record


def _requirements(*required, source_commit=_COMMIT, optional=None):
    requirements = {"source_commit": source_commit, "required": list(required)}
    if optional is not None:
        requirements["optional"] = optional
    return requirements


def _two_gate_requirements():
    """One Linux + one Darwin row, both pinned to the record dimensions."""
    return _requirements(
        {
            "gate": "native",
            "platform": "Linux",
            "machine": "x86_64",
            "profile": "native",
            "toolchain": "clang++",
            "mode": None,
        },
        {
            "gate": "native",
            "platform": "Darwin",
            "machine": "arm64",
            "profile": "native",
            "toolchain": "AppleClang clang++",
            "mode": None,
        },
    )


def _two_gate_records():
    return [
        _record(),
        _record(
            platform="Darwin",
            machine="arm64",
            toolchain="AppleClang clang++",
            evidence="darwin-native.json",
        ),
    ]


def _failed_reasons(result):
    return [
        reason for entry in result["failed"] for reason in entry["reasons"]
    ]


# ---------------------------------------------------------------------------
# Plan step-1 example — verbatim.
# ---------------------------------------------------------------------------


def test_linux_only_results_cannot_release():
    records = [{"gate": "native", "platform": "Linux", "status": "pass",
                "source_commit": "a" * 40, "evidence": "linux-native.json"}]
    requirements = {"source_commit": "a" * 40,
                    "required": [{"gate": "native", "platform": "Linux"},
                                 {"gate": "native", "platform": "Darwin"}]}
    result = evaluate_release(records, requirements)
    assert result["status"] == "blocked"
    assert any(item["platform"] == "Darwin" for item in result["missing"])


# ---------------------------------------------------------------------------
# Positive control.
# ---------------------------------------------------------------------------


def test_all_rows_pass_releases_with_evidence_links():
    result = evaluate_release(_two_gate_records(), _two_gate_requirements())
    assert result["status"] == "pass"
    assert result["missing"] == [] and result["failed"] == []
    assert len(result["satisfied"]) == 2
    darwin = next(
        entry
        for entry in result["satisfied"]
        if entry["required"]["platform"] == "Darwin"
    )
    assert darwin["evidence"]["path"] == "darwin-native.json"
    assert darwin["evidence"]["sha256"] == "c" * 64
    assert darwin["green"] is True
    assert "RELEASE PASS" in result["statement"]


def test_platform_aliases_normalize_without_collapsing_architectures():
    """``macOS`` folds to Darwin and satisfies its row; an aarch64 record
    is a different architecture row than x86_64, not a folded 'Linux'."""
    requirements = _requirements(
        {"gate": "native", "platform": "Darwin", "machine": "arm64",
         "profile": "native", "toolchain": "AppleClang clang++",
         "mode": None},
        {"gate": "native", "platform": "Linux", "machine": "x86_64",
         "profile": "native", "toolchain": "clang++", "mode": None},
    )
    records = [
        _record(platform="macOS", machine="arm64",
                toolchain="AppleClang clang++"),
        _record(machine="aarch64"),  # same OS, different arch — no match
    ]
    result = evaluate_release(records, requirements)
    assert result["status"] == "blocked"
    assert any(
        item["machine"] == "x86_64" and item["platform"] == "Linux"
        for item in result["missing"]
    )
    assert not any(
        item["platform"] == "Darwin" for item in result["missing"]
    )
    # the aarch64 record is reported as an extra, not lost
    assert any(
        entry["record"]["machine"] == "aarch64" for entry in result["extra"]
    )


# ---------------------------------------------------------------------------
# Source-commit binding.
# ---------------------------------------------------------------------------


def test_stale_source_commit_blocks_release():
    records = _two_gate_records()
    records[0]["source_commit"] = _STALE_COMMIT
    result = evaluate_release(records, _two_gate_requirements())
    assert result["status"] == "blocked"
    assert any("stale" in r or "foreign" in r for r in _failed_reasons(result))


def test_fingerprint_equivalence_is_explicit_not_silent():
    """A documentation-only amended commit is accepted only when the
    requirements carry its source/build fingerprint pair and the record
    reproduces it — a bare newer SHA is stale evidence."""
    equivalence = {"source_fingerprint": "srcfp", "build_fingerprint": "bfp"}
    requirements = _requirements(
        {
            "gate": "native", "platform": "Linux", "machine": "x86_64",
            "profile": "native", "toolchain": "clang++", "mode": None,
        },
        source_commit=_COMMIT,
    )
    requirements["equivalent_commits"] = {_STALE_COMMIT: equivalence}
    honest = _record(
        source_commit=_STALE_COMMIT, commit_equivalence=dict(equivalence)
    )
    result = evaluate_release([honest], requirements)
    assert result["status"] == "pass"
    silent = _record(source_commit=_STALE_COMMIT)
    result = evaluate_release([silent], requirements)
    assert result["status"] == "blocked"


# ---------------------------------------------------------------------------
# Evidence file / hash binding.
# ---------------------------------------------------------------------------


def test_manually_typed_pass_without_evidence_does_not_satisfy():
    """status=pass typed by hand, with no evidence file/hash binding, is
    invalid — it cannot satisfy the required gate."""
    record = _record()
    del record["evidence"]
    del record["evidence_sha256"]
    result = evaluate_release(
        [record, _two_gate_records()[1]], _two_gate_requirements()
    )
    assert result["status"] == "blocked"
    assert any("evidence" in r for r in _failed_reasons(result))


def test_record_without_evidence_hash_is_invalid():
    record = _record()
    del record["evidence_sha256"]
    result = evaluate_release(
        [record, _two_gate_records()[1]], _two_gate_requirements()
    )
    assert result["status"] == "blocked"
    assert any("sha256" in r for r in _failed_reasons(result))


def test_missing_evidence_file_under_evidence_dir_is_invalid(tmp_path):
    result = evaluate_release(
        _two_gate_records(), _two_gate_requirements(), evidence_dir=tmp_path
    )
    assert result["status"] == "blocked"
    assert any("missing" in r for r in _failed_reasons(result))


def test_evidence_file_hash_mismatch_is_invalid(tmp_path):
    (tmp_path / "linux-native.json").write_text("real evidence")
    (tmp_path / "darwin-native.json").write_text("real evidence")
    result = evaluate_release(
        _two_gate_records(), _two_gate_requirements(), evidence_dir=tmp_path
    )
    assert result["status"] == "blocked"
    assert any("hash" in r for r in _failed_reasons(result))


def test_matching_evidence_file_and_hash_satisfies(tmp_path):
    records = _two_gate_records()
    for record in records:
        payload = f"evidence for {record['platform']}"
        (tmp_path / record["evidence"]).write_text(payload)
        record["evidence_sha256"] = hashlib.sha256(
            payload.encode()
        ).hexdigest()
    result = evaluate_release(
        records, _two_gate_requirements(), evidence_dir=tmp_path
    )
    assert result["status"] == "pass"


def test_evidence_object_form_binds_path_and_hash(tmp_path):
    record = _record()
    record["evidence"] = {
        "path": "linux-native.json",
        "sha256": record.pop("evidence_sha256"),
    }
    (tmp_path / "linux-native.json").write_text("real evidence")
    record["evidence"]["sha256"] = hashlib.sha256(b"real evidence").hexdigest()
    darwin = _two_gate_records()[1]
    result = evaluate_release(
        [record, darwin], _two_gate_requirements(), evidence_dir=tmp_path
    )
    # the Darwin record still fails on its absent file; the Linux record
    # with the object-form evidence binding verifies
    linux_eval = next(
        entry for entry in result["records"]
        if entry["key"]["platform"] == "Linux"
    )
    assert linux_eval["verdict"] == "satisfied"
    assert result["status"] == "blocked"


# ---------------------------------------------------------------------------
# Counts and status semantics.
# ---------------------------------------------------------------------------


def test_zero_tests_fails_the_record():
    records = _two_gate_records()
    records[0]["counts"] = {"collected": 0, "executed": 0}
    result = evaluate_release(records, _two_gate_requirements())
    assert result["status"] == "blocked"
    assert any("zero tests" in r for r in _failed_reasons(result))


def test_failed_record_fails_its_gate():
    records = _two_gate_records()
    records[0]["status"] = "fail"
    records[0]["returncode"] = 1
    result = evaluate_release(records, _two_gate_requirements())
    assert result["status"] == "blocked"
    assert len(result["failed"]) == 1


def test_status_pass_with_nonzero_exit_is_invalid():
    records = _two_gate_records()
    records[0]["returncode"] = 1
    result = evaluate_release(records, _two_gate_requirements())
    assert result["status"] == "blocked"
    assert any("contradicts" in r for r in _failed_reasons(result))


# ---------------------------------------------------------------------------
# baseline_unchanged — only the approved macOS full baseline row.
# ---------------------------------------------------------------------------

_BASELINE_ROW = {
    "gate": "macos-full",
    "platform": "Darwin",
    "machine": "arm64",
    "profile": "full",
    "toolchain": "AppleClang clang++",
    "mode": None,
    "baseline_unchanged": True,
}


def _baseline_record(**overrides):
    record = _record(
        gate="macos-full",
        platform="Darwin",
        machine="arm64",
        profile="full",
        toolchain="AppleClang clang++",
        status="baseline_unchanged",
        returncode=1,
        approved=True,
        evidence="darwin-full-approved-baseline.json",
    )
    record.update(overrides)
    return record


def test_linux_baseline_unchanged_is_invalid():
    """Linux required gates are pass-only — no new Linux baseline is
    accepted by analogy with the macOS one."""
    requirements = _requirements(
        {
            "gate": "native", "platform": "Linux", "machine": "x86_64",
            "profile": "native", "toolchain": "clang++", "mode": None,
        },
        {
            "gate": "full", "platform": "Linux", "machine": "x86_64",
            "profile": "full", "toolchain": "clang++", "mode": None,
        },
    )
    records = [
        _record(),
        _record(
            gate="full", profile="full", status="baseline_unchanged",
            returncode=1, approved=True, evidence="linux-full.json",
        ),
    ]
    result = evaluate_release(records, requirements)
    assert result["status"] == "blocked"
    assert any("pass-only" in r for r in _failed_reasons(result))


def test_approved_macos_full_baseline_row_accepts_baseline_unchanged():
    requirements = _requirements(
        {
            "gate": "native", "platform": "Linux", "machine": "x86_64",
            "profile": "native", "toolchain": "clang++", "mode": None,
        },
        dict(_BASELINE_ROW),
    )
    result = evaluate_release(
        [_record(), _baseline_record()], requirements
    )
    assert result["status"] == "pass"
    baseline_entry = next(
        entry for entry in result["satisfied"]
        if entry["required"]["gate"] == "macos-full"
    )
    assert baseline_entry["status"] == "baseline_unchanged"
    # explicitly non-green — never reported as a passing suite
    assert baseline_entry["green"] is False


def test_unapproved_baseline_unchanged_is_invalid_even_on_the_baseline_row():
    requirements = _requirements(dict(_BASELINE_ROW))
    result = evaluate_release(
        [_baseline_record(approved=False)], requirements
    )
    assert result["status"] == "blocked"
    assert any("approved" in r for r in _failed_reasons(result))


def test_baseline_unchanged_on_a_plain_row_is_invalid():
    """Only the designated baseline row may stay baseline_unchanged."""
    requirements = _requirements(
        {
            "gate": "native", "platform": "Darwin", "machine": "arm64",
            "profile": "full", "toolchain": "AppleClang clang++",
            "mode": None,
        },
    )
    record = _baseline_record(gate="native")
    result = evaluate_release([record], requirements)
    assert result["status"] == "blocked"
    assert any("baseline" in r for r in _failed_reasons(result))


# ---------------------------------------------------------------------------
# Missing capability rows.
# ---------------------------------------------------------------------------


def test_missing_numerical_qualification_blocks():
    requirements = _requirements(
        {
            "gate": "native", "platform": "Linux", "machine": "x86_64",
            "profile": "native", "toolchain": "clang++", "mode": None,
        },
        {
            "gate": "numerical-qualification", "platform": "Linux",
            "machine": "x86_64", "profile": "native", "toolchain": "clang++",
            "mode": None,
        },
    )
    result = evaluate_release([_record()], requirements)
    assert result["status"] == "blocked"
    assert any(
        item["gate"] == "numerical-qualification" for item in result["missing"]
    )


def test_missing_gui_row_blocks():
    requirements = _requirements(
        {
            "gate": "native", "platform": "Linux", "machine": "x86_64",
            "profile": "native", "toolchain": "clang++", "mode": None,
        },
        {
            "gate": "linux-manual-gui", "platform": "Linux",
            "machine": "x86_64", "profile": "native", "toolchain": "clang++",
            "mode": None,
        },
    )
    result = evaluate_release([_record()], requirements)
    assert result["status"] == "blocked"
    assert any(
        item["gate"] == "linux-manual-gui" for item in result["missing"]
    )


def test_missing_offline_mode_blocks():
    """full+slim are distinct mode rows — rehearsing only one leaves the
    other mode missing, not covered."""
    requirements = _requirements(
        {
            "gate": "linux-offline", "platform": "Linux",
            "machine": "x86_64", "profile": "offline", "toolchain": None,
            "mode": "full",
        },
        {
            "gate": "linux-offline", "platform": "Linux",
            "machine": "x86_64", "profile": "offline", "toolchain": None,
            "mode": "slim",
        },
    )
    record = _record(
        gate="linux-offline",
        profile="offline",
        evidence="linux-offline-full.log",
    )
    del record["toolchain"]  # the offline row binds no toolchain
    record["mode"] = "full"
    result = evaluate_release([record], requirements)
    assert result["status"] == "blocked"
    assert any(item["mode"] == "slim" for item in result["missing"])


# ---------------------------------------------------------------------------
# Sanitizer negative control.
# ---------------------------------------------------------------------------

_ASAN_ROW = {
    "gate": "linux-asan",
    "platform": "Linux",
    "machine": "x86_64",
    "profile": "native",
    "toolchain": "clang++",
    "mode": None,
    "negative_control": True,
}


def test_invalid_sanitizer_negative_control_fails():
    """A green sanitizer run proves nothing without the negative control
    showing the build detects an injected fault."""
    requirements = _requirements(dict(_ASAN_ROW))
    record = _record(gate="linux-asan", evidence="linux-asan.json")
    result = evaluate_release([record], requirements)
    assert result["status"] == "blocked"
    assert any("negative control" in r for r in _failed_reasons(result))
    # a control that 'ran green' (detected false / zero exit) is invalid
    bad = _record(
        gate="linux-asan",
        evidence="linux-asan.json",
        negative_control={"detected": False, "returncode": 0},
    )
    result = evaluate_release([bad], requirements)
    assert result["status"] == "blocked"
    assert any("negative control" in r for r in _failed_reasons(result))


def test_valid_sanitizer_negative_control_satisfies():
    requirements = _requirements(dict(_ASAN_ROW))
    record = _record(
        gate="linux-asan",
        evidence="linux-asan.json",
        negative_control={
            "detected": True,
            "returncode": 1,
            "signature": "AddressSanitizer: heap-use-after-free",
        },
    )
    result = evaluate_release([record], requirements)
    assert result["status"] == "pass"


# ---------------------------------------------------------------------------
# Ambiguity and extras.
# ---------------------------------------------------------------------------


def test_duplicate_conflicting_records_are_ambiguous():
    """Two records carrying the same gate key cannot both be the evidence —
    the conflict is rejected, never silently picked."""
    records = _two_gate_records()
    conflict = _record(status="fail", returncode=1, evidence="other.json")
    result = evaluate_release(
        [records[0], conflict, records[1]], _two_gate_requirements()
    )
    assert result["status"] == "blocked"
    assert any("ambiguous" in r for r in _failed_reasons(result))


def test_extra_records_never_satisfy_required_rows():
    """A foreign-platform record is reported as extra; required rows stay
    missing."""
    result = evaluate_release(
        [_record(platform="FreeBSD", machine="amd64")],
        _two_gate_requirements(),
    )
    assert result["status"] == "blocked"
    assert len(result["missing"]) == 2
    assert len(result["extra"]) == 1


def test_invalid_extra_record_is_a_problem():
    requirements = _requirements(
        {
            "gate": "native", "platform": "Linux", "machine": "x86_64",
            "profile": "native", "toolchain": "clang++", "mode": None,
        },
    )
    records = [_record(), {"gate": "unbound-extra", "status": "pass"}]
    result = evaluate_release(records, requirements)
    assert result["status"] == "blocked"
    assert any("extra" in p or "no required" in p for p in result["problems"])


def test_requirements_with_duplicate_rows_are_rejected():
    requirements = _requirements(
        {"gate": "native", "platform": "Linux"},
        {"gate": "native", "platform": "linux"},  # same row after alias fold
    )
    result = evaluate_release([], requirements)
    assert result["status"] == "blocked"
    assert any("duplicate" in p for p in result["problems"])


def test_missing_gates_is_a_required_set_comparison():
    """The verbatim helper computes only missing rows — present keys are
    'covered' for the purposes of the missing list even when the record is
    later judged invalid."""
    requirements = {
        "required": [
            {"gate": "native", "platform": "Linux"},
            {"gate": "native", "platform": "Darwin"},
        ]
    }
    records = [{"gate": "native", "platform": "Linux"}]
    missing = _missing_gates(records, requirements)
    assert missing == [{"gate": "native", "platform": "Darwin"}]


# ---------------------------------------------------------------------------
# Generated required set from the approved toolchain matrix.
# ---------------------------------------------------------------------------


def test_generated_requirements_cover_the_approved_matrix():
    matrix = json.loads(MATRIX_DOC.read_text())
    requirements = release_requirements(matrix, _COMMIT)
    required = requirements["required"]
    optional = requirements["optional"]
    keys = {
        (row["gate"], row["platform"], row["machine"], row["mode"])
        for row in required
    }
    # every hosted and trusted lane is a required row on its release machine
    for lane in matrix["lanes"]:
        assert any(row["gate"] == lane for row in required), lane
    # Debug+Release compiler coverage on Linux and the macOS lanes
    assert ("linux-clang-release", "Linux", "x86_64", None) in keys
    assert ("linux-gcc-debug", "Linux", "x86_64", None) in keys
    assert ("macos-appleclang-release", "Darwin", "arm64", None) in keys
    # both offline install modes per platform
    assert ("linux-offline", "Linux", "x86_64", "full") in keys
    assert ("linux-offline", "Linux", "x86_64", "slim") in keys
    assert ("macos-offline", "Darwin", "arm64", "full") in keys
    assert ("macos-offline", "Darwin", "arm64", "slim") in keys
    # real viewers on both platforms
    assert any(row["gate"] == "linux-manual-gui" for row in required)
    assert any(row["gate"] == "macos-manual-gui" for row in required)
    # numerical qualification + docs rehearsal per platform
    for platform, machine in (("Linux", "x86_64"), ("Darwin", "arm64")):
        assert (
            "numerical-qualification", platform, machine, None
        ) in keys
        assert ("docs-rehearsal", platform, machine, None) in keys
    # sanitizer lanes demand the negative control
    for row in required:
        if "asan" in row["gate"] or "rtsan" in row["gate"]:
            assert row.get("negative_control") is True, row
        else:
            assert row.get("negative_control") is not True, row
    # only the approved macOS full baseline may stay baseline_unchanged
    baseline_rows = [
        row for row in required if row.get("baseline_unchanged") is True
    ]
    assert [row["gate"] for row in baseline_rows] == ["macos-full"]
    # additional declared architectures are optional advertising gates
    assert optional
    assert all(
        row["machine"] == "aarch64" and row["platform"] == "Linux"
        for row in optional
    )
    # no dimension collapses: every row binds gate/platform/machine/profile
    for row in [*required, *optional]:
        for name in ("gate", "platform", "machine", "profile"):
            assert isinstance(row[name], str) and row[name], (name, row)
    # and no ambiguous duplicate required keys
    dimensions = (
        "gate", "platform", "machine", "profile", "toolchain", "mode",
    )
    all_keys = [
        tuple(row.get(name) for name in dimensions)
        for row in [*required, *optional]
    ]
    assert len(all_keys) == len(set(all_keys))


def test_generated_requirements_pass_release_when_all_rows_satisfied(
    tmp_path,
):
    """End-to-end over the real matrix: one bound record + evidence file
    per required row releases."""
    matrix = json.loads(MATRIX_DOC.read_text())
    requirements = release_requirements(matrix, _COMMIT)
    records = []
    for index, row in enumerate(requirements["required"]):
        evidence_name = f"evidence-{index}.log"
        payload = f"evidence for {row['gate']} {row['platform']} {row['mode']}"
        (tmp_path / evidence_name).write_text(payload)
        record = {
            "gate": row["gate"],
            "platform": row["platform"],
            "machine": row["machine"],
            "profile": row["profile"],
            "status": "pass",
            "source_commit": _COMMIT,
            "command": ["qualified", "command", row["gate"]],
            "returncode": 0,
            "counts": {"collected": 10, "executed": 10},
            "artifact": {"identity": f"artifacts/{row['gate']}"},
            "evidence": evidence_name,
            "evidence_sha256": hashlib.sha256(payload.encode()).hexdigest(),
            "_file": f"record-{index}.json",
        }
        if row["toolchain"] is not None:
            record["toolchain"] = row["toolchain"]
        if row["mode"] is not None:
            record["mode"] = row["mode"]
        if row.get("negative_control") is True:
            record["negative_control"] = {"detected": True, "returncode": 1}
        records.append(record)
    result = evaluate_release(records, requirements, evidence_dir=tmp_path)
    assert result["status"] == "pass", result["failed"] + [
        {"problems": result["problems"]}
    ]
    assert len(result["satisfied"]) == len(requirements["required"])
    assert result["optional"]["missing"], (
        "additional architectures are not silently advertised"
    )


def test_cli_release_blocked_on_empty_evidence_dir(tmp_path):
    output = tmp_path / "acceptance.json"
    rc = portability_qualification.main(
        [
            "--release",
            "--evidence",
            str(tmp_path / "evidence"),
            "--output",
            str(output),
            "--source-commit",
            _COMMIT,
        ]
    )
    assert rc == 1
    report = json.loads(output.read_text())
    assert report["status"] == "blocked"
    assert report["missing"]
    assert report["problems"]


def test_cli_release_pass_with_full_evidence(tmp_path):
    evidence_dir = tmp_path / "evidence"
    evidence_dir.mkdir()
    matrix = json.loads(MATRIX_DOC.read_text())
    requirements = release_requirements(matrix, _COMMIT)
    for index, row in enumerate(requirements["required"]):
        evidence_name = f"evidence-{index}.log"
        payload = f"evidence for {row['gate']} on {row['platform']}"
        (evidence_dir / evidence_name).write_text(payload)
        record = {
            "gate": row["gate"],
            "platform": row["platform"],
            "machine": row["machine"],
            "profile": row["profile"],
            "status": "pass",
            "source_commit": _COMMIT,
            "command": ["qualified", "command", row["gate"]],
            "returncode": 0,
            "counts": {"executed": 10},
            "artifact": {"identity": f"artifacts/{row['gate']}"},
            "evidence": evidence_name,
            "evidence_sha256": hashlib.sha256(payload.encode()).hexdigest(),
        }
        if row["toolchain"] is not None:
            record["toolchain"] = row["toolchain"]
        if row["mode"] is not None:
            record["mode"] = row["mode"]
        if row.get("negative_control") is True:
            record["negative_control"] = {"detected": True, "returncode": 1}
        (evidence_dir / f"record-{index}.json").write_text(
            json.dumps(record)
        )
    # a non-record document in the dir is ignored, not misread as a gate
    (evidence_dir / "full-result.json").write_text(
        json.dumps({"profile": "full", "status": "fail"})
    )
    output = tmp_path / "acceptance.json"
    rc = portability_qualification.main(
        [
            "--release",
            "--evidence",
            str(evidence_dir),
            "--output",
            str(output),
            "--matrix",
            str(MATRIX_DOC),
            "--source-commit",
            _COMMIT,
        ]
    )
    assert rc == 0
    report = json.loads(output.read_text())
    assert report["status"] == "pass"
    assert "full-result.json" in report["ignored_evidence_files"]
    assert report["matrix"]["sha256"]


def test_cli_release_blocked_when_a_macos_record_is_missing(tmp_path):
    """The required-set comparison: deleting one macOS evidence record
    blocks the release even though every other file is present."""
    evidence_dir = tmp_path / "evidence"
    evidence_dir.mkdir()
    matrix = json.loads(MATRIX_DOC.read_text())
    requirements = release_requirements(matrix, _COMMIT)
    skipped = 0
    for index, row in enumerate(requirements["required"]):
        if row["gate"] == "macos-manual-gui":
            skipped = index
            continue
        evidence_name = f"evidence-{index}.log"
        payload = "evidence"
        (evidence_dir / evidence_name).write_text(payload)
        record = {
            "gate": row["gate"],
            "platform": row["platform"],
            "machine": row["machine"],
            "profile": row["profile"],
            "status": "pass",
            "source_commit": _COMMIT,
            "command": ["qualified", "command", row["gate"]],
            "returncode": 0,
            "counts": {"executed": 10},
            "artifact": {"identity": f"artifacts/{row['gate']}"},
            "evidence": evidence_name,
            "evidence_sha256": hashlib.sha256(payload.encode()).hexdigest(),
        }
        if row["toolchain"] is not None:
            record["toolchain"] = row["toolchain"]
        if row["mode"] is not None:
            record["mode"] = row["mode"]
        if row.get("negative_control") is True:
            record["negative_control"] = {"detected": True, "returncode": 1}
        (evidence_dir / f"record-{index}.json").write_text(
            json.dumps(record)
        )
    output = tmp_path / "acceptance.json"
    rc = portability_qualification.main(
        [
            "--release",
            "--evidence",
            str(evidence_dir),
            "--output",
            str(output),
            "--matrix",
            str(MATRIX_DOC),
            "--source-commit",
            _COMMIT,
        ]
    )
    assert rc == 1
    report = json.loads(output.read_text())
    assert report["status"] == "blocked"
    missing = report["missing"]
    assert len(missing) == 1 and missing[0]["gate"] == "macos-manual-gui"


def test_release_report_is_deterministic_and_complete(tmp_path):
    records = _two_gate_records()
    requirements = _two_gate_requirements()
    snapshot = json.loads(json.dumps(records))
    result = evaluate_release(records, requirements)
    assert result["schema_version"] == 1
    assert result["source_commit"] == _COMMIT
    assert result["required_total"] == 2
    assert len(result["records"]) == 2
    # input records are never mutated by the aggregation
    assert records == snapshot
    again = evaluate_release(_two_gate_records(), _two_gate_requirements())
    assert result == again
