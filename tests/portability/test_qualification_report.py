"""Contract tests for tools.portability_qualification (T27, Python-only).
These tests never import the native extension and never invoke
``tools/run_tests.sh``: they exercise the result-document schema and the
``classify_result`` comparison contract directly, plus the JUnit/stdout
evidence parsers on tiny fixtures. The quick profile must be able to run
them on a host where no native artifact exists.
"""
from __future__ import annotations

import json
import platform
import textwrap
import time
from pathlib import Path

import pytest

from tools.portability_qualification import (
    _failure_keys,
    _verify_native_provenance,
    classify_result,
    parse_junit,
    parse_pytest_summary,
)

REPO_ROOT = Path(__file__).resolve().parents[2]


# ---------------------------------------------------------------------------
# Result builders — a well-formed schema-1 document unless overridden.
# ---------------------------------------------------------------------------

_DARWIN_ENV = {
    "system": "Darwin",
    "machine": "arm64",
    "release": "25.0.0",
    "version": "Darwin Kernel Version 25.0.0",
    "platform": "macOS-26.0-arm64-arm-64bit",
    "python_version": "3.14.0",
    "python_implementation": "CPython",
    "compiler_id": "AppleClang",
    "compiler_version": "21.0.0",
}

_LINUX_ENV = {
    "system": "Linux",
    "machine": "x86_64",
    "release": "6.8.0-generic",
    "version": "#1-Ubuntu SMP",
    "platform": "Linux-6.8.0-generic-x86_64-with-glibc2.39",
    "python_version": "3.13.7",
    "python_implementation": "CPython",
    "compiler_id": "Clang",
    "compiler_version": "18.1.3",
}


def _environment(system="Darwin"):
    return dict(_DARWIN_ENV if system == "Darwin" else _LINUX_ENV)


def _native_record(**overrides):
    record = {
        "imported_extension_verified": True,
        "native_executed": 12,
        "native_test_count": 12,
        "selected_build": "/abs/native/build",
        "imported_extension": "/abs/native/build/bike_native.cpython-313.so",
    }
    record.update(overrides)
    return record


def _result(
    *,
    system="Darwin",
    profile="full",
    environment_id=None,
    returncode=0,
    collected=40,
    executed=40,
    skipped=0,
    xfailed=0,
    errored=0,
    deselected=0,
    collection_errors=0,
    failures=None,
    skips=None,
    xfails=None,
    errors=None,
    timed_out=False,
    signal=None,
    execution_complete=True,
    native=None,
    approved=None,
):
    result = {
        "schema_version": 1,
        "environment_id": (
            environment_id
            if environment_id is not None
            else f"{system.lower()}-qualified-bench"
        ),
        "environment": _environment(system),
        "profile": profile,
        "command": ["bash", "tools/run_tests.sh", profile],
        "returncode": returncode,
        "signal": signal,
        "timed_out": timed_out,
        "collected": collected,
        "executed": executed,
        "skipped": skipped,
        "xfailed": xfailed,
        "errored": errored,
        "deselected": deselected,
        "collection_errors": collection_errors,
        "failures": list(failures or []),
        "skips": list(skips or []),
        "xfails": list(xfails or []),
        "errors": list(errors or []),
        "execution_complete": execution_complete,
    }
    if profile in ("native", "full"):
        result["native"] = _native_record() if native is None else native
    if approved is not None:
        result["approved"] = approved
    return result


def _baseline(**overrides):
    baseline = _result(
        system="Darwin",
        profile="full",
        returncode=1,
        collected=40,
        executed=40,
        failures=[
            {
                "nodeid": "tests/reference/test_old.py::test_old",
                "signature": "assert old == known",
            }
        ],
        approved=True,
    )
    baseline.update(overrides)
    return baseline


def _failure(nodeid, signature):
    return {"nodeid": nodeid, "signature": signature}


# ---------------------------------------------------------------------------
# Plan step-1 examples — verbatim.
# ---------------------------------------------------------------------------


def test_new_failure_is_never_a_baseline_pass():
    baseline = {"environment_id": "mac-baseline", "profile": "full",
                "failures": [{"nodeid": "tests/test_old.py::test_old",
                              "signature": "known-check"}]}
    current = {"environment_id": "mac-baseline", "profile": "full",
               "returncode": 1, "collected": 20, "executed": 20,
               "failures": baseline["failures"] + [
                   {"nodeid": "tests/test_new.py::test_new", "signature": "new"}]}
    assert classify_result(current, baseline)["status"] == "fail"


def test_zero_tests_is_failure_even_with_zero_exit():
    current = {"environment_id": "linux", "profile": "native",
               "returncode": 0, "collected": 0, "executed": 0,
               "failures": []}
    assert classify_result(current, None)["status"] == "fail"


# ---------------------------------------------------------------------------
# Positive controls.
# ---------------------------------------------------------------------------


def test_clean_full_run_passes():
    verdict = classify_result(_result(), None)
    assert verdict["status"] == "pass"
    assert verdict["green"] is True


def test_clean_linux_native_run_passes():
    """Linux is not second-class: a clean required gate is a pass."""
    verdict = classify_result(
        _result(system="Linux", profile="native"), None
    )
    assert verdict["status"] == "pass"
    assert verdict["green"] is True


def test_exact_approved_darwin_full_baseline_is_unchanged():
    """The one reachable non-green verdict: identical environment/profile,
    approved baseline, complete execution, failures ⊆ baseline."""
    current = _result(
        returncode=1,
        failures=[
            _failure(
                "tests/reference/test_old.py::test_old",
                "assert old == known",
            )
        ],
    )
    verdict = classify_result(current, _baseline())
    assert verdict["status"] == "baseline_unchanged"
    # Raw evidence is retained and the verdict must not claim a green run.
    assert verdict["green"] is False
    assert verdict["returncode"] == 1
    assert verdict["failures"] == current["failures"]
    assert "not green" in verdict["statement"].lower()


def test_subset_of_baseline_failures_is_an_improvement_not_a_pass():
    baseline = _baseline(
        failures=[
            _failure("tests/reference/test_old.py::test_old", "assert old == known"),
            _failure("tests/reference/test_older.py::test_older", "assert older"),
        ]
    )
    current = _result(
        returncode=1,
        failures=[
            _failure("tests/reference/test_old.py::test_old", "assert old == known")
        ],
    )
    verdict = classify_result(current, baseline)
    assert verdict["status"] == "baseline_unchanged"
    assert "improvement" in verdict["statement"]


def test_all_baseline_failures_fixed_is_a_pass():
    current = _result(returncode=0, failures=[])
    verdict = classify_result(current, _baseline())
    assert verdict["status"] == "pass"


# ---------------------------------------------------------------------------
# Baseline comparison rejections.
# ---------------------------------------------------------------------------


def test_mismatched_environment_rejects_baseline():
    baseline = _baseline(environment_id="other-mac-bench")
    current = _result(returncode=1, failures=list(baseline["failures"]))
    verdict = classify_result(current, baseline)
    assert verdict["status"] == "fail"
    assert any("environment" in r for r in verdict["reasons"])


def test_mismatched_environment_record_rejects_baseline():
    """Same environment_id but a changed recorded toolchain is a
    different qualified environment."""
    baseline = _baseline()
    baseline["environment"] = _environment()
    baseline["environment"]["compiler_version"] = "20.0.0"
    current = _result(returncode=1, failures=list(baseline["failures"]))
    assert classify_result(current, baseline)["status"] == "fail"


def test_mismatched_profile_rejects_baseline():
    """A native-profile result cannot inherit the full-profile baseline."""
    baseline = _baseline()
    current = _result(
        profile="native",
        returncode=1,
        collected=30,
        executed=30,
        failures=list(baseline["failures"]),
    )
    verdict = classify_result(current, baseline)
    assert verdict["status"] == "fail"
    assert any("full" in r for r in verdict["reasons"])


def test_quick_profile_rejects_baseline():
    baseline = _baseline(profile="quick")
    current = _result(
        profile="quick",
        returncode=1,
        collected=30,
        executed=30,
        failures=list(baseline["failures"]),
    )
    current.pop("native", None)
    assert classify_result(current, baseline)["status"] == "fail"


def test_changed_failure_signature_is_a_new_failure():
    current = _result(
        returncode=1,
        failures=[
            _failure(
                "tests/reference/test_old.py::test_old",
                "assert old == different",
            )
        ],
    )
    verdict = classify_result(current, _baseline())
    assert verdict["status"] == "fail"
    assert any("new failure" in r for r in verdict["reasons"])


def test_missing_baseline_evidence_fails():
    current = _result(
        returncode=1,
        failures=[
            _failure("tests/reference/test_old.py::test_old", "assert old == known")
        ],
    )
    for baseline in ({}, {"approved": True}, {"failures": []}, "not-a-dict"):
        assert classify_result(current, baseline)["status"] == "fail"


def test_unapproved_baseline_cannot_classify_unchanged():
    """The classifier never blesses current failures as a new baseline."""
    current = _result(returncode=1, failures=list(_baseline()["failures"]))
    for approved in (None, False):
        baseline = _baseline(approved=approved)
        verdict = classify_result(current, baseline)
        assert verdict["status"] == "fail"
        assert any("approved" in r for r in verdict["reasons"])


def test_collection_error_is_never_a_baseline_pass():
    current = _result(
        returncode=1,
        collection_errors=1,
        failures=list(_baseline()["failures"]),
    )
    verdict = classify_result(current, _baseline())
    assert verdict["status"] == "fail"
    assert any("collection" in r for r in verdict["reasons"])


def test_new_errored_test_rejects_baseline():
    current = _result(
        returncode=1,
        collected=41,
        executed=41,
        errored=1,
        failures=list(_baseline()["failures"]),
        errors=[
            {
                "nodeid": "tests/reference/test_new.py::test_new",
                "signature": "fixture 'x' not found",
            }
        ],
    )
    verdict = classify_result(current, _baseline())
    assert verdict["status"] == "fail"
    assert any("error" in r for r in verdict["reasons"])


def test_deselected_all_is_failure():
    """Every test deselected: zero collected with a clean exit is a fail."""
    current = _result(
        profile="quick",
        returncode=0,
        collected=0,
        executed=0,
        deselected=80,
    )
    current.pop("native", None)
    verdict = classify_result(current, None)
    assert verdict["status"] == "fail"
    assert any("executed" in r for r in verdict["reasons"])


def test_missing_imported_extension_fails_native():
    """A native run whose selected artifact was not verifiably imported is
    never a pass — exactly the Python-fallback / loader-crash case."""
    current = _result(
        profile="native",
        native=_native_record(imported_extension_verified=False),
    )
    verdict = classify_result(current, None)
    assert verdict["status"] == "fail"
    assert any("artifact" in r or "imported" in r for r in verdict["reasons"])


def test_tooling_only_native_tests_do_not_qualify():
    """test_native_* tooling modules ran but zero direct-oracle tests."""
    current = _result(
        profile="native",
        native=_native_record(native_executed=0, native_test_count=3),
    )
    verdict = classify_result(current, None)
    assert verdict["status"] == "fail"
    assert any("oracle" in r or "tooling" in r for r in verdict["reasons"])


def test_unexpected_skip_rejects_baseline():
    current = _result(
        returncode=1,
        collected=41,
        executed=40,
        skipped=1,
        failures=list(_baseline()["failures"]),
        skips=[
            {"nodeid": "tests/reference/test_x.py::test_x", "reason": "bored"}
        ],
    )
    verdict = classify_result(current, _baseline())
    assert verdict["status"] == "fail"
    assert any("skip" in r for r in verdict["reasons"])


def test_xfail_inflation_rejects_baseline():
    """A new xfail hides a failing-or-missing test behind an expectation."""
    current = _result(
        returncode=1,
        collected=41,
        executed=40,
        xfailed=1,
        failures=list(_baseline()["failures"]),
        xfails=[{"nodeid": "tests/reference/test_y.py::test_y"}],
    )
    verdict = classify_result(current, _baseline())
    assert verdict["status"] == "fail"
    assert any("xfail" in r for r in verdict["reasons"])


def test_timeout_is_failure_even_with_matching_failures():
    current = _result(
        returncode=None,
        timed_out=True,
        failures=list(_baseline()["failures"]),
    )
    verdict = classify_result(current, _baseline())
    assert verdict["status"] == "fail"
    assert any("timeout" in r for r in verdict["reasons"])


def test_signal_exit_is_failure():
    current = _result(
        returncode=-9,
        signal="SIGKILL",
        failures=list(_baseline()["failures"]),
    )
    verdict = classify_result(current, _baseline())
    assert verdict["status"] == "fail"
    assert any("signal" in r for r in verdict["reasons"])


def test_pytest_usage_exit_is_failure():
    current = _result(returncode=4, collected=0, executed=0)
    verdict = classify_result(current, None)
    assert verdict["status"] == "fail"


def test_linux_can_never_be_baseline_unchanged():
    """Linux required gates are pass-only — even a perfectly matching
    approved Linux baseline cannot produce baseline_unchanged."""
    baseline = _result(
        system="Linux",
        profile="full",
        returncode=1,
        collected=40,
        executed=40,
        failures=[
            _failure("tests/reference/test_old.py::test_old", "assert old")
        ],
        approved=True,
    )
    current = _result(
        system="Linux",
        profile="full",
        returncode=1,
        failures=list(baseline["failures"]),
    )
    verdict = classify_result(current, baseline)
    assert verdict["status"] == "fail"
    assert any("pass-only" in r or "Darwin" in r for r in verdict["reasons"])


def test_partial_result_must_not_hide_unexecuted_tests():
    """Running a subset of the baseline's suite and matching its failures
    is concealment, not baseline_unchanged."""
    current = _result(
        returncode=1,
        collected=20,
        executed=20,
        failures=list(_baseline()["failures"]),
    )
    verdict = classify_result(current, _baseline())
    assert verdict["status"] == "fail"
    assert any("fewer" in r for r in verdict["reasons"])


def test_incomplete_execution_rejects_baseline():
    current = _result(
        returncode=1,
        execution_complete=False,
        failures=list(_baseline()["failures"]),
    )
    assert classify_result(current, _baseline())["status"] == "fail"


def test_missing_fields_are_missing_evidence():
    current = _result()
    del current["environment"]
    del current["execution_complete"]
    verdict = classify_result(current, None)
    assert verdict["status"] == "fail"
    assert any("environment" in r for r in verdict["reasons"])


def test_inconsistent_counts_are_missing_evidence():
    current = _result(collected=40, executed=30, skipped=0, xfailed=0)
    verdict = classify_result(current, None)
    assert verdict["status"] == "fail"
    assert any("inconsistent" in r for r in verdict["reasons"])


# ---------------------------------------------------------------------------
# _failure_keys — exact identity, verbatim helper contract.
# ---------------------------------------------------------------------------


def test_failure_keys_rejects_empty_and_duplicate_evidence():
    with pytest.raises(ValueError, match="nonempty"):
        _failure_keys({"failures": [{"nodeid": "", "signature": "s"}]})
    with pytest.raises(ValueError, match="nonempty"):
        _failure_keys({"failures": [{"nodeid": "n", "signature": ""}]})
    with pytest.raises(ValueError, match="duplicate"):
        _failure_keys(
            {
                "failures": [
                    {"nodeid": "n", "signature": "s"},
                    {"nodeid": "n", "signature": "s"},
                ]
            }
        )


def test_failure_keys_same_nodeid_different_signature_is_distinct():
    keys = _failure_keys(
        {
            "failures": [
                {"nodeid": "n", "signature": "first"},
                {"nodeid": "n", "signature": "second"},
            ]
        }
    )
    assert keys == {("n", "first"), ("n", "second")}


# ---------------------------------------------------------------------------
# Evidence parsers — tiny fixtures, no test-runner invocation.
# ---------------------------------------------------------------------------

_JUNIT_XML = """<?xml version="1.0" encoding="utf-8"?>
<testsuites name="pytest tests">
  <testsuite name="pytest" errors="0" failures="1" skipped="2" tests="5">
    <testcase classname="tests.portability.test_qualification_report" name="test_ok" time="0.0"/>
    <testcase classname="tests.portability.test_qualification_report" name="test_bad" time="0.0">
      <failure message="assert 1 == 2">E   assert 1 == 2</failure>
    </testcase>
    <testcase classname="tests.reference.test_native_tire" name="test_native" time="0.0"/>
    <testcase classname="tests.portability.test_qualification_report" name="test_skip" time="0.0">
      <skipped type="pytest.skip" message="needs display"/>
    </testcase>
    <testcase classname="tests.portability.test_qualification_report" name="test_xf" time="0.0">
      <skipped type="pytest.xfail" message="known"/>
    </testcase>
  </testsuite>
</testsuites>
"""


def test_parse_junit_counts_and_evidence(tmp_path):
    junit = tmp_path / "report.xml"
    junit.write_text(_JUNIT_XML)
    parsed = parse_junit(junit, REPO_ROOT)
    assert parsed["present"] is True
    assert parsed["collected"] == 5
    # executed = passed + failed + errored (skipped/xfailed did not run)
    assert parsed["executed"] == 3
    assert parsed["skipped"] == 1
    assert parsed["xfailed"] == 1
    assert parsed["failed"] == 1
    assert parsed["native_executed"] == 1
    assert parsed["failures"] == [
        {
            "nodeid": "tests/portability/test_qualification_report.py::test_bad",
            "signature": "assert 1 == 2",
        }
    ]
    assert parsed["skips"][0]["nodeid"].endswith("::test_skip")
    assert parsed["xfails"][0]["nodeid"].endswith("::test_xf")


def test_parse_junit_error_events(tmp_path):
    junit = tmp_path / "report.xml"
    junit.write_text(
        """<?xml version="1.0"?>
<testsuites><testsuite name="pytest" tests="1">
  <testcase classname="tests.test_x" name="test_e" time="0.0">
    <error message="fixture 'f' not found">boom</error>
  </testcase>
</testsuite></testsuites>
"""
    )
    parsed = parse_junit(junit, REPO_ROOT)
    assert parsed["errored"] == 1
    assert parsed["errors"][0]["signature"] == "fixture 'f' not found"


def test_parse_junit_missing_or_invalid(tmp_path):
    missing = parse_junit(tmp_path / "nope.xml", REPO_ROOT)
    assert missing["present"] is False
    assert missing["error"] is not None
    bad = tmp_path / "bad.xml"
    bad.write_text("<testsuite>")
    assert parse_junit(bad, REPO_ROOT)["error"] is not None


def test_parse_pytest_summary_counts():
    stdout = textwrap.dedent(
        """\
        ....F..sx                                                        [100%]
        ===== 1 failed, 6 passed, 1 skipped, 1 xfailed, 2 deselected in 0.42s =====
        """
    )
    summary = parse_pytest_summary(stdout)
    assert summary == {
        "failed": 1,
        "passed": 6,
        "skipped": 1,
        "xfailed": 1,
        "deselected": 2,
        "collection_marked": 0,
    }


def test_parse_pytest_summary_bare_count_line():
    """Modern pytest prints the count line without ``=`` padding and may
    append an (h:mm:ss) wall-clock suffix."""
    stdout = (
        "....                                                                     [100%]\n"
        "3 failed, 1699 passed, 9 skipped, 70 deselected, 74 warnings "
        "in 836.34s (0:13:56)\n"
    )
    summary = parse_pytest_summary(stdout)
    assert summary["failed"] == 3
    assert summary["passed"] == 1699
    assert summary["skipped"] == 9
    assert summary["deselected"] == 70
    assert summary["warnings"] == 74


def test_parse_pytest_summary_collection_errors():
    stdout = (
        "!!!! Interrupted: 2 errors during collection !!!!\n"
        "===== 2 errors in 0.03s =====\n"
    )
    summary = parse_pytest_summary(stdout)
    assert summary["errors"] == 2
    assert summary["collection_marked"] == 2


def test_parse_pytest_summary_rejects_non_summary_lines():
    """A line that merely mentions a duration is not the count line."""
    assert parse_pytest_summary("simulation ran in 0.05s\n") is None
    assert parse_pytest_summary("no summary here\n") is None


# ---------------------------------------------------------------------------
# Native provenance verification — fixture files only, no real extension.
# ---------------------------------------------------------------------------


def _provenance_fixture(tmp_path, **overrides):
    build = (tmp_path / "native build").resolve()
    build.mkdir()
    extension = build / "bike_native.cpython-313-darwin.so"
    extension.write_bytes(b"fixture")
    provenance_path = tmp_path / "native_test_provenance.json"
    document = {
        "schema_version": 1,
        "selected_build": str(build),
        "imported_extension": str(extension),
        "native_test_count": 7,
        "platform": {
            "system": platform.system(),
            "machine": platform.machine(),
        },
        "python": {"version": platform.python_version()},
    }
    document.update(overrides)
    provenance_path.write_text(json.dumps(document))
    return build, provenance_path


def test_fresh_matching_provenance_verifies(tmp_path):
    build, provenance = _provenance_fixture(tmp_path)
    record = _verify_native_provenance(build, provenance, time.time() - 0.5)
    assert record["imported_extension_verified"] is True
    assert record["problems"] == []
    assert record["native_test_count"] == 7


def test_stale_provenance_does_not_certify_this_run(tmp_path):
    build, provenance = _provenance_fixture(tmp_path)
    # A record left over from an earlier run must never certify this one.
    record = _verify_native_provenance(
        build, provenance, time.time() + 60
    )
    assert record["imported_extension_verified"] is False
    assert any("stale" in problem for problem in record["problems"])


def test_other_platform_provenance_does_not_verify(tmp_path):
    other = "Linux" if platform.system() == "Darwin" else "Darwin"
    build, provenance = _provenance_fixture(
        tmp_path,
        platform={"system": other, "machine": platform.machine()},
    )
    record = _verify_native_provenance(build, provenance, time.time() - 0.5)
    assert record["imported_extension_verified"] is False
    assert any("platform" in problem for problem in record["problems"])


def test_zero_native_count_does_not_verify(tmp_path):
    build, provenance = _provenance_fixture(tmp_path, native_test_count=0)
    record = _verify_native_provenance(build, provenance, time.time() - 0.5)
    assert record["imported_extension_verified"] is False


def test_missing_or_foreign_extension_does_not_verify(tmp_path):
    build, provenance = _provenance_fixture(
        tmp_path, imported_extension=str(tmp_path / "elsewhere.so")
    )
    record = _verify_native_provenance(build, provenance, time.time() - 0.5)
    assert record["imported_extension_verified"] is False
    missing_provenance = _verify_native_provenance(
        build, tmp_path / "absent.json", time.time()
    )
    assert missing_provenance["imported_extension_verified"] is False
