"""Cross-platform test qualification and exact baseline comparison (T27).

This tool owns result comparison/reporting for the Linux+macOS port. It is
*not* a test runner: the only execution path is the existing
``bash tools/run_tests.sh <profile>`` entry point via
``subprocess.run(check=False)`` — never a weakened replacement. The wrapper
adds a pytest JUnit artifact, captures stdout/stderr verbatim, verifies the
selected native artifact's provenance for native/full profiles, stores
provider/compiler/CPU/source/lock identity, and classifies the run.

Classification contract (``classify_result``):

* malformed or missing evidence, timeouts, signal exits, tool/usage exit
  codes, collection/import errors and incomplete executions are rejected
  first — they can never be classified pass or baseline_unchanged;
* ``pass`` requires zero failures, zero errors, exit 0, complete execution
  and the required executed tests (nonzero ``executed`` everywhere; for
  native/full also a verified imported selected artifact and a nonzero
  executed direct-oracle set — tooling-only ``test_native_*.py`` modules
  do not qualify);
* ``baseline_unchanged`` requires an independently approved
  (``approved is True``) macOS ``full`` baseline recorded on the *exact*
  same environment, a complete current execution covering at least the
  baseline's collected/executed counts, raw exit 1, and
  ``_failure_keys(current) ⊆ _failure_keys(baseline)`` with no new
  skip/xfail/error. The raw exit and failures are retained and the verdict
  carries an explicit non-green statement — it is never "all tests passed";
* everything else is ``fail``. Linux required gates are pass-only: a
  non-Darwin environment can never be baseline_unchanged, and removed
  baseline failures are improvements inside the subset rule, while a
  partial result must not hide unexecuted tests.

CLI:
    python tools/portability_qualification.py --profile quick|native|full \
        --build PATH --output PATH [--baseline PATH] [--timeout SECONDS] \
        [-- EXTRA_PYTEST_ARGS...]

Exit status is 0 for pass and baseline_unchanged, 1 for fail, 2 for usage
errors. The JSON result file is the authoritative record.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import signal
import subprocess
import time
import xml.etree.ElementTree as ElementTree
from pathlib import Path

SCHEMA_VERSION = 1
PROFILES = ("quick", "native", "full")
_BASELINE_SYSTEM = "Darwin"
_BASELINE_PROFILE = "full"
_STATUS_PASS = "pass"
_STATUS_BASELINE = "baseline_unchanged"
_STATUS_FAIL = "fail"

_REPO_ROOT = Path(__file__).resolve().parents[1]

# Mirrors tests/reference/native_test_reporter.py::_NATIVE_TOOL_MODULES:
# these modules exercise the tooling around the extension, not the
# extension's own arithmetic — they never count as direct-oracle evidence.
_NATIVE_TOOL_MODULES = {
    "test_native_check_tools.py",
    "test_native_contract_checks.py",
    "test_native_loader.py",
}

_COUNTER_FIELDS = (
    "collected",
    "executed",
    "skipped",
    "xfailed",
    "errored",
    "deselected",
    "collection_errors",
)
_EVENT_LISTS = ("failures", "skips", "xfails", "errors")
_PYTEST_TOOL_EXIT_CODES = {2, 3, 4, 5}


def _failure_keys(value: dict) -> set[tuple[str, str]]:
    keys = set()
    for failure in value["failures"]:
        nodeid, signature = failure["nodeid"], failure["signature"]
        if not isinstance(nodeid, str) or not nodeid:
            raise ValueError("failure nodeid must be a nonempty string")
        if not isinstance(signature, str) or not signature:
            raise ValueError("failure signature must be a nonempty string")
        key = (nodeid, signature)
        if key in keys:
            raise ValueError("duplicate failure evidence")
        keys.add(key)
    return keys


def _node_keys(value: dict, field: str) -> set[str]:
    """The set of node ids recorded in a skip/xfail-style event list."""
    keys: set[str] = set()
    for entry in value[field]:
        nodeid = entry["nodeid"]
        if not isinstance(nodeid, str) or not nodeid:
            raise ValueError(f"{field} nodeid must be a nonempty string")
        if nodeid in keys:
            raise ValueError(f"duplicate {field} evidence")
        keys.add(nodeid)
    return keys


def _is_nonnegative_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _validate_result(value: object) -> list[str]:
    """Structural validation required before any classification.

    Returns a list of problems; an empty list means every required field is
    present, well-typed and internally consistent. Missing evidence is a
    problem, never a default.
    """
    problems: list[str] = []
    if not isinstance(value, dict):
        return ["result is not an object"]
    environment_id = value.get("environment_id")
    if not isinstance(environment_id, str) or not environment_id:
        problems.append("environment_id must be a nonempty string")
    environment = value.get("environment")
    if not isinstance(environment, dict):
        problems.append("environment must be an object")
    else:
        for key in ("system", "machine"):
            if not isinstance(environment.get(key), str) or not environment.get(key):
                problems.append(f"environment.{key} must be a nonempty string")
    if value.get("profile") not in PROFILES:
        problems.append(f"profile must be one of {PROFILES}")
    returncode = value.get("returncode")
    if returncode is not None and not isinstance(returncode, int):
        problems.append("returncode must be an integer or null")
    if isinstance(returncode, bool):
        problems.append("returncode must not be a boolean")
    if not isinstance(value.get("timed_out"), bool):
        problems.append("timed_out must be a boolean")
    for key in _COUNTER_FIELDS:
        if not _is_nonnegative_int(value.get(key)):
            problems.append(f"{key} must be a nonnegative integer")
    for key in _EVENT_LISTS:
        entries = value.get(key)
        if not isinstance(entries, list):
            problems.append(f"{key} must be a list")
            continue
        for index, entry in enumerate(entries):
            if not isinstance(entry, dict):
                problems.append(f"{key}[{index}] must be an object")
                continue
            if not isinstance(entry.get("nodeid"), str) or not entry.get("nodeid"):
                problems.append(f"{key}[{index}].nodeid must be a nonempty string")
    for key, entry_field in (("failures", "signature"), ("errors", "signature"), ("skips", "reason")):
        for index, entry in enumerate(value.get(key) or []):
            if isinstance(entry, dict) and (
                not isinstance(entry.get(entry_field), str) or not entry.get(entry_field)
            ):
                problems.append(f"{key}[{index}].{entry_field} must be a nonempty string")
    if not isinstance(value.get("execution_complete"), bool):
        problems.append("execution_complete must be a boolean")
    if problems:
        return problems
    # Count consistency: every collected test is either executed
    # (passed/failed/errored) or recorded as skipped/xfailed; deselected
    # tests were never collected.
    if (
        value["collected"]
        != value["executed"] + value["skipped"] + value["xfailed"]
    ):
        problems.append(
            "counts are inconsistent: collected must equal "
            "executed + skipped + xfailed"
        )
    if len(value["failures"]) > value["executed"]:
        problems.append("more failures recorded than tests executed")
    if value["skipped"] != len(value["skips"]):
        problems.append("skipped count does not match skip evidence")
    if value["xfailed"] != len(value["xfails"]):
        problems.append("xfailed count does not match xfail evidence")
    if value["errored"] != len(value["errors"]):
        problems.append("errored count does not match error evidence")
    if value["profile"] in ("native", "full"):
        native = value.get("native")
        if not isinstance(native, dict):
            problems.append("native profile results require a native record")
        else:
            if not isinstance(native.get("imported_extension_verified"), bool):
                problems.append(
                    "native.imported_extension_verified must be a boolean"
                )
            if not _is_nonnegative_int(native.get("native_executed")):
                problems.append(
                    "native.native_executed must be a nonnegative integer"
                )
            if not _is_nonnegative_int(native.get("native_test_count")):
                problems.append(
                    "native.native_test_count must be a nonnegative integer"
                )
            if not isinstance(native.get("selected_build"), str) or not native.get(
                "selected_build"
            ):
                problems.append("native.selected_build must be a nonempty string")
            imported = native.get("imported_extension")
            if imported is not None and not isinstance(imported, str):
                problems.append("native.imported_extension must be a string or null")
    return problems


def _hard_failures(current: dict) -> list[str]:
    """Rejection reasons checked before pass/baseline evaluation."""
    reasons: list[str] = []
    if current["timed_out"]:
        reasons.append("the runner exceeded the qualification timeout")
    returncode = current["returncode"]
    if returncode is None:
        reasons.append("the runner produced no exit status")
    elif returncode < 0:
        name = _signal_name(-returncode)
        reasons.append(
            f"the runner was terminated by signal {-returncode}"
            + (f" ({name})" if name else "")
        )
    elif returncode in _PYTEST_TOOL_EXIT_CODES:
        reasons.append(
            f"the runner exited with pytest tool/usage/collection status "
            f"{returncode}"
        )
    if current["collection_errors"]:
        reasons.append(
            f"the run recorded {current['collection_errors']} "
            "collection/import error(s)"
        )
    if current["execution_complete"] is not True:
        reasons.append("the execution is incomplete or its evidence disagrees")
    return reasons


def _pass_blockers(current: dict) -> list[str]:
    """Reasons the result cannot be ``pass`` (empty list means it passes)."""
    blockers: list[str] = []
    if current["returncode"] != 0:
        blockers.append(f"raw exit status is {current['returncode']}, not 0")
    if current["failures"]:
        blockers.append(f"{len(current['failures'])} test failure(s) recorded")
    if current["errored"]:
        blockers.append(f"{current['errored']} errored test(s) recorded")
    if current["executed"] == 0:
        blockers.append("no tests were executed")
    if current["profile"] in ("native", "full"):
        native = current["native"]
        if native["imported_extension_verified"] is not True:
            blockers.append(
                "the selected native artifact was not verifiably imported"
            )
        if native["native_executed"] == 0:
            blockers.append(
                "no direct native oracle tests were executed; tooling-only "
                "test_native_* modules do not qualify"
            )
    return blockers


def _baseline_blockers(current: dict, baseline: object) -> list[str]:
    """Reasons the result cannot be ``baseline_unchanged``.

    Only an independently approved macOS ``full`` baseline recorded on the
    exact same environment qualifies; the subset comparison runs only after
    every required result/environment field validates.
    """
    blockers: list[str] = []
    problems = _validate_result(baseline)
    if problems:
        return [f"baseline evidence invalid: {problem}" for problem in problems]
    if baseline.get("approved") is not True:
        blockers.append("the baseline is not independently approved")
    if baseline["timed_out"] or baseline["execution_complete"] is not True:
        blockers.append("the baseline run itself was not a complete execution")
    if current["environment_id"] != baseline["environment_id"] or (
        current["environment"] != baseline["environment"]
    ):
        blockers.append(
            "the baseline was recorded on a different environment "
            f"(baseline environment_id={baseline['environment_id']!r})"
        )
    if current["environment"]["system"] != _BASELINE_SYSTEM:
        blockers.append(
            "baseline_unchanged is defined only for the approved Darwin "
            f"environment, not {current['environment']['system']!r}; "
            "required gates on this platform are pass-only"
        )
    if current["profile"] != _BASELINE_PROFILE or (
        baseline["profile"] != _BASELINE_PROFILE
    ):
        blockers.append(
            "baseline comparison requires the full profile on both sides "
            f"(current={current['profile']!r}, baseline={baseline['profile']!r})"
        )
    if current["collected"] < baseline["collected"]:
        blockers.append(
            f"the run collected fewer tests ({current['collected']}) than the "
            f"baseline ({baseline['collected']}); a partial result must not "
            "hide unexecuted tests"
        )
    if current["executed"] < baseline["executed"]:
        blockers.append(
            f"the run executed fewer tests ({current['executed']}) than the "
            f"baseline ({baseline['executed']})"
        )
    if current["returncode"] != 1:
        blockers.append(
            f"raw exit status is {current['returncode']}, not the "
            "failure-with-completed-suite exit 1"
        )
    if blockers:
        return blockers
    try:
        current_keys = _failure_keys(current)
        baseline_keys = _failure_keys(baseline)
        current_error_keys = _failure_keys(
            {"failures": list(current["errors"])}
        )
        baseline_error_keys = _failure_keys(
            {"failures": list(baseline["errors"])}
        )
        current_skips = _node_keys(current, "skips")
        baseline_skips = _node_keys(baseline, "skips")
        current_xfails = _node_keys(current, "xfails")
        baseline_xfails = _node_keys(baseline, "xfails")
    except ValueError as error:
        return [f"malformed comparison evidence: {error}"]
    new_failures = sorted(current_keys - baseline_keys)
    if new_failures:
        blockers.append(
            "new failure(s) not in the approved baseline: "
            + ", ".join(f"{nodeid} [{signature}]" for nodeid, signature in new_failures)
        )
    new_errors = sorted(current_error_keys - baseline_error_keys)
    if new_errors:
        blockers.append(
            "new errored test(s) not in the approved baseline: "
            + ", ".join(nodeid for nodeid, _ in new_errors)
        )
    new_skips = sorted(current_skips - baseline_skips)
    if new_skips:
        blockers.append(
            "new skipped test(s) not in the approved baseline: "
            + ", ".join(new_skips)
        )
    new_xfails = sorted(current_xfails - baseline_xfails)
    if new_xfails:
        blockers.append(
            "new xfailed test(s) not in the approved baseline: "
            + ", ".join(new_xfails)
        )
    return blockers


def _signal_name(number: int) -> str | None:
    try:
        return signal.Signals(number).name
    except (ValueError, AttributeError):
        return None


def _verdict(
    status: str, current: dict, reasons: list[str], statement: str
) -> dict:
    verdict = {
        "status": status,
        # baseline_unchanged is explicitly NOT green: it records that the
        # suite still fails exactly the approved historical failures.
        "green": status == _STATUS_PASS,
        "statement": statement,
        "reasons": list(reasons),
        "profile": current.get("profile"),
        "environment_id": current.get("environment_id"),
        "command": current.get("command"),
        "returncode": current.get("returncode"),
        "signal": current.get("signal"),
        "timed_out": current.get("timed_out"),
        "failures": current.get("failures"),
        "counts": {
            key: current.get(key)
            for key in (
                "collected",
                "executed",
                "skipped",
                "xfailed",
                "errored",
                "deselected",
                "collection_errors",
            )
        },
    }
    return verdict


def classify_result(current: dict, baseline: dict | None) -> dict:
    """Classify a stored result against an optional approved baseline.

    Returns the verdict object (status, green flag, statement, reasons and
    the raw command/exit/failure evidence). The classification never
    mutates ``current`` and never approves a baseline by itself.
    """
    problems = _validate_result(current)
    if problems:
        return _verdict(
            _STATUS_FAIL,
            current if isinstance(current, dict) else {},
            [f"missing or malformed result evidence: {p}" for p in problems],
            "FAIL: the result record is missing or malformed; nothing may "
            "be inferred from absent evidence.",
        )
    hard = _hard_failures(current)
    if hard:
        return _verdict(
            _STATUS_FAIL,
            current,
            hard,
            "FAIL: " + "; ".join(hard),
        )
    pass_blockers = _pass_blockers(current)
    if not pass_blockers:
        statement = (
            f"PASS: {current['executed']} test(s) executed on "
            f"{current['environment_id']} with zero failures and zero errors; "
            "all required executed-test evidence is present."
        )
        return _verdict(_STATUS_PASS, current, [], statement)
    if baseline is None:
        return _verdict(
            _STATUS_FAIL,
            current,
            pass_blockers,
            "FAIL: " + "; ".join(pass_blockers),
        )
    blockers = _baseline_blockers(current, baseline)
    if blockers:
        return _verdict(
            _STATUS_FAIL,
            current,
            pass_blockers + blockers,
            "FAIL: " + "; ".join(pass_blockers + blockers),
        )
    removed = sorted(
        _failure_keys(baseline) - _failure_keys(current)
    )
    statement = (
        "NOT GREEN — baseline_unchanged: "
        f"{len(current['failures'])} failure(s) and "
        f"{current['errored']} error(s) exactly match the independently "
        "approved Darwin/full baseline recorded on environment "
        f"{current['environment_id']!r}; the raw exit status is "
        f"{current['returncode']} and the suite did not pass. "
        "No new failure, skip, xfail or error appeared."
    )
    if removed:
        statement += (
            f" {len(removed)} previously failing test(s) are gone — an "
            "improvement, recorded but still not a green suite."
        )
    return _verdict(_STATUS_BASELINE, current, [], statement)


# ---------------------------------------------------------------------------
# JUnit / stdout evidence parsing
# ---------------------------------------------------------------------------

_HEX_ADDRESS = re.compile(r"0x[0-9a-fA-F]+")
# The terminal pytest count line is bare in modern pytest —
# ``37 passed in 0.13s`` or ``3 failed, 1699 passed, 9 skipped,
# 70 deselected, 74 warnings in 836.34s (0:13:56)`` — optionally padded
# with ``=`` banners in older styles. Only a line whose body is entirely
# comma-separated ``N word`` segments (or ``no tests ran``) qualifies.
_SUMMARY_SHAPE = re.compile(
    r"(.*?)\s+in\s+[\d.]+s(?:\s*\(\d+:\d{2}:\d{2}\))?\Z"
)
_SUMMARY_SEGMENT = re.compile(r"\A\d+\s+[A-Za-z]+\Z")
_COLLECTION_ERRORS = re.compile(r"(\d+)\s+errors?\s+during\s+collection")


def _signature_from(message: str | None, text: str | None) -> str:
    """A stable first-failure signature: first nonempty line of the pytest
    message (or body), with hex addresses normalized and whitespace folded."""
    source = (message or "").strip() or (text or "").strip()
    line = next((ln.strip() for ln in source.splitlines() if ln.strip()), "")
    line = _HEX_ADDRESS.sub("0x…", line)
    return " ".join(line.split())[:240]


def _nodeid(classname: str, name: str, root: Path) -> str:
    """Reconstruct the pytest node id from JUnit classname/name attributes.

    ``classname`` is the dotted module path plus any class qualifiers;
    resolve the longest prefix that exists as a file under ``root`` so
    ``a.b.test_x.TestY`` → ``a/b/test_x.py::TestY::test_z``.
    """
    parts = classname.split(".") if classname else []
    for index in range(len(parts), 0, -1):
        candidate = root.joinpath(*parts[:index]).with_suffix(".py")
        if candidate.is_file():
            qualifiers = ".".join(parts[index:])
            nodeid = candidate.relative_to(root).as_posix()
            if qualifiers:
                nodeid += f"::{qualifiers}"
            return f"{nodeid}::{name}"
    if classname:
        return f"{classname.replace('.', '/')}.py::{name}"
    return name


def parse_junit(path: Path, root: Path) -> dict:
    """Parse the pytest JUnit artifact into counts and event evidence."""
    result: dict = {
        "present": False,
        "error": None,
        "collected": 0,
        "executed": 0,
        "skipped": 0,
        "xfailed": 0,
        "errored": 0,
        "failed": 0,
        "passed": 0,
        "native_executed": 0,
        "failures": [],
        "skips": [],
        "xfails": [],
        "errors": [],
    }
    try:
        tree = ElementTree.parse(path)
    except (OSError, ElementTree.ParseError) as error:
        result["error"] = f"cannot parse JUnit artifact {path}: {error}"
        return result
    result["present"] = True
    document = tree.getroot()
    if document.tag == "testsuite":
        suites = [document]
    else:
        suites = list(document.iter("testsuite"))
    for suite in suites:
        for case in suite.iter("testcase"):
            classname = case.get("classname", "")
            name = case.get("name", "")
            nodeid = _nodeid(classname, name, root)
            module_name = Path(nodeid.split("::", 1)[0]).name
            result["collected"] += 1
            failure = case.find("failure")
            error = case.find("error")
            skipped = case.find("skipped")
            if skipped is not None:
                if skipped.get("type") == "pytest.xfail":
                    result["xfailed"] += 1
                    result["xfails"].append(
                        {"nodeid": nodeid, "reason": skipped.get("message", "")}
                    )
                else:
                    result["skipped"] += 1
                    result["skips"].append(
                        {
                            "nodeid": nodeid,
                            "reason": _signature_from(
                                skipped.get("message"), skipped.text
                            )
                            or "(no reason recorded)",
                        }
                    )
                continue
            result["executed"] += 1
            if (
                module_name.startswith("test_native_")
                and module_name not in _NATIVE_TOOL_MODULES
            ):
                result["native_executed"] += 1
            if failure is not None:
                result["failed"] += 1
                result["failures"].append(
                    {
                        "nodeid": nodeid,
                        "signature": _signature_from(
                            failure.get("message"), failure.text
                        ),
                    }
                )
            elif error is not None:
                result["errored"] += 1
                result["errors"].append(
                    {
                        "nodeid": nodeid,
                        "signature": _signature_from(
                            error.get("message"), error.text
                        ),
                    }
                )
            else:
                result["passed"] += 1
    return result


def parse_pytest_summary(stdout: str) -> dict | None:
    """Parse pytest's terminal count line into counter fields.

    Returns None when no summary-shaped line exists. Only the last
    matching line wins, so progress output cannot masquerade as the
    final accounting.
    """
    body: str | None = None
    no_tests = False
    for line in stdout.splitlines():
        stripped = line.strip().strip("=").strip()
        match = _SUMMARY_SHAPE.fullmatch(stripped)
        if match is None:
            continue
        candidate = match.group(1).strip()
        if candidate.startswith("no tests ran"):
            body, no_tests = candidate, True
            continue
        parts = [part.strip() for part in candidate.split(",")]
        if parts and all(
            _SUMMARY_SEGMENT.fullmatch(part) for part in parts
        ):
            body, no_tests = candidate, False
    if body is None:
        return None
    counts: dict[str, int] = {}
    if not no_tests:
        key_map = {
            "passed": "passed",
            "failed": "failed",
            "failure": "failed",
            "failures": "failed",
            "skipped": "skipped",
            "deselected": "deselected",
            "xfailed": "xfailed",
            "xpassed": "xpassed",
            "error": "errors",
            "errors": "errors",
            "warning": "warnings",
            "warnings": "warnings",
        }
        for part in body.split(","):
            amount, word = part.strip().split(maxsplit=1)
            key = key_map.get(word)
            if key is not None:
                counts[key] = counts.get(key, 0) + int(amount)
    collection_match = _COLLECTION_ERRORS.search(stdout)
    counts["collection_marked"] = (
        int(collection_match.group(1)) if collection_match else 0
    )
    return counts


def _cross_check(
    junit: dict, summary: dict | None, returncode: int | None, timed_out: bool
) -> tuple[int, bool, list[str]]:
    """(collection_errors, execution_complete, problems).

    execution_complete requires both artifacts to exist and agree, an
    ordinary pytest exit status, and no timeout.
    """
    problems: list[str] = []
    collection_errors = 0
    if junit["error"] is not None:
        problems.append(junit["error"])
    if not junit["present"]:
        problems.append("the pytest JUnit artifact is missing")
    if summary is None:
        problems.append("the pytest summary line is missing from stdout")
    if junit["present"] and summary is not None:
        pairs = (
            ("failed", "failed"),
            ("passed", "passed"),
            ("skipped", "skipped"),
            ("xfailed", "xfailed"),
        )
        for junit_key, summary_key in pairs:
            if junit[junit_key] != summary.get(summary_key, 0):
                problems.append(
                    f"JUnit {junit_key}={junit[junit_key]} disagrees with "
                    f"pytest summary {summary_key}={summary.get(summary_key, 0)}"
                )
        summary_errors = summary.get("errors", 0)
        if summary_errors < junit["errored"]:
            problems.append(
                "the pytest summary reports fewer errors than the JUnit "
                "artifact"
            )
        collection_errors = max(
            summary_errors - junit["errored"], summary["collection_marked"]
        )
    if returncode not in (0, 1):
        problems.append(f"the pytest exit status is {returncode}, not 0 or 1")
    if timed_out:
        problems.append("the run was killed by the qualification timeout")
    complete = not problems and collection_errors == 0
    return collection_errors, complete, problems


# ---------------------------------------------------------------------------
# Environment, identity and provenance evidence
# ---------------------------------------------------------------------------


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_build_context(build: Path) -> dict | None:
    """Best-effort read of the build's schema-2 check context.

    Mirrors tools/native_checks.py::load_context's file selection (flat
    file or single per-config candidate) but is read-only evidence, not the
    validated check path — the profile's own preflight owns enforcement.
    """
    context_path = build / "native_check_context.json"
    if not context_path.is_file():
        directory = build / "native_check_context"
        candidates = (
            sorted(directory.glob("*.json")) if directory.is_dir() else []
        )
        if len(candidates) != 1:
            return None
        context_path = candidates[0]
    try:
        context = json.loads(context_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return None
    return context if isinstance(context, dict) else None


def _git_evidence(root: Path) -> dict:
    def probe(*argv: str) -> str | None:
        try:
            completed = subprocess.run(
                ["git", "-C", str(root), *argv],
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return None
        if completed.returncode != 0:
            return None
        return completed.stdout

    head = probe("rev-parse", "HEAD")
    status = probe("status", "--porcelain")
    return {
        "commit": head.strip() if head is not None else None,
        "dirty": (
            any(line.strip() for line in status.splitlines())
            if status is not None
            else None
        ),
    }


def _collect_environment(context: dict | None) -> tuple[dict, str]:
    compiler = context.get("compiler") if isinstance(context, dict) else None
    compiler_id = compiler.get("id") if isinstance(compiler, dict) else None
    compiler_version = (
        compiler.get("version") if isinstance(compiler, dict) else None
    )
    environment = {
        "system": platform.system(),
        "machine": platform.machine(),
        "release": platform.release(),
        "version": platform.version(),
        "platform": platform.platform(),
        "python_version": platform.python_version(),
        "python_implementation": platform.python_implementation(),
        "compiler_id": compiler_id,
        "compiler_version": compiler_version,
    }
    environment_id = "/".join(
        [
            environment["system"],
            environment["machine"],
            f"python-{environment['python_version']}",
            f"compiler-{compiler_id or 'unrecorded'}@{compiler_version or 'unrecorded'}",
        ]
    )
    return environment, environment_id


def _collect_identity(root: Path, context: dict | None) -> dict:
    identity: dict = {
        "cpu": platform.machine(),
        "source": _git_evidence(root),
        "lock": {"path": "uv.lock", "sha256": None, "present": False},
        "compiler": None,
        "provider": None,
    }
    lock_path = root / "uv.lock"
    if lock_path.is_file():
        identity["lock"] = {
            "path": "uv.lock",
            "sha256": _sha256(lock_path),
            "present": True,
        }
    if isinstance(context, dict):
        compiler = context.get("compiler")
        if isinstance(compiler, dict):
            identity["compiler"] = {
                "id": compiler.get("id"),
                "version": compiler.get("version"),
                "path": compiler.get("path"),
            }
        numerics = context.get("numerics")
        if isinstance(numerics, dict):
            libraries = []
            for entry in numerics.get("library_paths") or []:
                if isinstance(entry, dict):
                    libraries.append(
                        {
                            "role": entry.get("role"),
                            "path": entry.get("path"),
                            "identity_kind": entry.get("identity_kind"),
                            "build_id": entry.get("build_id"),
                            "sha256": entry.get("sha256"),
                        }
                    )
            identity["provider"] = {
                "backend": numerics.get("backend"),
                "numpy_version": numerics.get("numpy_version"),
                "blas_integer_bits": numerics.get("blas_integer_bits"),
                "lapack_integer_bits": numerics.get("lapack_integer_bits"),
                "libraries": libraries,
            }
    return identity


def _verify_native_provenance(
    build: Path, provenance_path: Path, run_started: float
) -> dict:
    """Verify the reporter/loader provenance for the selected artifact.

    The record must be freshly written during this run, name the selected
    build and an existing bike_native extension inside it, carry the
    platform identity of this host, and record a nonzero native count.
    """
    record: dict = {
        "provenance_path": str(provenance_path),
        "imported_extension_verified": False,
        "selected_build": str(build),
        "imported_extension": None,
        "native_test_count": 0,
        "problems": [],
    }
    problems: list[str] = record["problems"]
    try:
        stat = provenance_path.stat()
        document = json.loads(provenance_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        problems.append(f"cannot read native provenance: {error}")
        return record
    if not isinstance(document, dict):
        problems.append("native provenance is not an object")
        return record
    if document.get("schema_version") != 1:
        problems.append(
            f"native provenance schema_version is "
            f"{document.get('schema_version')!r}, not 1"
        )
    resolved_build = build.resolve()
    if document.get("selected_build") != str(resolved_build):
        problems.append(
            f"native provenance selected_build "
            f"{document.get('selected_build')!r} does not match the "
            f"selected build {resolved_build}"
        )
    imported = document.get("imported_extension")
    if not isinstance(imported, str) or not imported:
        problems.append("native provenance records no imported_extension")
    else:
        extension = Path(imported)
        resolved_extension = (
            extension.resolve() if extension.is_absolute() else None
        )
        if resolved_extension is None:
            problems.append(
                f"imported_extension is not an absolute path: {imported!r}"
            )
        else:
            if not resolved_extension.is_file():
                problems.append(
                    f"imported_extension does not exist: {resolved_extension}"
                )
            if not resolved_extension.is_relative_to(resolved_build):
                problems.append(
                    f"imported_extension {resolved_extension} is outside "
                    f"the selected build {resolved_build}"
                )
            if not resolved_extension.name.startswith("bike_native"):
                problems.append(
                    f"imported_extension is not a bike_native artifact: "
                    f"{resolved_extension.name}"
                )
            record["imported_extension"] = str(resolved_extension)
    recorded_platform = document.get("platform")
    if not isinstance(recorded_platform, dict):
        problems.append("native provenance records no platform identity")
    else:
        if recorded_platform.get("system") != platform.system():
            problems.append(
                f"native provenance platform.system "
                f"{recorded_platform.get('system')!r} does not match this "
                f"host {platform.system()!r}"
            )
        if recorded_platform.get("machine") != platform.machine():
            problems.append(
                f"native provenance platform.machine "
                f"{recorded_platform.get('machine')!r} does not match this "
                f"host {platform.machine()!r}"
            )
    native_test_count = document.get("native_test_count")
    if not _is_nonnegative_int(native_test_count) or native_test_count == 0:
        problems.append(
            "native provenance records no positive native_test_count"
        )
    else:
        record["native_test_count"] = native_test_count
    # A provenance file left over from an earlier run must not certify this
    # one: the reporter rewrites it at collection finish.
    if stat.st_mtime < run_started - 1.0:
        problems.append(
            "native provenance predates this run and is stale evidence"
        )
    record["imported_extension_verified"] = not problems
    return record


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------


def run_profile(
    *,
    profile: str,
    build: Path,
    output: Path,
    baseline: dict | None = None,
    timeout: float | None = None,
    pytest_args: tuple[str, ...] = (),
    environ: dict[str, str] | None = None,
    root: Path = _REPO_ROOT,
) -> dict:
    """Execute the real runner and store the qualified result document."""
    root = Path(root).resolve()
    build = Path(build).resolve()
    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    junit_path = output.parent / f"{output.stem}.junit.xml"
    stdout_path = output.parent / f"{output.stem}.stdout.log"
    stderr_path = output.parent / f"{output.stem}.stderr.log"

    environment = dict(os.environ if environ is None else environ)
    if profile in ("native", "full"):
        environment["NATIVE_TEST_BUILD_PATH"] = str(build)
    provenance_path = Path(
        environment.get(
            "NATIVE_TEST_PROVENANCE_PATH",
            str(build / "native_test_provenance.json"),
        )
    )

    command = [
        "bash",
        "tools/run_tests.sh",
        profile,
        f"--junitxml={junit_path}",
        *pytest_args,
    ]
    def _as_text(value: object) -> str:
        if isinstance(value, bytes):
            return value.decode("utf-8", "replace")
        return value if isinstance(value, str) else ""

    run_started = time.time()
    timed_out = False
    returncode: int | None
    stdout_text = ""
    stderr_text = ""
    try:
        completed = subprocess.run(
            command,
            cwd=root,
            env=environment,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        returncode = completed.returncode
        stdout_text = _as_text(completed.stdout)
        stderr_text = _as_text(completed.stderr)
    except subprocess.TimeoutExpired as error:
        timed_out = True
        returncode = None
        stdout_text = _as_text(error.stdout)
        stderr_text = _as_text(error.stderr)
    except OSError as error:
        returncode = None
        stderr_text = f"runner could not be executed: {error}"
    stdout_path.write_text(stdout_text, encoding="utf-8")
    stderr_path.write_text(stderr_text, encoding="utf-8")

    junit = parse_junit(junit_path, root)
    summary = parse_pytest_summary(stdout_text)
    collection_errors, complete, problems = _cross_check(
        junit, summary, returncode, timed_out
    )

    context = _read_build_context(build)
    env_dict, environment_id = _collect_environment(context)
    identity = _collect_identity(root, context)

    result: dict = {
        "schema_version": SCHEMA_VERSION,
        "profile": profile,
        "environment_id": environment_id,
        "environment": env_dict,
        "command": command,
        "returncode": returncode,
        "signal": (
            _signal_name(-returncode)
            if isinstance(returncode, int) and returncode < 0
            else None
        ),
        "timed_out": timed_out,
        "collected": junit["collected"],
        "executed": junit["executed"],
        "skipped": junit["skipped"],
        "xfailed": junit["xfailed"],
        "errored": junit["errored"],
        "deselected": (summary or {}).get("deselected", 0),
        "collection_errors": collection_errors,
        "failures": junit["failures"],
        "skips": junit["skips"],
        "xfails": junit["xfails"],
        "errors": junit["errors"],
        "execution_complete": complete,
        "evidence_problems": problems,
        "build": str(build),
        "identity": identity,
        "artifacts": {
            "junit": str(junit_path),
            "stdout": str(stdout_path),
            "stderr": str(stderr_path),
        },
        "stdout_tail": "\n".join(stdout_text.splitlines()[-64:]),
        "stderr_tail": "\n".join(stderr_text.splitlines()[-64:]),
        # Never self-approved: independent approval is recorded by a human
        # reviewing this file, never by this tool.
        "approved": False,
    }

    if profile in ("native", "full"):
        native = _verify_native_provenance(build, provenance_path, run_started)
        native["native_executed"] = junit["native_executed"]
        result["native"] = native

    verdict = classify_result(result, baseline)
    result.update(verdict)
    _write_result(output, result)
    return result


def _write_result(output: Path, result: dict) -> None:
    """Atomically write the result JSON (temp file + rename)."""
    temporary = output.with_name(f"{output.name}.tmp")
    temporary.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    os.replace(temporary, output)


def _load_result_document(path: Path) -> dict:
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"result document is not an object: {path}")
    return document


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run a test profile through tools/run_tests.sh and classify the "
            "result against an optional independently approved baseline."
        )
    )
    parser.add_argument("--profile", required=True, choices=PROFILES)
    parser.add_argument(
        "--build",
        required=True,
        type=Path,
        help="selected native build directory (required for every profile; "
        "native/full export it as NATIVE_TEST_BUILD_PATH)",
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="write the schema-1 result JSON here",
    )
    parser.add_argument(
        "--baseline",
        type=Path,
        default=None,
        help="independently approved result JSON used only for the exact "
        "Darwin/full baseline comparison",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=None,
        help="kill the runner after this many seconds (default: no timeout)",
    )
    parser.add_argument(
        "pytest_args",
        nargs="*",
        help="extra pytest arguments forwarded through tools/run_tests.sh",
    )
    args = parser.parse_args(argv)

    baseline: dict | None = None
    baseline_error: str | None = None
    if args.baseline is not None:
        try:
            baseline = _load_result_document(args.baseline)
        except (OSError, UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
            baseline_error = f"cannot load baseline {args.baseline}: {error}"

    result = run_profile(
        profile=args.profile,
        build=args.build,
        output=args.output,
        baseline=baseline,
        timeout=args.timeout,
        pytest_args=tuple(args.pytest_args),
    )
    if baseline_error is not None:
        # Reclassify without a baseline when the requested comparison
        # evidence cannot be read — a clean pass still stands.
        verdict = classify_result(result, None)
        verdict["reasons"] = [baseline_error, *verdict["reasons"]]
        result.update(verdict)
        result["baseline_error"] = baseline_error
        _write_result(Path(args.output).resolve(), result)
    print(result["statement"])
    return 0 if result["status"] in (_STATUS_PASS, _STATUS_BASELINE) else 1


if __name__ == "__main__":
    raise SystemExit(main())
