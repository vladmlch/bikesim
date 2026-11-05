# Devin Scripts Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix two defects in the Devin orchestration helper scripts and cover both scripts with behaviour tests.

**Architecture:** The scripts in `.claude/skills/devin-orchestration/scripts/` are standalone Python 3 programs. Tests run each script as a subprocess against a temporary run directory built by the test, so they check the printed output and exit code (the scripts' contract) rather than internals.

**Tech Stack:** Python 3 standard library only (`unittest`, `subprocess`, `tempfile`, `json`).

**Spec:** `.claude/skills/devin-orchestration/SKILL.md` (run-directory layout, watchdog and digest behaviour).

## Global Constraints

- Standard library only: no pytest, no third-party packages (the repository is Python-free; these are tool scripts).
- Run every Python command with `python3 -I`.
- Do not modify `.claude/skills/devin-orchestration/scripts/devin-run.sh`: it is the wrapper currently running the executor, and bash reads a script while executing it.
- Test command for the whole suite: `python3 -I -m unittest discover -s .claude/skills/devin-orchestration/tests -v`
- Stage only the files a task lists (`git add <paths>`), never `git add -A` or `git add .`: the checkout has many unrelated untracked and modified files (including `AGENTS.md`).
- Commit messages follow the repository style `type(scope): summary`.

## Run directory layout (from the spec)

For attempt `N`: `stdout.N.txt`, `stderr.N.txt`, `export.N.json`, `exit_code.N`. Shared: `attempt` (latest N as text), `session_id`, `pid`.
`export.N.json` is ATIF: `{"session_id": str, "steps": [...], "final_metrics": {...}}`. An agent step looks like:

```json
{"source": "agent", "message": "",
 "tool_calls": [{"tool_call_id": "exec:1#a", "function_name": "exec", "arguments": {"command": "ctest -R foo"}}],
 "observation": {"results": [{"source_call_id": "exec:1#a", "content": "Output from command in shell 1:\nTest #1 foo ... Failed\n\n\nExit code: 8"}]}}
```

---

## Preparation (orchestrator, before Task 1)

- [ ] Create branch `devin-scripts-hardening` from `main` in the main checkout (uncommitted `AGENTS.md` stays uncommitted).
- [ ] Commit the current skill as the baseline: `.claude/skills/devin-orchestration/` and `.claude/settings.json`, plus this plan.

---

### Task 1: devin-summary tolerates blank stderr; summary tests

**Files:**
- Modify: `.claude/skills/devin-orchestration/scripts/devin-summary.py` (the `stderr` block in `main()`)
- Create: `.claude/skills/devin-orchestration/tests/test_devin_summary.py`
- Modify: `.claude/skills/devin-orchestration/SKILL.md` (one line in `## Gotchas`)

**Interfaces:**
- Consumes: CLI `python3 -I devin-summary.py <run-dir> [--attempt N]`; exit 0 when the export exists, 1 when it does not.
- Produces: `tests/` directory discovered by the suite command; Task 2 adds its own file there.

**Defect:** a `stderr.N.txt` that is non-empty but holds only whitespace makes `.strip().splitlines()[-1]` raise `IndexError`, so the digest crashes instead of reporting.

- [ ] **Step 1: Write the failing tests**

Create `.claude/skills/devin-orchestration/tests/test_devin_summary.py`:

```python
"""Behaviour tests for devin-summary.py: run it on a synthetic run directory."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "devin-summary.py"


def exec_step(call_id, command, output):
    return {
        "source": "agent", "message": "",
        "tool_calls": [{"tool_call_id": call_id, "function_name": "exec",
                        "arguments": {"command": command}}],
        "observation": {"results": [{"source_call_id": call_id, "content": output}]},
    }


def tool_step(call_id, name, file_path):
    return {
        "source": "agent", "message": "",
        "tool_calls": [{"tool_call_id": call_id, "function_name": name,
                        "arguments": {"file_path": file_path}}],
    }


class SummaryTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.run_dir = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def write_run(self, steps, stderr="", with_export=True):
        (self.run_dir / "attempt").write_text("1\n")
        (self.run_dir / "exit_code.1").write_text("0\n")
        (self.run_dir / "stderr.1.txt").write_text(stderr)
        if with_export:
            export = {"session_id": "test-session", "steps": steps,
                      "final_metrics": {"total_prompt_tokens": 10, "total_completion_tokens": 2}}
            (self.run_dir / "export.1.json").write_text(json.dumps(export))

    def summarize(self):
        return subprocess.run([sys.executable, "-I", str(SCRIPT), str(self.run_dir)],
                              capture_output=True, text=True)

    def test_failed_exec_is_reported_with_command_and_last_output_line(self):
        """A non-zero exec shows its exit code, command and last output line."""
        self.write_run([exec_step("exec:1#a", "ctest -R foo",
                                  "Output from command in shell 1:\nTest #1 foo ... Failed\n\n\nExit code: 8")])
        result = self.summarize()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("non-zero exec (1):", result.stdout)
        self.assertIn("exit=8 $ ctest -R foo", result.stdout)
        self.assertIn("-> Test #1 foo ... Failed", result.stdout)

    def test_successful_exec_is_not_listed(self):
        """An exec that exited 0 does not appear among failures."""
        self.write_run([exec_step("exec:1#a", "ls", "Output from command in shell 1:\na\n\n\nExit code: 0")])
        result = self.summarize()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("non-zero exec (0):", result.stdout)

    def test_write_tools_list_touched_files_and_read_tools_do_not(self):
        """Only file paths passed to non-read tools count as touched."""
        self.write_run([tool_step("edit:1#a", "edit", "/w/changed.cpp"),
                        tool_step("read:2#b", "read", "/w/only_read.cpp")])
        result = self.summarize()
        self.assertIn("files touched (1):", result.stdout)
        self.assertIn("edit: /w/changed.cpp", result.stdout)
        self.assertNotIn("only_read.cpp", result.stdout)

    def test_whitespace_only_stderr_does_not_crash(self):
        """A stderr file holding only blank lines is treated as empty."""
        self.write_run([], stderr="\n   \n\n")
        result = self.summarize()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("stderr (", result.stdout)

    def test_stderr_last_non_blank_line_is_shown(self):
        """Trailing blank lines do not hide the last real stderr line."""
        self.write_run([], stderr="warning: one\nerror: two\n\n  \n")
        result = self.summarize()
        self.assertIn("last line: error: two", result.stdout)

    def test_missing_export_exits_nonzero(self):
        """Without an export the digest says so and exits 1."""
        self.write_run([], with_export=False)
        result = self.summarize()
        self.assertEqual(result.returncode, 1)
        self.assertIn("no export", result.stdout)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the tests to verify the defect**

Run: `python3 -I -m unittest discover -s .claude/skills/devin-orchestration/tests -v`
Expected: `test_whitespace_only_stderr_does_not_crash` FAILS (returncode 1, `IndexError` in stderr); the other 5 tests PASS (`test_stderr_last_non_blank_line_is_shown` guards the fix against regressing the current behaviour).

- [ ] **Step 3: Fix the stderr block**

In `devin-summary.py`, replace:

```python
    stderr = run_dir / f"stderr.{n}.txt"
    if stderr.exists() and stderr.stat().st_size:
        print(f"stderr ({stderr.stat().st_size} B), last line: "
              f"{short(stderr.read_text(errors='replace').strip().splitlines()[-1], 200)}")
```

with:

```python
    stderr = run_dir / f"stderr.{n}.txt"
    # Blank lines carry nothing; a file of only whitespace counts as empty.
    stderr_lines = ([line for line in stderr.read_text(errors="replace").splitlines() if line.strip()]
                    if stderr.exists() else [])
    if stderr_lines:
        print(f"stderr ({stderr.stat().st_size} B), last line: {short(stderr_lines[-1], 200)}")
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python3 -I -m unittest discover -s .claude/skills/devin-orchestration/tests -v`
Expected: all 6 tests PASS.

- [ ] **Step 5: Document the test command**

Append to the `## Gotchas` list in `.claude/skills/devin-orchestration/SKILL.md`:

```markdown
- After changing a script, run its tests: `python3 -I -m unittest discover -s .claude/skills/devin-orchestration/tests -v`.
```

- [ ] **Step 6: Commit**

```bash
git add .claude/skills/devin-orchestration/scripts/devin-summary.py \
        .claude/skills/devin-orchestration/tests/test_devin_summary.py \
        .claude/skills/devin-orchestration/SKILL.md
git commit -m "fix(devin-skill): tolerate blank stderr in the run digest"
```

---

### Task 2: devin-watch waits out the launch race; watchdog tests

**Files:**
- Modify: `.claude/skills/devin-orchestration/scripts/devin-watch.py` (`main()`: argument parser and the "process gone" branch)
- Create: `.claude/skills/devin-orchestration/tests/test_devin_watch.py`

**Interfaces:**
- Consumes: CLI `python3 -I devin-watch.py <run-dir> [--stall-min M] [--poll S]`; exit 0 with `devin attempt N finished exit=X`; exit 1 with `... STALL ...` or `... gone without exit_code ...`.
- Produces: new option `--grace SECONDS` (float, default 60).

**Defect:** `devin-run.sh` writes `attempt` before it launches Devin and writes `pid`; on a resumed attempt `pid` still holds the previous attempt's dead process for that moment. A watchdog started alongside the wrapper can read the new `attempt` with a missing or stale `pid` and report "gone" although the run is healthy. The fix: a missing or dead pid must persist for `--grace` seconds (re-checking `exit_code.N` each poll) before the watchdog reports it.

- [ ] **Step 1: Write the failing tests**

Create `.claude/skills/devin-orchestration/tests/test_devin_watch.py`:

```python
"""Behaviour tests for devin-watch.py on synthetic run directories.

No test needs a Devin session: without a session_id the watchdog never
queries sessions.db.
"""

import os
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "devin-watch.py"


def dead_pid():
    proc = subprocess.Popen(["true"])
    proc.wait()
    return proc.pid


class WatchTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.run_dir = Path(self._tmp.name)
        (self.run_dir / "attempt").write_text("1\n")

    def tearDown(self):
        self._tmp.cleanup()

    def watch(self, *args, timeout=20):
        return subprocess.run([sys.executable, "-I", str(SCRIPT), str(self.run_dir), *args],
                              capture_output=True, text=True, timeout=timeout)

    def test_finished_attempt_reports_exit_code(self):
        """An attempt with exit_code.N is reported as finished, exit 0."""
        (self.run_dir / "exit_code.1").write_text("3\n")
        result = self.watch("--poll", "0.1")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("devin attempt 1 finished exit=3", result.stdout)

    def test_missing_pid_during_launch_is_not_reported_as_gone(self):
        """attempt written but pid not yet: the watchdog waits for the wrapper."""
        def finish_later():
            time.sleep(0.5)
            (self.run_dir / "exit_code.1").write_text("0\n")
        threading.Thread(target=finish_later).start()
        result = self.watch("--poll", "0.1", "--grace", "5")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("finished exit=0", result.stdout)

    def test_dead_pid_is_reported_after_grace(self):
        """A pid that stays dead past --grace with no exit_code is reported, exit 1."""
        (self.run_dir / "pid").write_text(f"{dead_pid()}\n")
        started = time.monotonic()
        result = self.watch("--poll", "0.1", "--grace", "0.5")
        self.assertEqual(result.returncode, 1)
        self.assertIn("gone without exit_code", result.stdout)
        self.assertGreaterEqual(time.monotonic() - started, 0.5)

    def test_stall_without_session_reports_no_last_message(self):
        """A live pid with no session and no progress stalls immediately at --stall-min 0."""
        (self.run_dir / "pid").write_text(f"{os.getpid()}\n")
        result = self.watch("--poll", "0.1", "--stall-min", "0")
        self.assertEqual(result.returncode, 1)
        self.assertIn("STALL", result.stdout)
        self.assertIn("last message: none", result.stdout)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the tests to verify the defect**

Run: `python3 -I -m unittest discover -s .claude/skills/devin-orchestration/tests -v`
Expected: `test_missing_pid_during_launch_is_not_reported_as_gone` and `test_dead_pid_is_reported_after_grace` FAIL (`--grace` is an unknown argument, exit 2); the other watch tests and all Task 1 tests PASS.

- [ ] **Step 3: Add the grace period**

In `devin-watch.py` `main()`, add after the `--poll` argument:

```python
    ap.add_argument("--grace", type=float, default=60,
                    help="seconds a missing or dead pid may persist before the run counts as gone")
```

Replace `last_id, last_change = None, time.time()` with:

```python
    last_id, last_change = None, time.time()
    # devin-run.sh writes `attempt` before `pid`, and a resumed attempt briefly
    # sees the previous attempt's dead pid: only a pid that stays missing or
    # dead for the whole grace period means the wrapper is gone.
    gone_since = None
```

Replace:

```python
        pid = read(run_dir / "pid")
        if attempt and not alive(pid):
            print(f"devin attempt {attempt}: process {pid} gone without exit_code (wrapper killed?)")
            return 1
```

with:

```python
        pid = read(run_dir / "pid")
        if attempt and not alive(pid):
            gone_since = gone_since or time.time()
            if time.time() - gone_since >= args.grace:
                print(f"devin attempt {attempt}: process {pid} gone without exit_code (wrapper killed?)")
                return 1
        else:
            gone_since = None
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python3 -I -m unittest discover -s .claude/skills/devin-orchestration/tests -v`
Expected: all 10 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add .claude/skills/devin-orchestration/scripts/devin-watch.py \
        .claude/skills/devin-orchestration/tests/test_devin_watch.py
git commit -m "fix(devin-skill): wait out the launch race before reporting a gone run"
```

---

## Completion proof (orchestrator)

The diff touches nothing the static-analysis sweeps read (no C++, CMake, `native/src/tools/`, `tools/run_*.sh`, `tools/check_change.sh`; the `scripts` sweep reads only `tools/*.sh`), so per `AGENTS.md` the proof is level 1 against `main` plus this suite:

- `python3 -I -m unittest discover -s .claude/skills/devin-orchestration/tests -v` — all pass.
- `bash tools/check_change.sh main` — exits 0.
