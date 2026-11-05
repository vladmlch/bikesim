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
