"""Behaviour tests for devin-run.sh: attempt 1 must survive `set -u` on
stock macOS /bin/bash 3.2, where an empty array's `"${resume[@]}"` is a
fatal unbound variable. The wrapper runs against a stub `devin` binary.
"""

import os
import stat
import subprocess
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "devin-run.sh"

STUB = """#!/bin/sh
echo "stub devin ran"
export_path=
prev=
for arg in "$@"; do
  if [ "$prev" = "--export" ]; then export_path=$arg; fi
  prev=$arg
done
printf '%s\\n' '{"session_id": "stub-session", "steps": []}' > "$export_path"
exit 0
"""


@unittest.skipUnless(Path("/bin/bash").exists(), "/bin/bash not present")
class RunTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        tmp = Path(self._tmp.name)
        self.run_dir = tmp / "run"
        self.workdir = tmp / "work"
        self.home = tmp / "home"
        self.stub_dir = tmp / "stub"
        for d in (self.run_dir, self.workdir, self.home, self.stub_dir):
            d.mkdir()
        (self.run_dir / "brief.md").write_text("# Task: stub\n")
        stub = self.stub_dir / "devin"
        stub.write_text(STUB)
        stub.chmod(stub.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    def tearDown(self):
        self._tmp.cleanup()

    def wrapper(self):
        env = dict(os.environ)
        env["HOME"] = str(self.home)
        env["PATH"] = str(self.stub_dir) + os.pathsep + env["PATH"]
        return subprocess.run(["/bin/bash", str(SCRIPT), str(self.run_dir),
                               "--workdir", str(self.workdir)],
                              capture_output=True, text=True, env=env, timeout=60)

    def test_attempt_one_with_empty_resume_array_completes(self):
        """Attempt 1 leaves resume=() empty; under bash 3.2 `set -u` must not abort."""
        result = self.wrapper()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual((self.run_dir / "exit_code.1").read_text().strip(), "0")
        self.assertIn("stub-session", (self.run_dir / "session_id").read_text())
        self.assertIn("stub devin ran", (self.run_dir / "stdout.1.txt").read_text())
        self.assertTrue(result.stdout.startswith(
            "devin attempt 1 exit=0 session=stub-session"), result.stdout)


if __name__ == "__main__":
    unittest.main()
