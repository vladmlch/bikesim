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
