"""Behaviour tests for devin-brief.py: fill a template, refuse incomplete fills."""

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SKILL = Path(__file__).resolve().parents[1]
SCRIPT = SKILL / "scripts" / "devin-brief.py"


class BriefTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.template = self.tmp / "t.md"
        self.template.write_text("Goal: {{GOAL}}\nDir: {{WORKDIR}}\nGoal again: {{GOAL}}\n")
        self.out = self.tmp / "run" / "brief.md"

    def tearDown(self):
        self._tmp.cleanup()

    def fill(self, template, *pairs):
        return subprocess.run([sys.executable, "-I", str(SCRIPT), str(template), str(self.out), *pairs],
                              capture_output=True, text=True)

    def test_fills_every_occurrence_and_prints_only_a_status_line(self):
        """All placeholders are replaced; stdout names the file, never its content."""
        result = self.fill(self.template, "GOAL=ship it", "WORKDIR=/w")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.out.read_text(), "Goal: ship it\nDir: /w\nGoal again: ship it\n")
        self.assertEqual(result.stdout.strip(), f"wrote {self.out} (3 lines)")

    def test_value_from_file(self):
        """KEY=@path takes the value from a file, without its trailing newline."""
        source = self.tmp / "goal.txt"
        source.write_text("line one\nline two\n")
        result = self.fill(self.template, f"GOAL=@{source}", "WORKDIR=/w")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Goal: line one\nline two\nDir: /w", self.out.read_text())

    def test_missing_placeholder_value_refuses_and_writes_nothing(self):
        """A placeholder without a value fails with exit 2 and no output file."""
        result = self.fill(self.template, "GOAL=x")
        self.assertEqual(result.returncode, 2)
        self.assertIn("missing ['WORKDIR']", result.stderr)
        self.assertFalse(self.out.exists())

    def test_unknown_key_refuses(self):
        """A key with no placeholder (a typo) fails with exit 2."""
        result = self.fill(self.template, "GOAL=x", "WORKDIR=/w", "GAOL=y")
        self.assertEqual(result.returncode, 2)
        self.assertIn("unknown ['GAOL']", result.stderr)

    def test_shipped_templates_fill_cleanly(self):
        """Each template in templates/ fills with exactly its own placeholder set."""
        import re
        for template in sorted((SKILL / "templates").glob("*.md")):
            keys = sorted(set(re.findall(r"\{\{([A-Z_]+)\}\}", template.read_text())))
            result = self.fill(template, *(f"{k}=v" for k in keys))
            self.assertEqual(result.returncode, 0, f"{template.name}: {result.stderr}")
            self.assertNotIn("{{", self.out.read_text(), template.name)


if __name__ == "__main__":
    unittest.main()
