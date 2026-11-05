#!/usr/bin/env python3
"""Fill a brief template so the orchestrator writes only the task's delta.

Usage: python3 -I devin-brief.py <template> <out-file> KEY=VALUE ... [KEY=@file ...]

Every {{KEY}} in the template must get a value and every value must match a
placeholder, so a typo or a forgotten field fails here instead of reaching
Devin. `KEY=@path` reads the value from a file. Prints one line: the output
path and its line count; the filled brief is never echoed.
"""

import re
import sys
from pathlib import Path

PLACEHOLDER = re.compile(r"\{\{([A-Z_]+)\}\}")


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print(__doc__.strip().splitlines()[2], file=sys.stderr)
        return 2
    template, out, pairs = Path(argv[0]), Path(argv[1]), argv[2:]
    text = template.read_text()

    values = {}
    for pair in pairs:
        key, sep, value = pair.partition("=")
        if not sep:
            print(f"devin-brief: not KEY=VALUE: {pair}", file=sys.stderr)
            return 2
        values[key] = Path(value[1:]).read_text().rstrip("\n") if value.startswith("@") else value

    wanted = set(PLACEHOLDER.findall(text))
    missing, unknown = sorted(wanted - values.keys()), sorted(values.keys() - wanted)
    if missing or unknown:
        print(f"devin-brief: missing {missing or '-'}; unknown {unknown or '-'}", file=sys.stderr)
        return 2

    filled = PLACEHOLDER.sub(lambda m: values[m.group(1)], text)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(filled)
    print(f"wrote {out} ({len(filled.splitlines())} lines)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
