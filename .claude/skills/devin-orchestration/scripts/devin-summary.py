#!/usr/bin/env python3
"""Compact digest of one Devin attempt, so the export never enters context.

Usage: python3 -I devin-summary.py <run-dir> [--attempt N]

Prints: step and token totals, tool calls by name, files touched by
write-type tools, exec calls that exited non-zero (command + last output
line), and the final agent message truncated.
"""

import argparse
import json
import re
import sys
from collections import Counter
from pathlib import Path

EXIT_RE = re.compile(r"Exit code: (-?\d+)")
READ_ONLY_TOOLS = {"read", "grep", "glob", "ls", "todo_write", "ask_user_question"}


def short(text: str, limit: int) -> str:
    text = " ".join(text.split())
    return text if len(text) <= limit else text[: limit - 1] + "…"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--attempt", type=int)
    args = ap.parse_args()
    run_dir = args.run_dir.resolve()
    n = args.attempt or int((run_dir / "attempt").read_text().strip())

    exit_code = (run_dir / f"exit_code.{n}")
    print(f"attempt {n}: exit={exit_code.read_text().strip() if exit_code.exists() else 'running'}")
    stderr = run_dir / f"stderr.{n}.txt"
    # Blank lines carry nothing; a file of only whitespace counts as empty.
    stderr_lines = ([line for line in stderr.read_text(errors="replace").splitlines() if line.strip()]
                    if stderr.exists() else [])
    if stderr_lines:
        print(f"stderr ({stderr.stat().st_size} B), last line: {short(stderr_lines[-1], 200)}")

    export = run_dir / f"export.{n}.json"
    if not export.exists():
        print("no export (run unfinished or crashed before writing it)")
        return 1
    data = json.loads(export.read_text())
    steps = data.get("steps", [])
    metrics = data.get("final_metrics", {})
    print(f"session={data.get('session_id')} steps={len(steps)} "
          f"prompt_tokens={metrics.get('total_prompt_tokens')} "
          f"completion_tokens={metrics.get('total_completion_tokens')}")

    tools: Counter[str] = Counter()
    touched: dict[str, str] = {}
    failures = []
    final = ""
    for step in steps:
        if step.get("source") != "agent":
            continue
        results = {r.get("source_call_id"): r.get("content", "")
                   for r in (step.get("observation") or {}).get("results", [])}
        for call in step.get("tool_calls") or []:
            name = call.get("function_name", "?")
            tools[name] += 1
            arguments = call.get("arguments") or {}
            if name not in READ_ONLY_TOOLS and "file_path" in arguments:
                touched[arguments["file_path"]] = name
            if name == "exec":
                output = str(results.get(call.get("tool_call_id"), ""))
                codes = EXIT_RE.findall(output)
                if codes and codes[-1] != "0":
                    body = [l for l in EXIT_RE.split(output)[0].strip().splitlines() if l.strip()]
                    failures.append((codes[-1], short(arguments.get("command", ""), 160),
                                     short(body[-1] if body else "", 160)))
        if step.get("message"):
            final = step["message"]

    print("tool calls: " + ", ".join(f"{k}={v}" for k, v in tools.most_common()))
    print(f"files touched ({len(touched)}):")
    for path, tool in sorted(touched.items()):
        print(f"  {tool}: {path}")
    print(f"non-zero exec ({len(failures)}):")
    for code, command, last in failures:
        print(f"  exit={code} $ {command}\n    -> {last}")
    print("final message: " + short(final, 1500))
    return 0


if __name__ == "__main__":
    sys.exit(main())
