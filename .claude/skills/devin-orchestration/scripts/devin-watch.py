#!/usr/bin/env python3
"""Watchdog for one Devin run directory; exits with a short report.

Usage: python3 -I devin-watch.py <run-dir> [--stall-min 15] [--poll 30]

Exits 0 with one line when the latest attempt finished, and exits 1 with a
stall report when the session gained no new message for --stall-min
minutes. Silence alone is not a hang: a long build inside one exec call adds
no messages, which is why the report lists the live descendant processes.
"""

import argparse
import json
import sqlite3
import subprocess
import sys
import time
from pathlib import Path

DB = Path.home() / ".local/share/devin/cli/sessions.db"
SNIPPET = 500


def read(path: Path) -> str | None:
    try:
        return path.read_text().strip()
    except FileNotFoundError:
        return None


def alive(pid: str | None) -> bool:
    if not pid:
        return False
    return subprocess.run(["kill", "-0", pid], capture_output=True).returncode == 0


def latest_node(session_id: str) -> tuple[int, int, str] | None:
    """(node_id, created_at, chat_message) of the newest node, or None."""
    con = sqlite3.connect(f"file:{DB}?mode=ro", uri=True, timeout=10)
    try:
        return con.execute(
            "select node_id, created_at, chat_message from message_nodes "
            "where session_id = ? order by node_id desc limit 1",
            (session_id,),
        ).fetchone()
    finally:
        con.close()


def describe(chat_message: str) -> str:
    try:
        msg = json.loads(chat_message)
    except json.JSONDecodeError:
        return chat_message[:SNIPPET]
    role = msg.get("role", "?")
    calls = msg.get("tool_calls") or []
    if calls:
        call = calls[0]
        args = json.dumps(call.get("arguments", {}), ensure_ascii=False)
        return f"[{role}] tool_call {call.get('name')}: {args}"[:SNIPPET]
    return f"[{role}] {msg.get('content', '')}"[:SNIPPET]


def descendants(root: str) -> list[str]:
    out = subprocess.run(
        ["ps", "-A", "-o", "pid=,ppid=,%cpu=,etime=,command="],
        capture_output=True, text=True,
    ).stdout
    rows = {}
    children: dict[str, list[str]] = {}
    for line in out.splitlines():
        parts = line.split(None, 4)
        if len(parts) < 5:
            continue
        pid, ppid = parts[0], parts[1]
        rows[pid] = parts
        children.setdefault(ppid, []).append(pid)
    found, stack = [], list(children.get(root, []))
    while stack:
        pid = stack.pop()
        p = rows[pid]
        found.append(f"  pid={p[0]} cpu={p[2]}% etime={p[3]} {p[4][:120]}")
        stack.extend(children.get(pid, []))
    return found


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--stall-min", type=float, default=15)
    ap.add_argument("--poll", type=float, default=30)
    ap.add_argument("--grace", type=float, default=60,
                    help="seconds a missing or dead pid may persist before the run counts as gone")
    args = ap.parse_args()
    run_dir = args.run_dir.resolve()

    last_id, last_change = None, time.time()
    # devin-run.sh writes `attempt` before `pid`, and a resumed attempt briefly
    # sees the previous attempt's dead pid: only a pid that stays missing or
    # dead for the whole grace period means the wrapper is gone.
    gone_since = None
    while True:
        attempt = read(run_dir / "attempt")
        exit_code = read(run_dir / f"exit_code.{attempt}") if attempt else None
        if exit_code is not None:
            print(f"devin attempt {attempt} finished exit={exit_code}")
            return 0
        pid = read(run_dir / "pid")
        if attempt and not alive(pid):
            gone_since = gone_since or time.time()
            if time.time() - gone_since >= args.grace:
                print(f"devin attempt {attempt}: process {pid} gone without exit_code (wrapper killed?)")
                return 1
        else:
            gone_since = None

        session_id = read(run_dir / "session_id")
        node = latest_node(session_id) if session_id else None
        if node and node[0] != last_id:
            last_id, last_change = node[0], time.time()

        idle_min = (time.time() - last_change) / 60
        if idle_min >= args.stall_min:
            print(f"devin attempt {attempt} STALL: no new message for {idle_min:.0f} min "
                  f"(session={session_id}, node={last_id})")
            print("last message: " + (describe(node[2]) if node else "none"))
            procs = descendants(pid) if pid else []
            print("descendant processes:" if procs else "descendant processes: none")
            print("\n".join(procs))
            return 1
        time.sleep(args.poll)


if __name__ == "__main__":
    sys.exit(main())
