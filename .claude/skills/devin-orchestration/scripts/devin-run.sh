#!/usr/bin/env bash
# Run one Devin attempt for a run directory. Every byte Devin prints goes to
# files in the run directory; this script itself prints one status line, so
# the orchestrator's context only ever sees that line.
#
# Usage: devin-run.sh <run-dir> [--workdir DIR] [--model MODEL] [--prompt FILE]
#
# Attempt 1 reads <run-dir>/brief.md (or --prompt). Every later attempt
# resumes the recorded session and needs --prompt with the follow-up message.
# Files per attempt N: stdout.N.txt, stderr.N.txt, export.N.json, exit_code.N.
# Shared files: attempt (latest N), session_id, pid (the running devin).
set -uo pipefail

die() { echo "devin-run: $*" >&2; exit 2; }

[ $# -ge 1 ] || die "usage: devin-run.sh <run-dir> [--workdir DIR] [--model MODEL] [--prompt FILE]"
run_dir=$1; shift
workdir=$PWD
model=swe-2-max
prompt=
while [ $# -gt 0 ]; do
  case $1 in
    --workdir) workdir=$2; shift 2 ;;
    --model) model=$2; shift 2 ;;
    --prompt) prompt=$2; shift 2 ;;
    *) die "unknown argument: $1" ;;
  esac
done

[ -d "$run_dir" ] || die "run dir does not exist: $run_dir"
run_dir=$(cd "$run_dir" && pwd)
workdir=$(cd "$workdir" && pwd) || die "workdir does not exist: $workdir"

# One live attempt per run directory: a second one would interleave writes
# into the same session.
if [ -f "$run_dir/pid" ] && kill -0 "$(cat "$run_dir/pid")" 2>/dev/null; then
  die "attempt still running (pid $(cat "$run_dir/pid"))"
fi

n=$(( $(cat "$run_dir/attempt" 2>/dev/null || echo 0) + 1 ))
resume=()
if [ "$n" -eq 1 ]; then
  prompt=${prompt:-$run_dir/brief.md}
else
  [ -n "$prompt" ] || die "attempt $n resumes the session and needs --prompt FILE"
  [ -s "$run_dir/session_id" ] || die "attempt $n needs $run_dir/session_id"
  resume=(-r "$(cat "$run_dir/session_id")")
fi
[ -f "$prompt" ] || die "prompt file not found: $prompt"
prompt=$(cd "$(dirname "$prompt")" && pwd)/$(basename "$prompt")
echo "$n" > "$run_dir/attempt"

db=$HOME/.local/share/devin/cli/sessions.db
start=$(date +%s)

cd "$workdir" || die "cannot cd to $workdir"
devin -p --prompt-file "$prompt" --model "$model" --permission-mode dangerous \
  --export "$run_dir/export.$n.json" "${resume[@]}" \
  </dev/null >"$run_dir/stdout.$n.txt" 2>"$run_dir/stderr.$n.txt" &
pid=$!
echo "$pid" > "$run_dir/pid"

# The export is written only when the run ends, so a watchdog needs the
# session id earlier: take the newest session Devin created in this workdir
# after the launch. A resumed attempt already knows its id.
if [ "$n" -eq 1 ]; then
  for _ in $(seq 1 120); do
    sid=$(sqlite3 -readonly "$db" \
      "select id from sessions where working_directory = '${workdir//\'/\'\'}' and created_at >= $start order by created_at desc limit 1" 2>/dev/null)
    if [ -n "$sid" ]; then echo "$sid" > "$run_dir/session_id"; break; fi
    kill -0 "$pid" 2>/dev/null || break
    sleep 1
  done
fi

wait "$pid"
ec=$?
echo "$ec" > "$run_dir/exit_code.$n"

# The export is the authoritative record of which session ran.
if [ -s "$run_dir/export.$n.json" ]; then
  sid=$(python3 -I -c 'import json,sys; print(json.load(open(sys.argv[1]))["session_id"])' "$run_dir/export.$n.json" 2>/dev/null)
  [ -n "$sid" ] && echo "$sid" > "$run_dir/session_id"
fi

echo "devin attempt $n exit=$ec session=$(cat "$run_dir/session_id" 2>/dev/null || echo unknown) elapsed=$(( $(date +%s) - start ))s"
exit "$ec"
