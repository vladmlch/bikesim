#!/usr/bin/env bash
# tools/claude/pretool-guard.sh — Claude Code PreToolUse hook. Asks tools/guard.sh
# about a Bash command before it runs; wired in .claude/settings.json.
#
#   printf '%s' "$EVENT" | bash tools/claude/pretool-guard.sh
#
# Claude Code sends the event as JSON on stdin; this reads tool_name,
# tool_input.command and cwd. Any tool other than Bash is allowed. A denied
# command exits 2 with the guard's reason on stderr, which Claude Code blocks
# and shows to the model. An allowed command exits 0 with no output.
#
# Fail open: a missing jq, a payload that is not JSON, or a Bash call without a
# command string allows the call and notes it on stderr, because a broken hook
# must not wedge the session. guard.sh still fails closed on its own usage
# errors (an unreadable --cwd, for example); those arrive here as a block.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
repo=$(cd -- "$script_dir/../.." && pwd -P)
guard="$repo/tools/guard.sh"

# note <reason> — says on stderr why a command is allowed without a check.
note() {
  printf 'pretool-guard: %s; allowing the command\n' "$1" >&2
}

payload=$(cat)

if ! command -v jq >/dev/null 2>&1; then
  note "jq is not installed"
  exit 0
fi

tool=$(printf '%s' "$payload" | jq -r '.tool_name // empty' 2>/dev/null) || {
  note "payload is not JSON"
  exit 0
}
if [[ $tool != Bash ]]; then
  exit 0
fi

if ! printf '%s' "$payload" | jq -e '.tool_input.command | type == "string"' >/dev/null 2>&1; then
  note "Bash call has no command string"
  exit 0
fi
cmd=$(printf '%s' "$payload" | jq -r '.tool_input.command')
cwd=$(printf '%s' "$payload" | jq -r '.cwd // empty' 2>/dev/null) || cwd=""
[[ -n $cwd ]] || cwd=$repo

rc=0
reason=$(bash "$guard" --cwd "$cwd" -- "$cmd" 2>&1 >/dev/null) || rc=$?
case $rc in
  0)
    exit 0
    ;;
  2)
    reason=${reason%%$'\n'*}
    [[ -n $reason ]] || reason="guard: denied"
    printf '%s\n' "$reason" >&2
    exit 2
    ;;
  *)
    note "guard.sh exited $rc"
    exit 0
    ;;
esac
