#!/usr/bin/env bash
# tools/agy/pretool-guard.sh — agy PreToolUse hook. Asks tools/guard.sh about a
# shell command the agent is about to run; wired by .agents/hooks.json.
#
#   printf '%s' "$PAYLOAD" | bash tools/agy/pretool-guard.sh
#
# agy sends the event as JSON on stdin (verified on agy 1.5.15):
#   {"cwd":"/abs/dir","hookEventName":"PreToolUse",
#    "toolCall":{"args":{"CommandLine":"echo probe-ok",...},"id":"call_1","name":"run_command"},
#    "workspacePaths":["/abs/dir"],"lastUserInput":"...","conversationId":"..."}
# Only toolCall.name "run_command" is a shell command, and only its
# toolCall.args.CommandLine is checked. lastUserInput echoes the prompt, so a
# git push the user merely mentioned must not deny an unrelated command.
#
# The reply is one JSON line on stdout, {"decision":"allow"} or
# {"decision":"deny","reason":"<guard's stderr line>"}, and the exit code is 0.
#
# Fail open: a missing jq, a payload that is not JSON, or a run_command without
# a CommandLine string allows the command and notes it on stderr, because a
# broken hook must not wedge the agent. guard.sh still fails closed on its own
# usage errors (an unreadable --cwd, for example); those arrive here as a deny.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
repo=$(cd -- "$script_dir/../.." && pwd -P)
guard="$repo/tools/guard.sh"

# allow — prints the allow verdict and exits.
allow() {
  printf '{"decision":"allow"}\n'
  exit 0
}

# note <reason> — says on stderr why a command is allowed without a check.
note() {
  printf 'pretool-guard: %s; allowing the command\n' "$1" >&2
}

payload=$(cat)

if ! command -v jq >/dev/null 2>&1; then
  note "jq is not installed"
  allow
fi

name=$(printf '%s' "$payload" | jq -r '.toolCall.name // empty' 2>/dev/null) || {
  note "payload is not JSON"
  allow
}
if [[ $name != run_command ]]; then
  allow
fi

if ! printf '%s' "$payload" | jq -e '.toolCall.args.CommandLine | type == "string"' >/dev/null 2>&1; then
  note "run_command has no CommandLine string"
  allow
fi
cmd=$(printf '%s' "$payload" | jq -r '.toolCall.args.CommandLine')
cwd=$(printf '%s' "$payload" | jq -r '.cwd // empty' 2>/dev/null) || cwd=""
[[ -n $cwd ]] || cwd=$repo

rc=0
reason=$(bash "$guard" --cwd "$cwd" -- "$cmd" 2>&1 >/dev/null) || rc=$?
case $rc in
  0)
    allow
    ;;
  2)
    reason=${reason%%$'\n'*}
    [[ -n $reason ]] || reason="guard: denied"
    jq -cn --arg reason "$reason" '{decision: "deny", reason: $reason}'
    exit 0
    ;;
  *)
    note "guard.sh exited $rc"
    allow
    ;;
esac
