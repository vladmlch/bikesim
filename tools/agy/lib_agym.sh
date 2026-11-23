#!/usr/bin/env bash
# tools/agy/lib_agym.sh — quota-reset parsing for tools/agy/agym. Sourced, never run.
#
# agy logs each error with a glog prefix that has no year and no zone:
#   E1117 23:18:38.409415   29699 errorreport.go:307] ... code 429): ... Resets in 1h45m13s.
#   I1122 23:16:35.796617  781482 runner.go:957] ... code 429): ... Your quota will reset after 19m9s.
# The quota reset is the line's local time plus that relative Go duration. The
# two wordings and the choice of the latest line follow agy-orchestration's
# scripts/agy-quota.py. The year comes from the agym-YYYY... name of the wrapper's
# own log file, else from the current date. Bash 3.2 compatible: macOS /bin/bash
# is the reference.

# agym_duration_seconds DURATION — prints whole seconds for a Go duration made of
# h, m and s parts ("1h45m13s", "13.5s"); a fraction is truncated. Returns 1 for
# anything else (ms, us, a stray character), so the caller prints the raw text.
agym_duration_seconds() {
  local rest=$1 total=0 re='^([0-9]+)(\.[0-9]+)?(h|m|s)'
  [[ -n $rest ]] || return 1
  while [[ -n $rest ]]; do
    [[ $rest =~ $re ]] || return 1
    local n=${BASH_REMATCH[1]} unit=${BASH_REMATCH[3]}
    case $unit in
      h) total=$((total + 10#$n * 3600)) ;;
      m) total=$((total + 10#$n * 60)) ;;
      s) total=$((total + 10#$n)) ;;
    esac
    rest=${rest:${#BASH_REMATCH[0]}}
  done
  printf '%s\n' "$total"
}

# agym_quota_phrase LINE — prints the reset announcement of LINE, "Resets in <d>" or
# "reset after <d>", with the duration's trailing period removed. Returns 1 when
# the line has neither wording.
agym_quota_phrase() {
  local re='(Resets in|reset after) ([0-9hms.]+)'
  [[ $1 =~ $re ]] || return 1
  printf '%s %s\n' "${BASH_REMATCH[1]}" "${BASH_REMATCH[2]%.}"
}

# agym_local_after YYYYMMDD HH:MM:SS SECONDS — prints the local-time instant
# "YYYY-MM-DD HH:MM:SS" that many seconds after the given local time. Returns 1
# when the time does not parse. BSD date (macOS) and GNU date (Linux) differ.
agym_local_after() {
  local day=$1 hms=$2 add=$3 epoch
  if [[ $(uname -s) == Darwin ]]; then
    epoch=$(date -j -f '%Y%m%d %H:%M:%S' "$day $hms" '+%s' 2>/dev/null) || return 1
    date -r "$((epoch + add))" '+%Y-%m-%d %H:%M:%S'
  else
    epoch=$(date -d "${day:0:4}-${day:4:2}-${day:6:2} $hms" '+%s' 2>/dev/null) || return 1
    date -d "@$((epoch + add))" '+%Y-%m-%d %H:%M:%S'
  fi
}

# agym_report_quota LOG — when LOG holds a quota reset, prints to stderr the local
# reset time and, when a conversation id precedes that 429, the resume line
# `agym --conversation=<id>`. The reset comes from the latest 429 line that
# announces one: a later line is a fresher countdown, and an earlier one may
# belong to a quota that has since reset. Prints nothing otherwise, including
# when LOG does not exist. Always returns 0.
agym_report_quota() {
  local log=$1 hit line_no line re mmdd hms year phrase dur secs when id
  re='^[IWEF]([0-9]{4}) ([0-9]{2}:[0-9]{2}:[0-9]{2})'
  [[ -f $log ]] || return 0
  hit=$(grep -n -E 'RESOURCE_EXHAUSTED \(code 429\).*(Resets in|reset after) [0-9]' "$log" \
    | tail -n 1) || true
  [[ -n $hit ]] || return 0
  line_no=${hit%%:*}
  line=${hit#*:}
  [[ $line =~ $re ]] || return 0
  mmdd=${BASH_REMATCH[1]}
  hms=${BASH_REMATCH[2]}
  year=$(date +%Y)
  if [[ $(basename -- "$log") =~ ^agym-([0-9]{4})[0-9]{4}T ]]; then
    year=${BASH_REMATCH[1]}
  fi
  phrase=$(agym_quota_phrase "$line") || return 0
  dur=${phrase##* }
  if secs=$(agym_duration_seconds "$dur") && when=$(agym_local_after "$year$mmdd" "$hms" "$secs"); then
    printf 'agym: quota resets at %s local time (429 "%s").\n' "$when" "$phrase" >&2
  else
    printf 'agym: quota resets in %s (429; the local reset time could not be computed).\n' "$dur" >&2
  fi
  # The conversation is the last one the wrapper sent a message to before the 429.
  # Its id is on the "Forwarding user message to conversation" line and on the
  # latency line's cascade_id. The same latency line also has a trajectory_id,
  # which names no brain directory (agy keys ~/.gemini/antigravity-cli/brain/ by
  # conversation id), so only the two anchored forms are read.
  id=""
  if ((line_no > 1)); then
    # BSD head rejects a zero count, so the scan runs only when a line precedes the 429.
    id=$(head -n "$((line_no - 1))" "$log" \
      | sed -nE 's/.*(Forwarding user message to conversation |cascade_id:)([0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}).*/\2/p' \
      | tail -n 1) || true
  fi
  if [[ -n $id ]]; then
    printf 'agym: resume the conversation with:\n' >&2
    printf 'agym --conversation=%s\n' "$id" >&2
  fi
  return 0
}
