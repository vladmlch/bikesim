#!/usr/bin/env bash
# tools/agy/lib_agym.sh — quota-reset parsing for tools/agy/agym. Sourced, never run.
#
# agy logs each error with a glog prefix that has no year and no zone:
#   E1117 23:18:38.409415   29699 errorreport.go:307] ... code 429): ... Resets in 1h45m13s.
# The quota reset is the line's local time plus that relative Go duration. The
# year comes from the agym-YYYY... name of the wrapper's own log file, else from
# the current date. Bash 3.2 compatible: macOS /bin/bash is the reference.

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

# agym_quota_duration LINE — prints the duration after "Resets in " with the
# sentence's trailing period removed.
agym_quota_duration() {
  local rest=${1#*Resets in }
  rest=${rest%%[!0-9hms.]*}
  printf '%s\n' "${rest%.}"
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

# agym_report_quota LOG — when LOG holds a 429 line with "Resets in", prints to
# stderr the local reset time and, when a conversation id precedes that line, the
# resume line `agym --conversation=<id>`. Prints nothing otherwise, including when
# LOG does not exist. Always returns 0.
agym_report_quota() {
  local log=$1 hit line_no line re mmdd hms year dur secs when id
  re='^[IWEF]([0-9]{4}) ([0-9]{2}:[0-9]{2}:[0-9]{2})'
  [[ -f $log ]] || return 0
  hit=$(grep -n 'code 429.*Resets in ' "$log" | head -n 1) || true
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
  dur=$(agym_quota_duration "$line")
  if secs=$(agym_duration_seconds "$dur") && when=$(agym_local_after "$year$mmdd" "$hms" "$secs"); then
    printf 'agym: quota resets at %s local time (429 "Resets in %s").\n' "$when" "$dur" >&2
  else
    printf 'agym: quota resets in %s (429; the local reset time could not be computed).\n' "$dur" >&2
  fi
  # The conversation is the last one the wrapper forwarded a message to before
  # the 429; cascade_id lines carry the same id in the latency breakdown.
  id=""
  if ((line_no > 1)); then
    # BSD head rejects a zero count, so the scan runs only when a line precedes the 429.
    id=$(head -n "$((line_no - 1))" "$log" \
      | grep -E 'Forwarding user message to conversation |cascade_id:' \
      | grep -Eo '[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}' \
      | tail -n 1) || true
  fi
  if [[ -n $id ]]; then
    printf 'agym: resume the conversation with:\n' >&2
    printf 'agym --conversation=%s\n' "$id" >&2
  fi
  return 0
}
