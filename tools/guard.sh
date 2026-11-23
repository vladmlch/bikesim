#!/usr/bin/env bash
# tools/guard.sh — deny-list for a shell command an agent is about to run.
# One source of truth for every agent harness; the harness adapters call it.
#
#   bash tools/guard.sh [--cwd DIR] [--] [COMMAND]
#   printf '%s' "$COMMAND" | bash tools/guard.sh [--cwd DIR]
#
# COMMAND is the argument, or all of stdin when there is none. Exit 0 allows
# it and prints nothing. Exit 2 denies it and prints one reason line on
# stderr; a usage error also exits 2, so a request the guard cannot read is
# never allowed. The command is parsed, never executed.
#
# Denied, matched on words after quote removal (never on substrings):
#   git reset --hard   discards uncommitted work in the shared tree
#   git clean          deletes untracked files another agent may be using
#   git push           publishes work; only the user pushes
#   git stash          drops other agents' work in the shared tree
#   BIKE_GOLDEN_RECORD=1, or the misspelt BIKE_RECORD_GOLDENS=1, as a prefix,
#                      an export argument or an env argument: rewrites goldens.
#                      Allowed when the guard's own environment has
#                      BIKE_GUARD_ALLOW_GOLDENS=1.
#   rm with a recursive and a force flag on a target outside the repository.
#                      Relative targets resolve against --cwd (default: the
#                      current directory) and against any earlier cd in the
#                      command. A target the guard cannot decide statically
#                      (a $ expansion, braces, ~user, a hidden-name glob) counts
#                      as outside. The repository root and .git are denied even
#                      though they are inside.
#   A command name or git subcommand that is not a literal word (a $ expansion).
#
# Parsing: the command is split on ; & | ( ) and newlines outside quotes.
# Comments, heredoc bodies and redirection targets are skipped. $(...) and
# `...` are checked as commands of their own; the word that held them becomes
# a bare $, which the rules above treat as unknown.
#
# A backstop against honest mistakes, not a sandbox: sudo, xargs, bash -c and
# eval are not unwrapped, and a symlink inside the repository is not followed.
set -euo pipefail

GUARD_ROOT=""   # physical repository root
GUARD_BASE=""   # directory relative rm targets resolve against; "" = unknown
RESOLVED=""     # output of guard_resolve
EXPANDED=""     # output of guard_expand
GUARD_CLOSE=0   # output of guard_find_close: index of the matching )
GW=()           # tokenizer output: words, quotes removed
GS=()           # tokenizer output: index of each simple command's first word
G_BUF=""        # tokenizer state: word being built
G_HAVE=0        # tokenizer state: a word is pending (may be empty when quoted)
G_SKIP=0        # tokenizer state: the next word is a redirection target
G_HDWAIT=0      # tokenizer state: the next word is a heredoc delimiter
G_HDQ=""        # tokenizer state: queued heredoc delimiters, each ended by a newline

fail() {
  printf 'guard: %s\n' "$1" >&2
  exit 2
}

# guard_deny <reason> — the deny verdict: one line on stderr, exit 2.
guard_deny() {
  printf 'guard: denied: %s\n' "$1" >&2
  exit 2
}

# --- tokenizer --------------------------------------------------------------

# _guard_flush — ends the pending word. After a redirection operator the word
# is dropped; a heredoc delimiter is queued; anything else joins GW.
_guard_flush() {
  if ((G_HAVE)); then
    if ((G_HDWAIT)); then
      G_HDQ+="$G_BUF"$'\n'
      G_HDWAIT=0
    elif ((G_SKIP)); then
      G_SKIP=0
    else
      GW+=("$G_BUF")
    fi
  fi
  G_BUF=""
  G_HAVE=0
}

# _guard_sep — ends the pending word and the simple command.
_guard_sep() {
  _guard_flush
  GS+=("${#GW[@]}")
}

# _guard_skip_bodies <command> <index> — prints the index just past the
# heredoc bodies queued in G_HDQ. <index> is the first character after the
# newline that ends the line the heredocs were started on.
_guard_skip_bodies() {
  local s=$1 n=${#1} i=$2 nl=$'\n' d rest line t
  while [[ -n $G_HDQ ]]; do
    d=${G_HDQ%%"$nl"*}
    G_HDQ=${G_HDQ#*"$nl"}
    while ((i < n)); do
      rest=${s:i}
      line=${rest%%"$nl"*}
      t=$line
      while [[ $t == $'\t'* ]]; do
        t=${t#$'\t'}
      done
      i=$((i + ${#line} + 1))
      [[ $t == "$d" ]] && break
    done
  done
  printf '%s\n' "$i"
}

# guard_tokenize <command> — fills GW and GS. A heredoc body is not tokenized
# as commands; the pre-pass in guard_expand still checks $(...) inside it.
guard_tokenize() {
  local s=$1 n=${#1} i=0 c j nx op
  GW=()
  GS=(0)
  G_BUF=""
  G_HAVE=0
  G_SKIP=0
  G_HDWAIT=0
  G_HDQ=""
  while ((i < n)); do
    c=${s:i:1}
    case $c in
      ' '|$'\t')
        _guard_flush
        i=$((i + 1))
        ;;
      $'\n')
        _guard_sep
        i=$((i + 1))
        if [[ -n $G_HDQ ]]; then
          i=$(_guard_skip_bodies "$s" "$i")
          G_HDQ=""
        fi
        ;;
      ';'|'|'|'('|')')
        _guard_sep
        i=$((i + 1))
        ;;
      '<'|'>'|'&')
        if [[ $c == '&' && ${s:i+1:1} != '>' ]]; then
          _guard_sep
          i=$((i + 1))
          continue
        fi
        # "2>file": a word of digits right before the operator is a descriptor.
        case $G_BUF in
          ''|*[!0-9]*) _guard_flush ;;
          *) G_BUF=""; G_HAVE=0 ;;
        esac
        op=""
        while ((i < n && ${#op} < 3)); do
          case ${s:i:1} in
            '<'|'>'|'&') op+=${s:i:1}; i=$((i + 1)) ;;
            *) break ;;
          esac
        done
        case $op in
          '<<'|'<<-') G_HDWAIT=1 ;;
          *) G_SKIP=1 ;;
        esac
        ;;
      "'")
        j=$((i + 1))
        while ((j < n)) && [[ ${s:j:1} != "'" ]]; do
          j=$((j + 1))
        done
        G_BUF+=${s:i+1:j-i-1}
        G_HAVE=1
        i=$((j + 1))
        ;;
      '"')
        G_HAVE=1
        i=$((i + 1))
        while ((i < n)) && [[ ${s:i:1} != '"' ]]; do
          c=${s:i:1}
          nx=${s:i+1:1}
          if [[ $c == \\ ]]; then
            case $nx in
              '"'|\\|'$'|'`')
                G_BUF+=$nx
                i=$((i + 2))
                continue
                ;;
            esac
          fi
          G_BUF+=$c
          i=$((i + 1))
        done
        i=$((i + 1))
        ;;
      \\)
        nx=${s:i+1:1}
        if [[ -n $nx && $nx != $'\n' ]]; then
          G_BUF+=$nx
          G_HAVE=1
        fi
        i=$((i + 2))
        ;;
      '#')
        if ((G_HAVE)); then
          G_BUF+=$c
          i=$((i + 1))
        else
          j=$i
          while ((j < n)) && [[ ${s:j:1} != $'\n' ]]; do
            j=$((j + 1))
          done
          i=$j
        fi
        ;;
      *)
        G_BUF+=$c
        G_HAVE=1
        i=$((i + 1))
        ;;
    esac
  done
  _guard_sep
}

# --- pre-pass: command substitutions ----------------------------------------

# guard_find_close <command> <index> — sets GUARD_CLOSE to the ) that closes a
# $( whose body starts at <index>. Quotes inside the body are skipped.
guard_find_close() {
  local s=$1 n=${#1} j=$2 depth=1 c q
  while ((j < n)); do
    c=${s:j:1}
    case $c in
      \\)
        j=$((j + 2))
        continue
        ;;
      "'")
        q=$((j + 1))
        while ((q < n)) && [[ ${s:q:1} != "'" ]]; do
          q=$((q + 1))
        done
        j=$((q + 1))
        continue
        ;;
      '"')
        q=$((j + 1))
        while ((q < n)) && [[ ${s:q:1} != '"' ]]; do
          if [[ ${s:q:1} == \\ ]]; then
            q=$((q + 1))
          fi
          q=$((q + 1))
        done
        j=$((q + 1))
        continue
        ;;
      '(')
        depth=$((depth + 1))
        ;;
      ')')
        depth=$((depth - 1))
        if ((depth == 0)); then
          GUARD_CLOSE=$j
          return 0
        fi
        ;;
    esac
    j=$((j + 1))
  done
  guard_deny "unterminated \$( substitution"
}

# guard_expand <command> — sets EXPANDED: each $(...) and `...` is checked as
# a command of its own and replaced by a bare $. Quotes are ignored here on
# purpose: a heredoc body with an apostrophe must not hide a live $( from the
# scan, at the cost of checking a quoted $( that never runs.
guard_expand() {
  local s=$1 n=${#1} i=0 j c saved inner_len out=""
  while ((i < n)); do
    c=${s:i:1}
    if [[ $c == \\ ]]; then
      out+=${s:i:2}
      i=$((i + 2))
    elif [[ $c == '$' && ${s:i+1:1} == '(' ]]; then
      guard_find_close "$s" $((i + 2))
      inner_len=$((GUARD_CLOSE - i - 2))
      saved=$GUARD_BASE
      guard_verdict "${s:i+2:inner_len}"
      GUARD_BASE=$saved
      out+='$'
      i=$((GUARD_CLOSE + 1))
    elif [[ $c == '`' ]]; then
      j=$((i + 1))
      while ((j < n)) && [[ ${s:j:1} != '`' ]]; do
        if [[ ${s:j:1} == \\ ]]; then
          j=$((j + 1))
        fi
        j=$((j + 1))
      done
      ((j < n)) || guard_deny "unterminated backtick substitution"
      saved=$GUARD_BASE
      guard_verdict "${s:i+1:j-i-1}"
      GUARD_BASE=$saved
      out+='$'
      i=$((j + 1))
    else
      out+=$c
      i=$((i + 1))
    fi
  done
  EXPANDED=$out
}

# --- simple-command rules ---------------------------------------------------

# guard_is_assignment <word> — NAME=value with a shell-valid NAME.
guard_is_assignment() {
  local name=${1%%=*}
  [[ $1 == *=* ]] || return 1
  case $name in
    ''|[0-9]*|*[!A-Za-z0-9_]*) return 1 ;;
  esac
  return 0
}

# guard_check_golden <assignment> — the golden re-record switch, as read by
# native/tests/support: the value must be exactly 1. A value the shell would
# expand could still be 1, so it is denied too.
guard_check_golden() {
  local name=${1%%=*} value=${1#*=}
  case $name in
    BIKE_GOLDEN_RECORD|BIKE_RECORD_GOLDENS) ;;
    *) return 0 ;;
  esac
  case $value in
    1|*'$'*|*'`'*) ;;
    *) return 0 ;;
  esac
  if [[ ${BIKE_GUARD_ALLOW_GOLDENS:-} == 1 ]]; then
    return 0
  fi
  guard_deny "$name=$(printf '%q' "$value") re-records goldens; BIKE_GUARD_ALLOW_GOLDENS=1 in the guard's environment allows it"
}

# guard_check_git <argument>... — the words after `git`.
guard_check_git() {
  local a sub=""
  while (($#)); do
    a=$1
    shift
    case $a in
      -C|-c|--git-dir|--work-tree|--namespace)
        if (($#)); then
          shift
        fi
        ;;
      -*) ;;
      *)
        sub=$a
        break
        ;;
    esac
  done
  case $sub in
    *'$'*|*'`'*) guard_deny "git subcommand is not a literal word: $(printf '%q' "$sub")" ;;
    reset)
      for a in "$@"; do
        if [[ $a == --hard ]]; then
          guard_deny "git reset --hard discards uncommitted work in the shared tree"
        fi
      done
      ;;
    clean) guard_deny "git clean deletes untracked files another agent may be using" ;;
    push) guard_deny "git push: only the user publishes work" ;;
    stash) guard_deny "git stash drops other agents' work in the shared tree" ;;
  esac
}

# guard_resolve <base> <path> — sets RESOLVED to the lexical absolute path.
# Fails for a relative path with no base, for ~user, and for ~ without HOME.
guard_resolve() {
  local base=$1 p=$2 full rest comp out k depth=0
  local -a stack
  case $p in
    /*) full=$p ;;
    '~') [[ -n ${HOME:-} ]] || return 1; full=$HOME ;;
    \~/*) [[ -n ${HOME:-} ]] || return 1; full=$HOME/${p#\~/} ;;
    \~*) return 1 ;;
    *) [[ -n $base ]] || return 1; full=$base/$p ;;
  esac
  rest=${full#/}
  while [[ -n $rest ]]; do
    comp=${rest%%/*}
    case $rest in
      */*) rest=${rest#*/} ;;
      *) rest="" ;;
    esac
    case $comp in
      ''|.) ;;
      ..)
        if ((depth > 0)); then
          depth=$((depth - 1))
        fi
        ;;
      *)
        stack[depth]=$comp
        depth=$((depth + 1))
        ;;
    esac
  done
  out=""
  k=0
  while ((k < depth)); do
    out="$out/${stack[k]}"
    k=$((k + 1))
  done
  RESOLVED=${out:-/}
}

# guard_check_rm_target <path> — the target must resolve inside the repository.
guard_check_rm_target() {
  local t=$1 rest comp
  [[ -n $t ]] || guard_deny "rm target is empty"
  case $t in
    *'$'*|*'`'*|*'{'*|*'}'*)
      guard_deny "rm target is not a literal path: $(printf '%q' "$t")"
      ;;
  esac
  rest=$t
  while :; do
    comp=${rest%%/*}
    case $comp in
      .*)
        case $comp in
          *[*?[]*) guard_deny "rm target could match .git through a hidden-name glob: $(printf '%q' "$t")" ;;
        esac
        ;;
    esac
    case $rest in
      */*) rest=${rest#*/} ;;
      *) break ;;
    esac
  done
  guard_resolve "$GUARD_BASE" "$t" || guard_deny "rm target cannot be resolved: $(printf '%q' "$t")"
  case $RESOLVED in
    "$GUARD_ROOT"|"$GUARD_ROOT/.git"|"$GUARD_ROOT/.git"/*)
      guard_deny "rm would remove the repository root or its .git: $(printf '%q' "$t")"
      ;;
    "$GUARD_ROOT"/*) ;;
    *) guard_deny "rm target is outside the repository: $(printf '%q' "$t") resolves to $RESOLVED" ;;
  esac
}

# guard_check_rm <argument>... — the words after `rm`.
guard_check_rm() {
  local a recursive=0 force=0 ends=0 k=0 nt=0
  local -a targets
  while (($#)); do
    a=$1
    shift
    if ((ends)); then
      targets[nt]=$a
      nt=$((nt + 1))
    elif [[ $a == -- ]]; then
      ends=1
    elif [[ $a == --recursive ]]; then
      recursive=1
    elif [[ $a == --force ]]; then
      force=1
    elif [[ $a == --* ]]; then
      :
    elif [[ $a == -?* ]]; then
      case $a in *[rR]*) recursive=1 ;; esac
      case $a in *f*) force=1 ;; esac
    else
      targets[nt]=$a
      nt=$((nt + 1))
    fi
  done
  ((recursive && force)) || return 0
  while ((k < nt)); do
    guard_check_rm_target "${targets[k]}"
    k=$((k + 1))
  done
}

# guard_track_cd <argument>... — cd and pushd move the base for later relative
# rm targets. A destination the guard cannot read (cd -, a $ expansion) clears it.
guard_track_cd() {
  local a dest="~"
  while (($#)); do
    a=$1
    shift
    case $a in
      -) dest=-; break ;;
      -*) ;;
      *) dest=$a; break ;;
    esac
  done
  case $dest in
    -|*'$'*|*'`'*) GUARD_BASE=""; return 0 ;;
  esac
  if guard_resolve "$GUARD_BASE" "$dest"; then
    GUARD_BASE=$RESOLVED
  else
    GUARD_BASE=""
  fi
}

# guard_is_wrapper <word> — reserved words and wrappers that precede the real
# command name; the name is what gets checked.
guard_is_wrapper() {
  case $1 in
    '{'|'!'|if|then|do|else|elif|while|until|time|command|builtin|exec|nohup|-*) return 0 ;;
  esac
  return 1
}

# guard_check_simple <word>... — one simple command, words already unquoted.
guard_check_simple() {
  local -a w
  local i=0 m word name k
  w=("$@")
  m=${#w[@]}
  while ((i < m)); do
    word=${w[i]}
    if guard_is_assignment "$word"; then
      guard_check_golden "$word"
      i=$((i + 1))
    elif [[ $word == env ]]; then
      i=$((i + 1))
      while ((i < m)); do
        case ${w[i]} in
          -u|-C|-S) i=$((i + 2)) ;;
          -*) i=$((i + 1)) ;;
          *) break ;;
        esac
      done
    elif guard_is_wrapper "$word"; then
      i=$((i + 1))
    else
      break
    fi
  done
  ((i < m)) || return 0
  word=${w[i]}
  case $word in
    *'$'*|*'`'*) guard_deny "command name is not a literal word: $(printf '%q' "$word")" ;;
  esac
  name=${word##*/}
  case $name in
    git) guard_check_git "${w[@]:i+1}" ;;
    rm) guard_check_rm "${w[@]:i+1}" ;;
    cd|pushd) guard_track_cd "${w[@]:i+1}" ;;
    popd) GUARD_BASE="" ;;
    export)
      k=$((i + 1))
      while ((k < m)); do
        if guard_is_assignment "${w[k]}"; then
          guard_check_golden "${w[k]}"
        fi
        k=$((k + 1))
      done
      ;;
  esac
}

# guard_verdict <command> — checks every simple command in order. Returns
# only when all of them pass; a denial exits 2.
guard_verdict() {
  local k=0 start end
  local -a sc
  guard_expand "$1"
  guard_tokenize "$EXPANDED"
  while ((k < ${#GS[@]} - 1)); do
    start=${GS[k]}
    end=${GS[k + 1]}
    if ((end > start)); then
      sc=("${GW[@]:start:end-start}")
      guard_check_simple "${sc[@]}"
    fi
    k=$((k + 1))
  done
  return 0
}

main() {
  local cwd="" cmd="" base
  while (($#)); do
    case $1 in
      --cwd)
        (($# >= 2)) || fail "--cwd needs a directory"
        cwd=$2
        shift 2
        ;;
      --cwd=*)
        cwd=${1#--cwd=}
        shift
        ;;
      --)
        shift
        break
        ;;
      -h|--help)
        printf 'usage: guard.sh [--cwd DIR] [--] [COMMAND]\n'
        exit 0
        ;;
      -*) fail "unknown option: $1" ;;
      *) break ;;
    esac
  done
  (($# <= 1)) || fail "expected one COMMAND, got $#"
  if (($# == 1)); then
    cmd=$1
  else
    cmd=$(cat)
  fi
  GUARD_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P) \
    || fail "cannot locate the repository root"
  if [[ -n $cwd ]]; then
    base=$(cd -- "$cwd" 2>/dev/null && pwd -P) || fail "--cwd is not a directory: $cwd"
  else
    base=$(pwd -P) || fail "cannot read the current directory"
  fi
  GUARD_BASE=$base
  guard_verdict "$cmd"
}

main "$@"
