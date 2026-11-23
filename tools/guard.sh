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
#   git stash          drops other agents' work in the shared tree. Only the
#                      read-only forms `git stash list` and `git stash show`
#                      are allowed; every other stash form is denied.
#   git global options before the subcommand (-C, -c, --opt=value, --opt value)
#                      are skipped, so the real subcommand is the one checked.
#   BIKE_GOLDEN_RECORD=1, or the misspelt BIKE_RECORD_GOLDENS=1, as a prefix,
#                      an export argument or an env argument: rewrites goldens.
#                      Also through env (/usr/bin/env, env -i, env -S TEXT,
#                      env -S'TEXT') and cmake -E env. Allowed when the guard's
#                      own environment has BIKE_GUARD_ALLOW_GOLDENS=1.
#   rm with a recursive and a force flag on a target outside the repository
#                      and outside the scratch roots. A target strictly inside
#                      $TMPDIR, /tmp or /private/tmp is allowed (lexical and
#                      physical forms of the roots); a root itself, or anything
#                      that resolves to one, is denied. The scratch roots also
#                      include the physical directory each repository-root
#                      symlink .scratch or .work names (when it is a symlink):
#                      a target inside it is allowed, the directory itself is
#                      denied, and a path through the link is judged by the
#                      directory it names. Relative targets resolve against
#                      --cwd (default: the current directory) and against any
#                      earlier cd in the command; a subshell's cd ends with the
#                      subshell. A target the guard cannot decide statically (a
#                      $ expansion, braces, ~user, a hidden-name glob) counts as
#                      outside. The repository root and .git are denied even
#                      though they are inside.
#   rm -rf of a wildcard made only of * and ? (with a *) directly inside a
#                      protected directory: the repository root, a scratch root
#                      or .scratch/.work. `rm -rf *`, `rm -rf /tmp/*` and
#                      `rm -rf .scratch/*` clear the directory; `rm -rf build/*`
#                      and `rm -rf /tmp/mydir/*` are allowed.
#   rm through a repository-root symlink that resolves outside the repository,
#                      other than the scratch links .scratch and .work: `.deps`
#                      (the shared dependency cache) and `.deps/*` are denied.
#   A command name or git subcommand that is not a literal word (a $ expansion).
#
# Parsing: one left-to-right scan that tracks quoting. Single-quoted text,
# comments and quoted heredoc bodies (<<'EOF', <<"EOF", <<\EOF) are text and
# are skipped. $(...), `...` and $(...) inside double quotes or in an unquoted
# heredoc body run, so each one is checked as a command of its own, in the
# scope of the word that held it. $((...)) and ((...)) are arithmetic, not
# commands. Words in a case pattern list are not command names. Each simple
# command is checked when its separator is reached, so cd and subshell scope
# follow the order the shell runs them. The word that held a substitution
# becomes a bare $, which the rules above treat as unknown. An unterminated
# quote or substitution is a syntax error in the shell, and is denied.
# [[ ... ]] is a conditional expression, not a command: its words are operands,
# so && || ( ) < > inside it are not separators or redirections, and a newline
# is blank. Only $(...), backticks and <(...) inside it run, and they are
# checked. It ends at a ]] word. A ; & | or # inside it is a shell error, so the
# guard leaves the conditional there and checks the text after it as usual.
# Reserved words (if then do else elif while until time ! {) before a command
# are skipped, and the word after them is the command start. case, (( and [[
# are reserved only at a command start, and only when unquoted and not after a
# redirection; a quoted or redirected one is an ordinary command name.
#
# A backstop against honest mistakes, not a sandbox: sudo, xargs, bash -c and
# eval are not unwrapped, and a symlink inside the repository or a scratch root
# is not followed, except the repository-root .scratch and .work links named
# above: rm -rf /tmp/link/ is judged by the lexical path /tmp/link. Any other
# repository-root symlink that leaves the repository is denied, not followed.
set -euo pipefail

GUARD_ROOT=""    # physical repository root
GUARD_BASE=""    # directory relative rm targets resolve against; "" = unknown
GUARD_TMP=()     # scratch roots an rm may clear inside (guard_scratch_roots)
GUARD_LINK=()    # repository-root scratch links (.scratch, .work) that are symlinks
GUARD_LINK_TO=() # physical directory each GUARD_LINK names, same order
RESOLVED=""      # output of guard_resolve
ARITH_END=""     # output of guard_arith_end: index of the first ) of the closing ))

fail() {
  printf 'guard: %s\n' "$1" >&2
  exit 2
}

# guard_deny <reason> — the deny verdict: one line on stderr, exit 2.
guard_deny() {
  printf 'guard: denied: %s\n' "$1" >&2
  exit 2
}

# guard_is_keyword <word> — a reserved word that may precede a command name.
guard_is_keyword() {
  case $1 in
    '{'|'!'|if|then|do|else|elif|while|until|time) return 0 ;;
  esac
  return 1
}

# guard_is_delim <index> — true when the character at <index> ends a word: the
# end of the text, blank, newline, or one of ; & | ).
guard_is_delim() {
  case ${G_STR:$1:1} in
    ''|' '|$'\t'|$'\n'|';'|'&'|'|'|')') return 0 ;;
  esac
  return 1
}

# --- scanner ----------------------------------------------------------------
#
# guard_verdict owns the text (G_STR, G_LEN) and the position (G_POS). guard_scan
# owns the state of one command level: the words of the current simple command
# (GW), queued heredocs (G_HD), the subshell base stack (G_STK), the case state
# (G_CS) and the pending word (G_BUF and its flags). Bash local scoping is
# dynamic, so the helpers below act on the state of the scan that calls them,
# and a nested scan (a substitution) declares its own copies and leaves the
# outer word untouched.
#
# G_CS holds one letter per open case: h (after `case`, up to `in`), p (pattern
# list, up to `)`), b (arm body, up to `;;` or `esac`).
#
# G_LEAD is 1 while the simple command has no word yet, or only unquoted
# reserved words before its command name: a command start, where case, (( and
# [[ are reserved. G_QUOTED is 1 when the pending word has quoted or escaped
# text, so it is never a reserved word. G_COND is 1 inside [[ ... ]], where the
# words are operands and are discarded; G_CPAR counts the ( open inside it.

# _guard_top — sets the caller's top to the innermost case state, or "".
_guard_top() {
  top=""
  if [[ -n $G_CS ]]; then
    top=${G_CS:${#G_CS}-1:1}
  fi
}

# _guard_word — a finished word that is not a redirection target or heredoc
# delimiter. Inside [[ ... ]] it is an operand and is dropped. It drives the case
# state; otherwise it joins the simple command. A reserved word keeps G_LEAD;
# any other word, or a quoted one, ends the command start.
_guard_word() {
  local top=""
  if ((G_COND)); then
    return 0
  fi
  _guard_top
  case $top in
    h)
      if [[ $G_BUF == in ]] && ((G_QUOTED == 0)); then
        G_CS=${G_CS%?}p
      fi
      ;;
    p)
      if [[ $G_BUF == "esac" ]] && ((G_QUOTED == 0)); then
        G_CS=${G_CS%?}
      fi
      ;;
    *)
      if ((G_LEAD && G_QUOTED == 0)) && [[ $G_BUF == case ]]; then
        G_CS+=h
      elif ((${#GW[@]} == 0)) && [[ $top == b && $G_BUF == "esac" ]] && ((G_QUOTED == 0)); then
        G_CS=${G_CS%?}
      else
        GW+=("$G_BUF")
        if ((G_QUOTED)) || ! guard_is_keyword "$G_BUF"; then
          G_LEAD=0
        fi
      fi
      ;;
  esac
}

# _guard_flush — ends the pending word. After a redirection operator the word
# is dropped; a heredoc delimiter is queued with its flags (quoted, dash).
_guard_flush() {
  local dash=0
  if ((G_HAVE)); then
    if ((G_HDWAIT)); then
      if ((G_HDWAIT == 2)); then
        dash=1
      fi
      G_HD+=("$G_HDRAW$dash$G_BUF")
      G_HDWAIT=0
    elif ((G_SKIP)); then
      G_SKIP=0
    else
      _guard_word
    fi
  fi
  G_BUF=""
  G_HAVE=0
  G_HDRAW=0
  G_QUOTED=0
}

# _guard_sep — ends the pending word and the simple command, and checks it.
_guard_sep() {
  _guard_flush
  if ((${#GW[@]})); then
    guard_check_simple "${GW[@]}"
  fi
  GW=()
  G_SKIP=0
  G_LEAD=1
}

# _guard_redir — a < or > operator, or &> (the caller handles a lone &). A word
# of digits right before the operator is a file descriptor and is dropped. A
# redirection ends the command start: bash does not treat a reserved word after
# it as reserved, so `>f case` is a command named case.
_guard_redir() {
  local op=""
  G_LEAD=0
  case $G_BUF in
    ''|*[!0-9]*) _guard_flush ;;
    *)
      G_BUF=""
      G_HAVE=0
      G_QUOTED=0
      ;;
  esac
  while ((G_POS < G_LEN)) && ((${#op} < 3)); do
    case ${G_STR:G_POS:1} in
      '<'|'>'|'&')
        op+=${G_STR:G_POS:1}
        G_POS=$((G_POS + 1))
        ;;
      *) break ;;
    esac
  done
  case $op in
    '<<')
      G_HDWAIT=1
      if [[ ${G_STR:G_POS:1} == '-' ]]; then
        G_HDWAIT=2
        G_POS=$((G_POS + 1))
      fi
      ;;
    '<<<') G_SKIP=1 ;;
    *)
      # <(...) and >(...) start a subshell, not a redirection target.
      if [[ ${G_STR:G_POS:1} != '(' ]]; then
        G_SKIP=1
      fi
      ;;
  esac
}

# _guard_bodies — called at the newline that ends the line the queued heredocs
# were started on. A quoted delimiter makes the body text. An unquoted body is
# scanned for the substitutions the shell runs inside it, then skipped.
_guard_bodies() {
  local e quoted dash d p line t rest bstart bend dend c
  local nl=$'\n' k=0
  local -a q
  q=("${G_HD[@]}")
  G_HD=()
  while ((k < ${#q[@]})); do
    e=${q[k]}
    quoted=${e:0:1}
    dash=${e:1:1}
    d=${e:2}
    bstart=$G_POS
    bend=$G_LEN
    dend=$G_LEN
    p=$G_POS
    while ((p < G_LEN)); do
      rest=${G_STR:p}
      line=${rest%%"$nl"*}
      t=$line
      if [[ $dash == 1 ]]; then
        while [[ $t == $'\t'* ]]; do
          t=${t#$'\t'}
        done
      fi
      if [[ $t == "$d" ]]; then
        bend=$p
        dend=$((p + ${#line}))
        break
      fi
      p=$((p + ${#line} + 1))
    done
    if [[ $quoted == 0 ]]; then
      G_POS=$bstart
      while ((G_POS < bend)); do
        c=${G_STR:G_POS:1}
        if [[ $c == \\ ]]; then
          G_POS=$((G_POS + 2))
        elif [[ $c == '`' || ( $c == '$' && ${G_STR:G_POS+1:1} == '(' ) ]]; then
          guard_substitute
        else
          G_POS=$((G_POS + 1))
        fi
      done
    fi
    G_POS=$dend
    if ((G_POS < G_LEN)); then
      G_POS=$((G_POS + 1))
    fi
    k=$((k + 1))
  done
}

# guard_arith_end <index> — <index> is just past an opening (( or $((. Sets
# ARITH_END to the index of the first ) of the closing )) and returns 0, or
# returns 1 when there is none (bash then reads the text as a subshell).
guard_arith_end() {
  local p=$1 depth=0 c nx q
  while ((p < G_LEN)); do
    c=${G_STR:p:1}
    nx=${G_STR:p+1:1}
    case $c in
      \\)
        p=$((p + 2))
        continue
        ;;
      "'")
        q=$((p + 1))
        while ((q < G_LEN)) && [[ ${G_STR:q:1} != "'" ]]; do
          q=$((q + 1))
        done
        p=$((q + 1))
        continue
        ;;
      '"')
        q=$((p + 1))
        while ((q < G_LEN)) && [[ ${G_STR:q:1} != '"' ]]; do
          if [[ ${G_STR:q:1} == \\ ]]; then
            q=$((q + 1))
          fi
          q=$((q + 1))
        done
        p=$((q + 1))
        continue
        ;;
      '(')
        depth=$((depth + 1))
        ;;
      ')')
        if ((depth == 0)); then
          if [[ $nx == ')' ]]; then
            ARITH_END=$p
            return 0
          fi
        else
          depth=$((depth - 1))
        fi
        ;;
    esac
    p=$((p + 1))
  done
  return 1
}

# guard_arith_body <end> — scans arithmetic text up to <end> (the first ) of
# the closing )). Only the substitutions inside it run; the rest is numbers and
# operators. Leaves G_POS just past the closing ))
guard_arith_body() {
  local end=$1 c
  while ((G_POS < end)); do
    c=${G_STR:G_POS:1}
    if [[ $c == \\ ]]; then
      G_POS=$((G_POS + 2))
    elif [[ $c == '`' || ( $c == '$' && ${G_STR:G_POS+1:1} == '(' ) ]]; then
      guard_substitute
    else
      G_POS=$((G_POS + 1))
    fi
  done
  G_POS=$((end + 2))
}

# guard_substitute — G_POS is at a $( , $(( or ` that the shell would run. The
# substitution is checked as a command of its own; G_POS is left just past it.
# The caller appends the bare $ to its word. A cd inside the substitution does
# not reach the caller.
guard_substitute() {
  local c=${G_STR:G_POS:1} n3=${G_STR:G_POS+2:1} saved j len end
  if [[ $c == '`' ]]; then
    j=$((G_POS + 1))
    while ((j < G_LEN)) && [[ ${G_STR:j:1} != '`' ]]; do
      if [[ ${G_STR:j:1} == \\ ]]; then
        j=$((j + 1))
      fi
      j=$((j + 1))
    done
    if ((j >= G_LEN)); then
      guard_deny "unterminated backtick substitution"
    fi
    len=$((j - G_POS - 1))
    saved=$GUARD_BASE
    guard_verdict "${G_STR:G_POS+1:len}"
    GUARD_BASE=$saved
    G_POS=$((j + 1))
    return 0
  fi
  if [[ $n3 == '(' ]] && guard_arith_end $((G_POS + 3)); then
    end=$ARITH_END
    G_POS=$((G_POS + 3))
    guard_arith_body "$end"
    return 0
  fi
  saved=$GUARD_BASE
  G_POS=$((G_POS + 2))
  guard_scan sub
  GUARD_BASE=$saved
  G_POS=$((G_POS + 1))
  return 0
}

# guard_scan <top|sub> — scans from G_POS. In sub mode it stops at the ) that
# closes a $( with G_POS left on that ); in top mode it runs to the end.
guard_scan() {
  local mode=$1 c nx top="" j len n
  local -a GW=() G_HD=() G_STK=()
  local G_BUF="" G_CS="" G_HAVE=0 G_SKIP=0 G_HDWAIT=0 G_HDRAW=0
  local G_LEAD=1 G_QUOTED=0 G_COND=0 G_CPAR=0
  while ((G_POS < G_LEN)); do
    c=${G_STR:G_POS:1}
    nx=${G_STR:G_POS+1:1}
    _guard_top
    # [[ at a command start (not a case pattern) opens a conditional expression.
    if ((G_COND == 0 && G_LEAD && G_HAVE == 0)) && [[ $top != h && $top != p ]] \
      && [[ ${G_STR:G_POS:2} == '[[' ]] && guard_is_delim $((G_POS + 2)); then
      G_COND=1
      G_CPAR=0
      G_LEAD=0
      G_POS=$((G_POS + 2))
      continue
    fi
    if ((G_COND)); then
      # Inside [[ ... ]] the words are operands. ]] ends it, a ; & | or # ends it
      # early (the shell rejects those inside), and any other operator is skipped.
      if [[ ${G_STR:G_POS:2} == ']]' ]] && ((G_HAVE == 0)) && guard_is_delim $((G_POS + 2)); then
        G_COND=0
        G_POS=$((G_POS + 2))
        continue
      fi
      case $c in
        '&'|'|')
          if [[ $nx == "$c" ]]; then
            _guard_flush
            G_POS=$((G_POS + 2))
            continue
          fi
          G_COND=0
          ;;
        '#')
          if ((G_HAVE == 0)); then
            G_COND=0
          fi
          ;;
        ';') G_COND=0 ;;
        $'\n')
          # A newline is blank inside [[ ]], except that a pending heredoc body
          # starts after it, so the normal newline handling must run.
          if ((${#G_HD[@]})); then
            G_COND=0
          else
            _guard_flush
            G_POS=$((G_POS + 1))
            continue
          fi
          ;;
        '(')
          _guard_flush
          G_CPAR=$((G_CPAR + 1))
          G_POS=$((G_POS + 1))
          continue
          ;;
        ')')
          if ((G_CPAR == 0)); then
            G_COND=0
          else
            _guard_flush
            G_CPAR=$((G_CPAR - 1))
            G_POS=$((G_POS + 1))
            continue
          fi
          ;;
        '<'|'>')
          # <(...) and >(...) run a command, so they leave the conditional.
          if [[ $nx == '(' ]]; then
            G_COND=0
          else
            _guard_flush
            G_POS=$((G_POS + 1))
            continue
          fi
          ;;
      esac
    fi
    case $c in
      ' '|$'\t')
        _guard_flush
        G_POS=$((G_POS + 1))
        ;;
      $'\n')
        _guard_sep
        G_POS=$((G_POS + 1))
        if ((${#G_HD[@]})); then
          _guard_bodies
        fi
        ;;
      ';')
        _guard_sep
        _guard_top
        if [[ $top == b && ( $nx == ';' || $nx == '&' ) ]]; then
          G_POS=$((G_POS + 2))
          if [[ $nx == ';' && ${G_STR:G_POS:1} == '&' ]]; then
            G_POS=$((G_POS + 1))
          fi
          G_CS=${G_CS%?}p
        else
          G_POS=$((G_POS + 1))
        fi
        ;;
      '&')
        if [[ $nx == '>' ]]; then
          _guard_redir
        else
          _guard_sep
          G_POS=$((G_POS + 1))
        fi
        ;;
      '<'|'>')
        _guard_redir
        ;;
      '|')
        _guard_top
        if [[ $top == p ]]; then
          _guard_flush
        else
          _guard_sep
        fi
        G_POS=$((G_POS + 1))
        ;;
      '(')
        _guard_top
        if [[ $top == h || $top == p ]]; then
          _guard_flush
        elif [[ $nx == '(' ]] && ((G_LEAD && G_HAVE == 0)) && guard_arith_end $((G_POS + 2)); then
          local end=$ARITH_END
          G_POS=$((G_POS + 2))
          guard_arith_body "$end"
          continue
        else
          _guard_sep
          G_STK+=("$GUARD_BASE")
        fi
        G_POS=$((G_POS + 1))
        ;;
      ')')
        _guard_top
        if [[ $top == p ]]; then
          _guard_flush
          G_CS=${G_CS%?}b
        elif ((${#G_STK[@]})); then
          _guard_sep
          # bash rejects a word right after a subshell's ), so it is no command start.
          G_LEAD=0
          n=${#G_STK[@]}
          GUARD_BASE=${G_STK[n - 1]}
          unset "G_STK[$((n - 1))]"
        elif [[ $mode == sub ]]; then
          _guard_sep
          return 0
        else
          _guard_sep
        fi
        G_POS=$((G_POS + 1))
        ;;
      "'")
        j=$((G_POS + 1))
        while ((j < G_LEN)) && [[ ${G_STR:j:1} != "'" ]]; do
          j=$((j + 1))
        done
        if ((j >= G_LEN)); then
          guard_deny "unterminated quote"
        fi
        len=$((j - G_POS - 1))
        G_BUF+=${G_STR:G_POS+1:len}
        G_HAVE=1
        G_QUOTED=1
        if ((G_HDWAIT)); then
          G_HDRAW=1
        fi
        G_POS=$((j + 1))
        ;;
      '"')
        G_HAVE=1
        G_QUOTED=1
        if ((G_HDWAIT)); then
          G_HDRAW=1
        fi
        G_POS=$((G_POS + 1))
        while ((G_POS < G_LEN)) && [[ ${G_STR:G_POS:1} != '"' ]]; do
          c=${G_STR:G_POS:1}
          nx=${G_STR:G_POS+1:1}
          if [[ $c == \\ ]]; then
            case $nx in
              '"'|\\|'$'|'`')
                G_BUF+=$nx
                G_POS=$((G_POS + 2))
                continue
                ;;
            esac
          elif [[ $c == '`' || ( $c == '$' && $nx == '(' ) ]]; then
            guard_substitute
            G_BUF+='$'
            continue
          fi
          G_BUF+=$c
          G_POS=$((G_POS + 1))
        done
        if ((G_POS >= G_LEN)); then
          guard_deny "unterminated quote"
        fi
        G_POS=$((G_POS + 1))
        ;;
      \\)
        nx=${G_STR:G_POS+1:1}
        if [[ -z $nx ]]; then
          G_POS=$((G_POS + 1))
        elif [[ $nx == $'\n' ]]; then
          G_POS=$((G_POS + 2))
        else
          G_BUF+=$nx
          G_HAVE=1
          G_QUOTED=1
          if ((G_HDWAIT)); then
            G_HDRAW=1
          fi
          G_POS=$((G_POS + 2))
        fi
        ;;
      '#')
        if ((G_HAVE)); then
          G_BUF+=$c
          G_POS=$((G_POS + 1))
        else
          j=$G_POS
          while ((j < G_LEN)) && [[ ${G_STR:j:1} != $'\n' ]]; do
            j=$((j + 1))
          done
          G_POS=$j
        fi
        ;;
      '$')
        if [[ $nx == '(' ]]; then
          guard_substitute
          G_BUF+='$'
          G_HAVE=1
        elif [[ $nx == "'" ]]; then
          # $'...' is ANSI-C quoting: a backslash escapes the next character.
          j=$((G_POS + 2))
          while ((j < G_LEN)) && [[ ${G_STR:j:1} != "'" ]]; do
            if [[ ${G_STR:j:1} == \\ ]]; then
              j=$((j + 1))
            fi
            j=$((j + 1))
          done
          if ((j >= G_LEN)); then
            guard_deny "unterminated quote"
          fi
          len=$((j - G_POS - 2))
          G_BUF+='$'"${G_STR:G_POS+2:len}"
          G_HAVE=1
          G_QUOTED=1
          G_POS=$((j + 1))
        elif [[ $nx == '"' ]]; then
          # $"..." is a locale-translated double-quoted string: quote it as one.
          G_POS=$((G_POS + 1))
        else
          G_BUF+='$'
          G_HAVE=1
          G_POS=$((G_POS + 1))
        fi
        ;;
      '`')
        guard_substitute
        G_BUF+='$'
        G_HAVE=1
        ;;
      *)
        G_BUF+=$c
        G_HAVE=1
        G_POS=$((G_POS + 1))
        ;;
    esac
  done
  if ((G_COND)); then
    guard_deny "unterminated [[ conditional expression"
  fi
  if [[ $mode == sub ]]; then
    guard_deny "unterminated \$( substitution"
  fi
  _guard_sep
  return 0
}

# guard_verdict <command> — scans one command text. Returns only when every
# simple command in it passes; a denial exits 2.
guard_verdict() {
  local G_STR=$1 G_LEN=${#1} G_POS=0
  guard_scan top
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

# guard_check_git <argument>... — the words after `git`. Global options come
# before the subcommand. -C and -c take the next word. A long option written
# without `=` takes the next word as its value, unless git documents it as a
# bare flag. An unknown long option whose next word is a denied subcommand is
# itself denied: the word may be the subcommand, and the guard cannot tell.
guard_check_git() {
  local a sub=""
  while (($#)); do
    a=$1
    shift
    case $a in
      -C|-c)
        if (($#)); then
          shift
        fi
        ;;
      --*=*) ;;
      --no-pager|--paginate|--bare|--no-replace-objects|--literal-pathspecs) ;;
      --glob-pathspecs|--noglob-pathspecs|--icase-pathspecs|--no-optional-locks) ;;
      --no-advice|--version|--help|--html-path|--man-path|--info-path) ;;
      --*)
        if (($#)); then
          case $1 in
            reset|clean|push|stash)
              guard_deny "git option $a is followed by $1, which may be the subcommand"
              ;;
          esac
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
    stash)
      case ${1:-} in
        list|show) ;;
        *) guard_deny "git stash drops other agents' work in the shared tree; only 'git stash list' and 'git stash show' are allowed" ;;
      esac
      ;;
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

# guard_scratch_roots — fills GUARD_TMP with the roots an rm may clear strictly
# inside: $TMPDIR, /tmp and /private/tmp, each in its lexical and physical form
# (on macOS /tmp is /private/tmp, and $TMPDIR sits under /var, a link to
# /private/var). A root that is the repository root or holds it is dropped:
# clearing it would clear the repository too. "/" is never a root.
# Then each repository-root .scratch or .work that is a symlink adds the
# physical directory it names as a root, and is recorded in GUARD_LINK so that
# a path through the link is judged by that directory.
guard_scratch_roots() {
  local cand form phys name
  GUARD_TMP=()
  GUARD_LINK=()
  GUARD_LINK_TO=()
  for cand in "${TMPDIR:-}" /tmp /private/tmp; do
    [[ -n $cand ]] || continue
    guard_resolve "" "$cand" || continue
    phys=$(cd -- "$RESOLVED" 2>/dev/null && pwd -P) || phys=""
    for form in "$RESOLVED" "$phys"; do
      [[ -n $form && $form != / ]] || continue
      case $GUARD_ROOT in
        "$form"|"$form"/*) continue ;;
      esac
      GUARD_TMP+=("$form")
    done
  done
  for name in .scratch .work; do
    [[ -L $GUARD_ROOT/$name ]] || continue
    phys=$(cd -P -- "$GUARD_ROOT/$name" 2>/dev/null && pwd -P) || continue
    [[ -n $phys && $phys != / ]] || continue
    case $GUARD_ROOT in
      "$phys"|"$phys"/*) continue ;;
    esac
    GUARD_TMP+=("$phys")
    GUARD_LINK+=("$GUARD_ROOT/$name")
    GUARD_LINK_TO+=("$phys")
  done
}

# guard_check_rm_target <path> — the target must resolve inside the repository
# or strictly inside a scratch root (GUARD_TMP).
guard_check_rm_target() {
  local t=$1 rest comp k top phys last parent
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
  # A repository-root scratch link is followed: rm clears the directory it names.
  k=0
  while ((k < ${#GUARD_LINK[@]})); do
    case $RESOLVED in
      "${GUARD_LINK[k]}"|"${GUARD_LINK[k]}"/*)
        RESOLVED=${GUARD_LINK_TO[k]}${RESOLVED#"${GUARD_LINK[k]}"}
        break
        ;;
    esac
    k=$((k + 1))
  done
  # Any other repository-root symlink is not followed: one that leaves the
  # repository (.deps, the shared dependency cache) denies the target.
  case $RESOLVED in
    "$GUARD_ROOT"/*)
      top=${RESOLVED#"$GUARD_ROOT"/}
      top=${top%%/*}
      if [[ -L $GUARD_ROOT/$top ]]; then
        phys=$(cd -P -- "$GUARD_ROOT/$top" 2>/dev/null && pwd -P) || phys=""
        case $phys in
          "$GUARD_ROOT"|"$GUARD_ROOT"/*) ;;
          *) guard_deny "rm target runs through repository-root symlink $top, which does not resolve inside the repository: $(printf '%q' "$t")" ;;
        esac
      fi
      ;;
  esac
  # A last component of only * and ? (with a *) matches every entry of its
  # parent. Directly inside a protected directory, that clears the directory.
  # Partial patterns such as /tmp/agy-* are not denied.
  last=${RESOLVED##*/}
  case $last in
    *'*'*)
      case $last in
        *[!*?]*) ;;
        *)
          parent=${RESOLVED%/*}
          [[ -n $parent ]] || parent=/
          if [[ $parent == "$GUARD_ROOT" || $parent == "$GUARD_ROOT/.scratch" || $parent == "$GUARD_ROOT/.work" ]]; then
            guard_deny "rm $(printf '%q' "$t") would remove every entry of the repository directory $parent"
          fi
          k=0
          while ((k < ${#GUARD_TMP[@]})); do
            if [[ $parent == "${GUARD_TMP[k]}" ]]; then
              guard_deny "rm $(printf '%q' "$t") would remove every entry of the scratch root $parent"
            fi
            k=$((k + 1))
          done
          ;;
      esac
      ;;
  esac
  case $RESOLVED in
    "$GUARD_ROOT"|"$GUARD_ROOT/.git"|"$GUARD_ROOT/.git"/*)
      guard_deny "rm would remove the repository root or its .git: $(printf '%q' "$t")"
      ;;
    "$GUARD_ROOT"/*) ;;
    *)
      k=0
      while ((k < ${#GUARD_TMP[@]})); do
        case $RESOLVED in
          "${GUARD_TMP[k]}"/*) return 0 ;;
        esac
        k=$((k + 1))
      done
      guard_deny "rm target is outside the repository and the scratch roots: $(printf '%q' "$t") resolves to $RESOLVED"
      ;;
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

# guard_check_text <text> — command text that a wrapper runs (env -S). It is
# scanned as shell text in its own scope: a cd inside it stays inside.
guard_check_text() {
  local saved=$GUARD_BASE
  guard_verdict "$1"
  GUARD_BASE=$saved
}

# guard_check_env <argument>... — the words after `env`, or after `cmake -E env`.
# Options are skipped. The text of -S / --split-string is checked as a command.
# The first word that is not an option starts the wrapped command, which
# guard_check_simple checks with its assignments. -u and -C take a value.
guard_check_env() {
  local a short c
  while (($#)); do
    a=$1
    case $a in
      --) shift; break ;;
      --split-string=*)
        shift
        guard_check_text "${a#--split-string=}"
        ;;
      --split-string)
        shift
        if (($#)); then
          guard_check_text "$1"
          shift
        fi
        ;;
      --unset|--chdir)
        shift
        if (($#)); then
          shift
        fi
        ;;
      --*|-) shift ;;
      -*)
        shift
        short=${a#-}
        while [[ -n $short ]]; do
          c=${short:0:1}
          short=${short:1}
          case $c in
            S)
              if [[ -z $short ]] && (($#)); then
                short=$1
                shift
              fi
              guard_check_text "$short"
              break
              ;;
            u|C)
              if [[ -z $short ]] && (($#)); then
                shift
              fi
              break
              ;;
          esac
        done
        ;;
      *) break ;;
    esac
  done
  guard_check_simple "$@"
}

# guard_check_simple <word>... — one simple command, words already unquoted.
guard_check_simple() {
  local -a w
  local i=0 m word name k
  w=("$@")
  m=${#w[@]}
  while ((i < m)); do
    word=${w[i]}
    name=${word##*/}
    if guard_is_assignment "$word"; then
      guard_check_golden "$word"
      i=$((i + 1))
    elif [[ $name == env ]]; then
      # The wrapped command and its assignments are checked by guard_check_env.
      guard_check_env "${w[@]:i+1}"
      return 0
    elif [[ $name == cmake && ${w[i+1]-} == -E && ${w[i+2]-} == env ]]; then
      guard_check_env "${w[@]:i+3}"
      return 0
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
  guard_scratch_roots
  guard_verdict "$cmd"
}

main "$@"
