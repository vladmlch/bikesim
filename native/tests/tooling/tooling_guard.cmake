# tooling_guard — contract for tools/guard.sh: every deny and allow example from
# its header, stdin input, a usage error, and --cwd / working-directory
# resolution for rm. The guard only parses; no command runs. Also covers the
# agy and Claude PreToolUse adapters (payload shapes, fail-open rule, verdict
# format) and .agents/hooks.json, which is run as written from .agents/.
# Stays in the ordinary (non-slow) CTest set.
#
# Commands are CMake bracket arguments ([==[ ... ]==]) so $, quotes, backticks
# and semicolons reach the guard byte for byte.
#
# Required -D inputs: REPO_ROOT, SCRATCH_DIR.
foreach(_required REPO_ROOT SCRATCH_DIR)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "tooling_guard requires -D${_required}")
  endif()
endforeach()
file(REMOVE_RECURSE "${SCRATCH_DIR}")
file(MAKE_DIRECTORY "${SCRATCH_DIR}")
set(_failures "")
set(_guard "${REPO_ROOT}/tools/guard.sh")

# guard_case(<name> <allow|deny> <command> [CWD <dir>] [WD <dir>] [ENV <k=v>] [STDIN])
#   CWD  passes --cwd <dir>            (default: REPO_ROOT)
#   WD   runs the guard in <dir>, no --cwd
#   ENV  sets a variable in the guard's environment
#   STDIN feeds <command> on stdin instead of as the argument
# allow: exit 0 with no output. deny: exit 2 and exactly one stderr line
# starting "guard: denied: ".
function(guard_case name expect command)
  set(_cwd_args "--cwd" "${REPO_ROOT}")
  set(_wd "${REPO_ROOT}")
  set(_envs "")
  set(_stdin OFF)
  set(_i 3)
  while(_i LESS ARGC)
    set(_key "${ARGV${_i}}")
    math(EXPR _i "${_i} + 1")
    if("${_key}" STREQUAL "STDIN")
      set(_stdin ON)
    elseif("${_key}" STREQUAL "CWD")
      set(_cwd_args "--cwd" "${ARGV${_i}}")
      math(EXPR _i "${_i} + 1")
    elseif("${_key}" STREQUAL "WD")
      set(_cwd_args "")
      set(_wd "${ARGV${_i}}")
      math(EXPR _i "${_i} + 1")
    elseif("${_key}" STREQUAL "ENV")
      list(APPEND _envs "${ARGV${_i}}")
      math(EXPR _i "${_i} + 1")
    else()
      message(FATAL_ERROR "guard_case ${name}: unknown option ${_key}")
    endif()
  endwhile()
  set(_base_env "${CMAKE_COMMAND}" -E env --unset=BIKE_GUARD_ALLOW_GOLDENS ${_envs})
  if(_stdin)
    file(WRITE "${SCRATCH_DIR}/stdin_${name}.txt" "${command}")
    execute_process(COMMAND ${_base_env} bash "${_guard}" ${_cwd_args}
      WORKING_DIRECTORY "${_wd}"
      INPUT_FILE "${SCRATCH_DIR}/stdin_${name}.txt"
      RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
  else()
    execute_process(COMMAND ${_base_env} bash "${_guard}" ${_cwd_args} "${command}"
      WORKING_DIRECTORY "${_wd}"
      RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
  endif()
  string(STRIP "${_err}" _errs)
  string(FIND "${_errs}" "\n" _nl)
  string(FIND "${_errs}" "guard: denied: " _pfx)
  if(expect STREQUAL "allow")
    if(NOT _rc EQUAL 0 OR NOT "${_out}${_err}" STREQUAL "")
      set(_failures "${_failures}\n  ${name}: expected allow, got rc=${_rc}, output '${_out}${_err}'" PARENT_SCOPE)
    endif()
  else()
    if(NOT _rc EQUAL 2 OR NOT _pfx EQUAL 0 OR NOT _nl EQUAL -1 OR NOT "${_out}" STREQUAL "")
      set(_failures "${_failures}\n  ${name}: expected deny (rc 2, one 'guard: denied:' line), got rc=${_rc}, stdout '${_out}', stderr '${_err}'" PARENT_SCOPE)
    endif()
  endif()
endfunction()

# --- deny: git -----------------------------------------------------------------
guard_case(deny_reset_hard deny [==[git reset --hard HEAD~1]==])
guard_case(deny_reset_hard_after_and deny [==[git status && git reset --hard]==])
guard_case(deny_reset_hard_git_c deny [==[git -c core.x=1 reset --hard]==])
guard_case(deny_clean_dry_run deny [==[git clean -n]==])
guard_case(deny_clean_fdx deny [==[git clean -fdx]==])
guard_case(deny_push deny [==[git push]==])
guard_case(deny_push_dry_run deny [==[git push --dry-run origin main]==])
guard_case(deny_push_after_semicolon deny [==[make; git push]==])
guard_case(deny_push_after_newline deny [==[make
git push]==])
guard_case(deny_push_after_or deny [==[false || git push]==])
guard_case(deny_push_after_pipe deny [==[echo x | git push]==])
guard_case(deny_push_after_background deny [==[make & git push]==])
guard_case(deny_push_in_parens deny [==[(git push)]==])
guard_case(deny_push_in_brace_group deny [==[{ git push; }]==])
guard_case(deny_push_git_c deny [==[git -C native push]==])
guard_case(deny_push_absolute_path deny [==[/usr/bin/git push]==])
guard_case(deny_push_env_prefix deny [==[env GIT_TRACE=1 git push]==])
guard_case(deny_push_command_prefix deny [==[command git push]==])
guard_case(deny_push_assignment_prefix deny [==[GIT_X=1 git push]==])
guard_case(deny_push_quoted_command_word deny [==["git" push]==])
guard_case(deny_push_after_redirect deny [==[git push > out.txt]==])
guard_case(deny_push_after_fd_redirect deny [==[git push 2>&1]==])
guard_case(deny_push_in_while_body deny [==[while true; do git push; done]==])
guard_case(deny_stash deny [==[git stash]==])
guard_case(deny_stash_pop deny [==[git stash pop]==])
guard_case(deny_stash_git_c deny [==[git -C native stash list]==])
guard_case(deny_stash_in_command_substitution deny [==[echo "$(git stash)"]==])
guard_case(deny_push_in_backticks deny [==[echo `git push`]==])
guard_case(deny_push_in_double_quoted_substitution deny [==[git commit -m "$(git push)"]==])
guard_case(deny_push_variable_subcommand deny [==[git $SUB]==])
guard_case(deny_push_substituted_command_name deny [==[$(echo git) push]==])

# --- deny: golden re-record ----------------------------------------------------
guard_case(deny_golden_prefix deny [==[BIKE_GOLDEN_RECORD=1 ./run]==])
guard_case(deny_golden_export deny [==[export BIKE_GOLDEN_RECORD=1]==])
guard_case(deny_golden_env deny [==[env BIKE_GOLDEN_RECORD=1 ./run]==])
guard_case(deny_golden_env_option deny [==[env -i BIKE_GOLDEN_RECORD=1 ./run]==])
guard_case(deny_golden_quoted_value deny [==[BIKE_GOLDEN_RECORD="1" ./run]==])
guard_case(deny_golden_after_and deny [==[make && BIKE_GOLDEN_RECORD=1 ./run]==])
guard_case(deny_golden_in_subshell deny [==[(BIKE_GOLDEN_RECORD=1 ./run)]==])
guard_case(deny_misspelt_prefix deny [==[BIKE_RECORD_GOLDENS=1 ./run]==])
guard_case(deny_misspelt_export deny [==[export BIKE_RECORD_GOLDENS=1]==])
guard_case(deny_misspelt_env deny [==[env BIKE_RECORD_GOLDENS=1 ./run]==])
guard_case(deny_golden_guard_env_zero deny [==[BIKE_GOLDEN_RECORD=1 ./run]==] ENV BIKE_GUARD_ALLOW_GOLDENS=0)

# --- deny: rm outside the repository ------------------------------------------
guard_case(deny_rm_root deny [==[rm -rf /]==])
guard_case(deny_rm_fr_root deny [==[rm -fr /]==])
guard_case(deny_rm_separate_flags deny [==[rm -r -f /etc/x]==])
guard_case(deny_rm_long_flags deny [==[rm --recursive --force /etc/x]==])
guard_case(deny_rm_tilde deny [==[rm -rf ~]==])
guard_case(deny_rm_home_variable deny [==[rm -rf $HOME]==])
guard_case(deny_rm_home_variable_quoted deny [==[rm -rf "$HOME/x"]==])
guard_case(deny_rm_home_braced deny [==[rm -rf ${HOME}/x]==])
guard_case(deny_rm_parent deny [==[rm -rf ..]==])
guard_case(deny_rm_parent_relative deny [==[rm -rf ../x]==])
guard_case(deny_rm_glob_root deny [==[rm -rf /*]==])
guard_case(deny_rm_glob_escape deny [==[rm -rf build/*/../../..]==])
guard_case(deny_rm_unexpanded_variable deny [==[rm -rf $DIR]==])
guard_case(deny_rm_brace_expansion deny [==[rm -rf {/,build}]==])
guard_case(deny_rm_empty_target deny [==[rm -rf ""]==])
guard_case(deny_rm_repository_root deny [==[rm -rf .]==])
guard_case(deny_rm_dot_git deny [==[rm -rf .git]==])
guard_case(deny_rm_dot_git_objects deny [==[rm -rf .git/objects]==])
guard_case(deny_rm_hidden_glob deny [==[rm -rf .*]==])
guard_case(deny_rm_tilde_user deny [==[rm -rf ~root/x]==])
guard_case(deny_rm_mixed_targets deny [==[rm -rf build /etc/x]==])
guard_case(deny_rm_substituted_target deny [==[rm -rf $(pwd)]==])
guard_case(deny_rm_cwd_outside_relative deny [==[rm -rf build]==] CWD /)
guard_case(deny_rm_wd_outside_relative deny [==[rm -rf build]==] WD /)
guard_case(deny_rm_wd_native_parent deny [==[rm -rf ../../x]==] WD ${REPO_ROOT}/native)
guard_case(deny_rm_cd_outside deny [==[cd / && rm -rf build]==])
guard_case(deny_rm_cd_semicolon deny [==[cd / ; rm -rf build]==])
guard_case(deny_rm_cd_then_parent deny [==[cd native && rm -rf ../../x]==])
guard_case(deny_rm_cd_unknown deny [==[cd "$X" && rm -rf build]==])
guard_case(deny_rm_cd_dash deny [==[cd - && rm -rf build]==])
guard_case(deny_rm_popd deny [==[popd && rm -rf build]==])

# --- scratch roots: $TMPDIR, /tmp and /private/tmp ----------------------------
# A target strictly inside a scratch root is allowed; the root itself, a
# lexical escape out of it, and a $ expansion stay denied. TMPDIR is set per
# case so the results do not depend on the caller's environment.
guard_case(allow_rm_tmp_child allow [==[rm -rf /tmp/x]==])
guard_case(allow_rm_tmp_child_trailing_slash allow [==[rm -rf /tmp/x/]==])
guard_case(allow_rm_tmp_nested allow [==[rm -rf /tmp/a/b/../c]==])
guard_case(allow_rm_private_tmp_child allow [==[rm -rf /private/tmp/x]==])
guard_case(allow_rm_tmpdir_child allow [==[rm -rf /var/folders/zz/T/x]==] ENV TMPDIR=/var/folders/zz/T/)
guard_case(deny_rm_tmp_root deny [==[rm -rf /tmp]==])
guard_case(deny_rm_tmp_root_trailing_slash deny [==[rm -rf /tmp/]==])
guard_case(deny_rm_tmp_root_dot deny [==[rm -rf /tmp/.]==])
guard_case(deny_rm_tmp_parent_escape deny [==[rm -rf /tmp/../etc]==])
guard_case(deny_rm_tmp_sibling_prefix deny [==[rm -rf /tmpx/y]==])
guard_case(deny_rm_private_tmp_root deny [==[rm -rf /private/tmp]==])
guard_case(deny_rm_tmpdir_root deny [==[rm -rf /var/folders/zz/T]==] ENV TMPDIR=/var/folders/zz/T/)
guard_case(deny_rm_tmpdir_variable deny [==[rm -rf "$TMPDIR"]==])
guard_case(deny_rm_tmp_and_outside deny [==[rm -rf /tmp/x /etc/y]==])
# A TMPDIR that holds the repository is not a scratch root: clearing it would
# clear the checkout's siblings, and the repository itself stays protected.
get_filename_component(_repo_parent "${REPO_ROOT}" DIRECTORY)
guard_case(deny_rm_tmpdir_holds_repo deny "rm -rf ${_repo_parent}/sibling" ENV TMPDIR=${_repo_parent})

# --- allow: quoted text, comments, heredocs, safe rm ---------------------------
guard_case(allow_commit_message_push allow [==[git commit -m "do not git push"]==])
guard_case(allow_commit_message_reset allow [==[git commit -m "never git reset --hard"]==])
guard_case(allow_log_grep_reset allow [==[git log --grep reset]==])
guard_case(allow_log_grep_quoted_push allow [==[git log --grep "git push"]==])
guard_case(allow_echo_double_stash allow [==[echo "git stash"]==])
guard_case(allow_echo_single_push allow [==[echo 'git push']==])
guard_case(allow_echo_unquoted_words allow [==[echo git push]==])
guard_case(allow_echo_escaped_word allow [==[echo \git push]==])
guard_case(allow_git_status allow [==[git status]==])
guard_case(allow_git_reset_soft allow [==[git reset --soft HEAD~1]==])
guard_case(allow_git_reset_mixed allow [==[git reset HEAD~1]==])
guard_case(allow_git_pushover allow [==[git pushover]==])
guard_case(allow_git_c_status allow [==[git -C native status]==])
guard_case(allow_git_c_log_grep allow [==[git -C native log --grep reset]==])
guard_case(allow_comment_push allow [==[echo hi # git push]==])
guard_case(allow_grep_text allow [==[grep -r "git push" tools]==])
guard_case(allow_double_quoted_separator allow [==[echo "a; git push"]==])
guard_case(allow_empty_command allow [==[]==])
guard_case(allow_substitution_commit_message allow [==[git commit -m "$(cat msg)"]==])
guard_case(allow_substitution_date allow [==[echo "$(date)"]==])
guard_case(allow_arithmetic_shift allow [==[echo $((1 << 2))]==])
guard_case(allow_heredoc_body_text allow [==[cat > NOTES.md <<'EOF'
git push is banned
EOF]==])
guard_case(allow_heredoc_apostrophe_body allow [==[cat <<'EOF'
don't do it
EOF
git status]==])
guard_case(allow_rm_build_tmp allow [==[rm -rf build/tmp]==])
guard_case(allow_rm_dot_slash allow [==[rm -rf ./build/tmp/]==])
guard_case(allow_rm_glob_inside allow [==[rm -rf build/*]==])
guard_case(allow_rm_parent_inside allow [==[rm -rf native/../build/tmp]==])
guard_case(allow_rm_recursive_only allow [==[rm -r /tmp/x]==])
guard_case(allow_rm_force_only allow [==[rm -f /tmp/x]==])
guard_case(allow_rm_then_status allow [==[rm -rf build/tmp; git status]==])
guard_case(allow_rm_redirected allow [==[rm -rf build/tmp 2>/dev/null]==])
guard_case(allow_rm_quoted_text allow [==[echo "rm -rf /"]==])
guard_case(allow_rm_cd_inside allow [==[cd native && rm -rf ../build/tmp]==])
guard_case(allow_rm_cwd_native allow [==[rm -rf tmp]==] CWD ${REPO_ROOT}/native)
guard_case(allow_rm_cwd_native_parent allow [==[rm -rf ../build/tmp]==] CWD ${REPO_ROOT}/native)
guard_case(allow_rm_wd_native_parent allow [==[rm -rf ../build/tmp]==] WD ${REPO_ROOT}/native)
guard_case(allow_golden_zero allow [==[BIKE_GOLDEN_RECORD=0 ./run]==])
guard_case(allow_golden_text allow [==[echo BIKE_GOLDEN_RECORD=1]==])
guard_case(allow_golden_with_guard_env allow [==[BIKE_GOLDEN_RECORD=1 ./run]==] ENV BIKE_GUARD_ALLOW_GOLDENS=1)
guard_case(allow_misspelt_with_guard_env allow [==[export BIKE_RECORD_GOLDENS=1]==] ENV BIKE_GUARD_ALLOW_GOLDENS=1)

# --- stdin input ---------------------------------------------------------------
guard_case(stdin_deny_push deny [==[make
git push]==] STDIN)
guard_case(stdin_allow_status allow [==[git status]==] STDIN)

# --- usage errors fail closed --------------------------------------------------
execute_process(COMMAND "${CMAKE_COMMAND}" -E env --unset=BIKE_GUARD_ALLOW_GOLDENS
  bash "${_guard}" --cwd "${SCRATCH_DIR}/does-not-exist" "git status"
  RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
if(NOT _rc EQUAL 2)
  set(_failures "${_failures}\n  usage_missing_cwd: expected exit 2, got ${_rc}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E env --unset=BIKE_GUARD_ALLOW_GOLDENS
  bash "${_guard}" --bogus "git status"
  RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
if(NOT _rc EQUAL 2)
  set(_failures "${_failures}\n  usage_unknown_option: expected exit 2, got ${_rc}")
endif()

# --- agy and Claude PreToolUse adapters ---------------------------------------
# hook_run(<name> <script> <payload>) runs <script> from REPO_ROOT with <payload>
# on stdin; @REPO@ in the payload becomes REPO_ROOT. Sets <name>_rc, <name>_out
# and <name>_err in the caller, each stripped of surrounding whitespace.
function(hook_run name script payload)
  string(REPLACE "@REPO@" "${REPO_ROOT}" _payload "${payload}")
  file(WRITE "${SCRATCH_DIR}/hook_${name}.in" "${_payload}")
  execute_process(COMMAND "${CMAKE_COMMAND}" -E env --unset=BIKE_GUARD_ALLOW_GOLDENS
    bash "${script}"
    WORKING_DIRECTORY "${REPO_ROOT}"
    INPUT_FILE "${SCRATCH_DIR}/hook_${name}.in"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
  string(STRIP "${_out}" _out)
  string(STRIP "${_err}" _err)
  set(${name}_rc "${_rc}" PARENT_SCOPE)
  set(${name}_out "${_out}" PARENT_SCOPE)
  set(${name}_err "${_err}" PARENT_SCOPE)
endfunction()

# agy_payload(<out> <tool> <command> <last-user-input>) — an agy PreToolUse
# payload in the verified shape (see tools/agy/pretool-guard.sh).
function(agy_payload out tool command last)
  set(${out} "{\"cwd\":\"@REPO@\",\"hookEventName\":\"PreToolUse\",\"toolCall\":{\"args\":{\"CommandLine\":\"${command}\",\"toolSummary\":\"probe\",\"WaitMsBeforeAsync\":5000},\"id\":\"call_1\",\"name\":\"${tool}\"},\"workspacePaths\":[\"@REPO@\"],\"lastUserInput\":\"${last}\",\"conversationId\":\"test\"}" PARENT_SCOPE)
endfunction()

# agy_check(<name> <allow|deny|open> [<needle>]) — checks an agy hook result.
#   allow: exit 0, stdout decision allow, empty stderr
#   deny:  exit 0, stdout decision deny, reason contains <needle>, empty stderr
#   open:  fail-open: exit 0, decision allow, stderr notes it with <needle>
function(agy_check name expect)
  set(_bad "")
  set(_needle "${ARGV2}")
  set(_out "${${name}_out}")
  set(_err "${${name}_err}")
  if(NOT ${name}_rc EQUAL 0)
    string(APPEND _bad " exit=${${name}_rc}")
  endif()
  if(expect STREQUAL "open")
    set(_want allow)
  else()
    set(_want "${expect}")
  endif()
  string(JSON _dec ERROR_VARIABLE _jerr GET "${_out}" decision)
  if(NOT "${_jerr}" STREQUAL "NOTFOUND")
    string(APPEND _bad " stdout is not a decision object: '${_out}'")
  elseif(NOT "${_dec}" STREQUAL "${_want}")
    string(APPEND _bad " decision '${_dec}', expected '${_want}'")
  endif()
  if(expect STREQUAL "deny")
    string(FIND "${_out}" "${_needle}" _pos)
    if(_pos EQUAL -1)
      string(APPEND _bad " reason lacks '${_needle}'")
    endif()
  endif()
  if(expect STREQUAL "open")
    string(FIND "${_err}" "${_needle}" _pos)
    if(_pos EQUAL -1)
      string(APPEND _bad " stderr lacks fail-open note '${_needle}': '${_err}'")
    endif()
  elseif(NOT "${_err}" STREQUAL "")
    string(APPEND _bad " unexpected stderr '${_err}'")
  endif()
  if(NOT _bad STREQUAL "")
    set(_failures "${_failures}\n  ${name}:${_bad}" PARENT_SCOPE)
  endif()
endfunction()

# claude_check(<name> <allow|deny|open> [<needle>]) — checks a Claude hook result.
#   allow: exit 0, no stdout, empty stderr
#   deny:  exit 2, no stdout, one stderr line containing <needle>
#   open:  fail-open: exit 0, no stdout, stderr notes it with <needle>
function(claude_check name expect)
  set(_bad "")
  set(_needle "${ARGV2}")
  set(_out "${${name}_out}")
  set(_err "${${name}_err}")
  if(expect STREQUAL "deny")
    set(_want_rc 2)
  else()
    set(_want_rc 0)
  endif()
  if(NOT ${name}_rc EQUAL _want_rc)
    string(APPEND _bad " exit=${${name}_rc}, expected ${_want_rc}")
  endif()
  if(NOT "${_out}" STREQUAL "")
    string(APPEND _bad " unexpected stdout '${_out}'")
  endif()
  if(expect STREQUAL "allow")
    if(NOT "${_err}" STREQUAL "")
      string(APPEND _bad " unexpected stderr '${_err}'")
    endif()
  else()
    string(FIND "${_err}" "${_needle}" _pos)
    if(_pos EQUAL -1)
      string(APPEND _bad " stderr lacks '${_needle}': '${_err}'")
    endif()
    string(FIND "${_err}" "\n" _nl)
    if(NOT _nl EQUAL -1)
      string(APPEND _bad " stderr is more than one line")
    endif()
  endif()
  if(NOT _bad STREQUAL "")
    set(_failures "${_failures}\n  ${name}:${_bad}" PARENT_SCOPE)
  endif()
endfunction()

set(_agy_hook "${REPO_ROOT}/tools/agy/pretool-guard.sh")
set(_claude_hook "${REPO_ROOT}/tools/claude/pretool-guard.sh")

# --- agy: verdict JSON on stdout, only run_command is checked -----------------
agy_payload(_p view_file "git push" "")
hook_run(agy_allow_non_shell "${_agy_hook}" "${_p}")
agy_check(agy_allow_non_shell allow)

agy_payload(_p run_command "echo ok" "")
hook_run(agy_allow_echo "${_agy_hook}" "${_p}")
agy_check(agy_allow_echo allow)

agy_payload(_p run_command "git push" "")
hook_run(agy_deny_push "${_agy_hook}" "${_p}")
agy_check(agy_deny_push deny "guard: denied: git push")

# lastUserInput echoes the prompt; a push the user only mentioned must not deny.
agy_payload(_p run_command "echo ok" "run git push now")
hook_run(agy_allow_prompt_mentions_push "${_agy_hook}" "${_p}")
agy_check(agy_allow_prompt_mentions_push allow)

# --- agy: fail open on a payload the hook cannot read -------------------------
hook_run(agy_open_garbage "${_agy_hook}" "not json {")
agy_check(agy_open_garbage open "payload is not JSON")

hook_run(agy_open_array "${_agy_hook}" "[1,2]")
agy_check(agy_open_array open "payload is not JSON")

hook_run(agy_open_no_command "${_agy_hook}" [==[{"toolCall":{"name":"run_command","args":{}}}]==])
agy_check(agy_open_no_command open "no CommandLine")

# --- Claude: exit codes, reason on stderr --------------------------------------
hook_run(claude_allow_status "${_claude_hook}" [==[{"tool_name":"Bash","tool_input":{"command":"git status"},"cwd":"@REPO@"}]==])
claude_check(claude_allow_status allow)

hook_run(claude_deny_push "${_claude_hook}" [==[{"tool_name":"Bash","tool_input":{"command":"git push"},"cwd":"@REPO@"}]==])
claude_check(claude_deny_push deny "guard: denied: git push")

hook_run(claude_allow_non_bash "${_claude_hook}" [==[{"tool_name":"Edit","tool_input":{"command":"git push"},"cwd":"@REPO@"}]==])
claude_check(claude_allow_non_bash allow)

# The payload cwd is the base for relative rm targets: ../build/tmp is inside
# the repository from native/ and outside it from the repository root.
hook_run(claude_allow_cwd_native "${_claude_hook}" [==[{"tool_name":"Bash","tool_input":{"command":"rm -rf ../build/tmp"},"cwd":"@REPO@/native"}]==])
claude_check(claude_allow_cwd_native allow)

hook_run(claude_deny_cwd_root "${_claude_hook}" [==[{"tool_name":"Bash","tool_input":{"command":"rm -rf ../build/tmp"},"cwd":"@REPO@"}]==])
claude_check(claude_deny_cwd_root deny "rm target is outside the repository")

hook_run(claude_open_garbage "${_claude_hook}" "not json {")
claude_check(claude_open_garbage open "payload is not JSON")

hook_run(claude_open_no_command "${_claude_hook}" [==[{"tool_name":"Bash","tool_input":{}}]==])
claude_check(claude_open_no_command open "no command string")

# --- .agents/hooks.json: valid, and the command as written runs from .agents/ --
file(READ "${REPO_ROOT}/.agents/hooks.json" _hooks)
string(JSON _hook_cmd ERROR_VARIABLE _jerr GET "${_hooks}" guard PreToolUse 0 hooks 0 command)
if(NOT "${_jerr}" STREQUAL "NOTFOUND")
  set(_failures "${_failures}\n  hooks_json: ${_jerr}")
else()
  string(REGEX MATCH "^bash ([^ ]+)$" _hook_match "${_hook_cmd}")
  if(NOT _hook_match)
    set(_failures "${_failures}\n  hooks_json: command is not 'bash <path>': '${_hook_cmd}'")
  else()
    get_filename_component(_hook_script "${REPO_ROOT}/.agents/${CMAKE_MATCH_1}" ABSOLUTE)
    if(NOT EXISTS "${_hook_script}")
      set(_failures "${_failures}\n  hooks_json: '${CMAKE_MATCH_1}' does not name a file relative to .agents/")
    endif()
    agy_payload(_p run_command "git push" "")
    string(REPLACE "@REPO@" "${REPO_ROOT}" _p "${_p}")
    file(WRITE "${SCRATCH_DIR}/hooks_json.in" "${_p}")
    execute_process(COMMAND bash -c "${_hook_cmd}"
      WORKING_DIRECTORY "${REPO_ROOT}/.agents"
      INPUT_FILE "${SCRATCH_DIR}/hooks_json.in"
      RESULT_VARIABLE hooks_json_rc OUTPUT_VARIABLE hooks_json_out ERROR_VARIABLE hooks_json_err)
    string(STRIP "${hooks_json_out}" hooks_json_out)
    string(STRIP "${hooks_json_err}" hooks_json_err)
    agy_check(hooks_json deny "guard: denied: git push")
  endif()
endif()

if(NOT _failures STREQUAL "")
  message(FATAL_ERROR "tooling_guard contract failed:${_failures}")
endif()
message(STATUS "tooling_guard contract passed")
