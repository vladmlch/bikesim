# Harness improvements for agy work in mujoco_clean (2026-11-23)

Outcome of a grilling session on 2026-11-23 that started from an AI report
on the user's Antigravity (agy) sessions of 2026-11-15..22. Many of that
report's claims were checked and found wrong or overstated; this plan acts
only on verified causes. Revision 2 folds in the agy plan review
(`.scratch/agy/2026-11-23-harness-plan-review/review.md`, verdict REWORK:
C1-C4, I1-I6) and the user's rulings on it.

## Goals and principles

Goals, in priority order:

- (a) stop wasting gate time on failures a cheap check would have caught;
- (b) less manual babysitting of agents (relaunches, permission prompts);
- (d) stop losing delegated output (truncated reports, artefacts left in a
  container).

Not a goal: enforcing physics invariants. Guards below touch them only
incidentally (golden recording).

Principles:

- **One source of truth for every executor.** Rules stay in `AGENTS.md`
  and `docs/agents/`; bans and checks are executable scripts in `tools/`.
  Each harness (agy, Claude, ...) gets a thin adapter that calls the same
  script. No agy-only `.agents/rules/` split: `AGENTS.md` is 13.8 KB,
  below agy's 24,000-byte per-file cap.
- **Design for the user driving agy interactively** (UI or CLI);
  headless `agy-run.sh` runs inherit the same scripts.
- **Orchestration skills are project-scoped.** Their single source is
  `~/Desktop/Projects_my/llm-orchestrators/skills/`; they are linked into
  each project that uses them, never into `~/*` (user ruling 2026-11-23).

## Verified baseline (2026-11-15..22)

Gates (`<git-common-dir>/bike-gate/timings.tsv`): 18 runs, 26,273 s
(7.3 h). full: 7 passed, 4 failed; native: 7 passed.

| Gate | Failed at | Root cause |
|---|---|---|
| efb8e415 full | cppcheck, 124 s | level 1 run as `check_change.sh HEAD`: only uncommitted files checked; the committed offender skipped |
| 03826ff3 full | cppcheck, 85 s | level 1 not run at all (Devin SDD task 9) |
| 771cbbb2 full | odr, 561 s | a GCC-only `-Wreturn-type` (no return after a switch covering every enum value; fix e4bb0e6d) that clang and level 1 do not report; only the whole-image GCC odr sweep compiled it |
| 5e1af40a full | slow, 1943 s | real test failure (not in scope) |

Also: native re-run on a SHA where full had already passed (26b1691c,
1001 s); five per-task native runs on physics-2026-11-05 against "once per
merge". `check_change.sh` writes nothing to `timings.tsv`, so the journal
cannot show whether level 1 ran.

`check_change.sh` already defaults to `merge-base(HEAD, main)`. The `HEAD`
base came from `AGENTS.md:52` ("pass the commit before the stage as
`base`"), which agents read as `HEAD`.

agy relaunches (agy CLI logs `~/.gemini/antigravity-cli/log/cli-*.log`):
mostly a plain `agy` launch (conversation picker, auto preset with sandbox)
followed by a relaunch with `--dangerously-skip-permissions`; then quota
429 (`Individual quota reached ... Resets in 1h48m26s`, a relative Go
duration in the CLI log/stderr; the quota is shared across all models, so
switching models does not help); then agy 1.5.10 crashes; then closed tmux
panes. No log shows the sandbox blocking `~/.config/mise`,
`~/.cache/bike-deps` or `/Volumes/SSD_1TB_M2`.

Lost output: `.work/implicit-fsm-audit-2026-11-17/reports/discovery/D14.md`
cut at line 485 mid-word by a discovery subagent, patched by the
coordinator; the container agy session kept `brain/` in `/root/.gemini`
and was copied out with `container cp`.

Config rot: 19 broken skill symlinks in the project (9 in `.agents/skills`,
10 in `.claude/skills` incl. `devin-orchestration-v2`) point at the removed
`llm-orchestrators/{claude,devin,shared}/skills`;
`.claude/skills/devin-orchestration` is a stale tracked copy; `.mcp.json`
`bike_kinematics` runs the removed `tools.mcp_server`.

### Success metrics (re-measure on 2026-11-30)

1. Share of native/full gates failing in a stage that level 1 covers.
2. Count of manual agy relaunches, by cause.
3. Count of rule violations caught by review instead of a script.

## Execution model

- Branch `harness-agy-2026-11-23` in the main checkout (its
  `build/native-release` is built; no other worktree near `main` is clean).
  llm-orchestrators work happens in that repo on its current branch.
- llm-orchestrators has uncommitted user edits (README.md, install.toml,
  nvidia/qoder SKILL.md and run script). A task there commits only files it
  alone changed; a file that already had user edits is edited in place,
  left uncommitted and named in the task report.
- Implementation by Haiku 5.5 (effort high) subagents, sequential, one at a
  time. Each brief is self-contained (paths, current code excerpts,
  acceptance) and keeps the subagent under ~100k tokens of context. No cap
  on files or lines. A failed task is split further, not retried wider.
- Inside a task only targeted tests (`mise run test '<pattern>'`, one
  standalone sweep). The coordinator runs level 1 at the end of each stage;
  never two gates at once.
- Review: agy `gemini-4-carbon-medium`, read-only, at review points R1..R4,
  not on every task. On quota 429 the coordinator reviews and agy
  re-reviews later.
- Test budget: level 1 per stage (~5 min x 5); the tooling CTest tests
  touched by stage 3 including their `slow`-labelled ones
  (`mise run test 'tooling'`, ~few min); one
  `bash tools/run_tests.sh native` before merge (~11 min). No `full`: no
  physics or realtime path changes. Total ~45 min.

## Stage 1 — AGENTS.md commit (done)

`561cbb04` on `main`: the user's removal of the Behavioral Stability
section plus deletion of `.devin/hooks.v1.json`. `AGENTS copy.md` left
untouched.

## Interlude — equilibrium fingerprint fix (branch fix/equilibrium-fingerprint-paths)

Level 1 after stage 2 failed in `reference.equilibrium_state_file`: the pose
fingerprint hashed absolute joint-file paths, so the pinned
`savage_pneumatic.npz` was valid only in the worktree that recorded it.
657f21a0 keys those files by content sha256 (non-legacy branch only) with a
regression test; ca65fc62 re-records the pose (user-approved); f5925f1d
fixes the re-record hint. Level 1 green (198/198, 114 s). This branch is
rebased onto it; both merge into `main` together after one native gate.

## Stage 2 — hygiene and project-scoped skills

2.1 **Drop irrelevant tracked skills.** `git rm` from both `.agents/skills`
and `.claude/skills`: react-best-practices, vercel-react-best-practices,
composition-patterns, react-view-transitions, web-design-guidelines,
playwright-skill, python-patterns, python-testing-patterns,
python-type-safety, async-python-patterns, the `.lazyskills-disabled/`
tree, stray `grilling.zip`, and the stale tracked copy
`.claude/skills/devin-orchestration`. Before removing, `git grep` each name
outside the skill dirs; a hit is reported, not deleted around.

2.2 **Project mode for `install.sh`** (llm-orchestrators,
`lib/install.py` + `install.toml`). `install.sh --project <dir> [skill...]`
links each skill into `<dir>/.claude/skills/<skill>` (agent `claude`) and
`<dir>/.agents/skills/<skill>` (agents `agy` and `devin`; Devin CLI reads
the project's `.agents/skills`) per the manifest's `agents` list; codex keeps
its one user-level skill (its codex-specific SKILL.md cannot share
`.agents/skills`), and `--uninstall` never removes a skill without a project
location, nor `[links]` or hooks; with absolute symlinks; replaces a broken link; refuses to overwrite
a real directory or a link it does not own; writes each link path into
`<git-common-dir>/info/exclude` (creating the file when absent). The
user-level mode stops installing skills into `~/*`; `--uninstall` of the
user level removes the existing user-level skill links it owns.
`--check --project <dir>` lists broken links under both dirs and exits
non-zero. Extend the existing tests under `test/`.

2.3 **Project-relative skill paths.** Every orchestration `SKILL.md`
(22 hardcoded `~/.claude/skills/...` occurrences) uses
`.claude/skills/<skill>/...`, run from the project root, which the
project's `.claude/settings.json` allow-rules already match; add the
missing allow-rules there for cld-igx-orchestration and
nvidia-orchestration. agy reads the same skill dirs through
`.agents/skills/<skill>`; an agy-side script path is `.agents/skills/...`
where the skill's agy instructions name one.

2.4 **Relink mujoco_clean.** Remove the 19 broken links; run
`install.sh --project <mujoco_clean>` for the orchestration skills in
`install.toml`; then remove the user-level links (`install.sh --uninstall`
user level). Verify: `install.sh --check --project`, and one dry
`agy-run.sh --help`-level call through the new relative path. Other
projects lose these skills until linked; accepted by the user.

2.5 **Dead MCP server.** Remove `bike_kinematics` from `.mcp.json`.

2.6 **AGENTS.md size warning.** `check_change.sh` prints a warning when
`AGENTS.md` exceeds 18,000 bytes (agy truncates a rule file at 24,000
bytes silently and demotes always-on rules past a 20,000-token
aggregate). Not in the contracts kind: it has only pass/fail semantics and
its fixture is `slow`-labelled.

Level 1. **R1**.

## Stage 3 — gates

3.1 **Tree id helper.** `tools/lib/tree_id.sh`: prints a content id of the
working tree without writing git objects: sha256 over `git rev-parse HEAD`,
`git diff HEAD --binary`, and the sorted list of untracked non-ignored
files with size and mtime (contents not hashed: `output/` holds ~950 MB).
Prints `nogit` unless `git rev-parse --show-toplevel` from the script's
repo root equals that root (a scratch repo under `build/` must not walk up
to the outer checkout's `.git`; R1 I1).

3.2 **`check_change.sh` base.** Default stays `merge-base(HEAD, main)`. An
explicitly passed base that resolves to the `HEAD` commit is refused when
`HEAD` has commits not in `main` (the efb8e415 case), with a message naming
the default and `--uncommitted` (check only uncommitted + untracked files).
On `main` itself, or with the default base, nothing is refused (R1 I2:
`mise.toml` passes `main`; `change_docs_only` runs with `HEAD == main`).
Also task 2.6 (AGENTS.md size warning) lands here, in the same file. Update
`AGENTS.md:52` and the header comment: omit the base unless the branch is
long-lived; never pass `HEAD`.

3.3 **Level 1 leaves a journal row.** `check_change.sh` resolves
`BIKE_GATE_STATE_DIR` the same way `run_tests.sh` does, and on exit (an
`EXIT` trap, so failures are recorded too) appends a row with the same 10
columns as `run_tests.sh` (profile `change`, preset `release`, from `-`,
jobs, status, seconds, summary) plus an 11th column `tree=<id>`;
`run_tests.sh` appends the same 11th column. Before changing the format,
check every reader of `timings.tsv` (only the tooling tests and
docs/TESTING.md read it; checks_internal.hpp and mise.toml only mention it)
and keep them working. Only complete runs (from `-`) satisfy a
precondition; `--uncommitted` journals from `uncommitted` (R2 I1).

3.4 **Gate preconditions.** `run_tests.sh native|full` (release preset,
no `--from`) refuses to start unless the journal has a passed `change` row
for the current tree id; `native` also refuses when a passed `full` row
exists for it. Skipped when the tree id is `nogit` (offline bundle,
container `/src`). Override: `BIKE_GATE_OVERRIDE='<reason>'`, written into
the run's journal row. `quick` and `--from` are unaffected. Update
`native/tests/tooling/tooling_gate.cmake`: copy `tools/lib/tree_id.sh` into
the scratch repo and keep the existing gate cases green (scratch repos
without `.git` take the `nogit` path; cases with git seed a `change` row or
set the override). New cases: refusal without L1, pass with L1, refusal
after full, override logged. `mise run full`/`native` tasks inherit the
check through `run_tests.sh`.

3.5 **GCC warnings in level 1 (one diagnosis task).** Find what compiler
and flags the `frontends` kind uses on changed TUs
(`native/src/tools/check_frontends.cpp`) and whether a GCC run on those
TUs can report `-Wreturn-type` (and similar flow warnings) in seconds,
e.g. `-c -O0 -o /dev/null` instead of syntax-only. Reproduce on the
pre-fix sources of e4bb0e6d. If it works within seconds per TU, add it to
the kind used by level 1, with a test. If not, record why here and leave
the sweep order unchanged (odr stays where it is: before tidy).
Outcome: the diagnostic was `-Winvalid-noreturn` ('noreturn' function does
return), which `-fsyntax-only` cannot emit. b55800ed + 44979530 + 7d36b82f:
the GCC frontend compiles (`-c -O0`) only under `--only-sources` (level 1,
~1.2 s/TU); the full sweep stays `-fsyntax-only` (88 s vs 236 s; user
ruling). `-fanalyzer` was a no-op under `-fsyntax-only` and is dropped;
the -O0 stack budgets were never effective and are queued. f13041d8 keeps
`native/src/tools/` out of the source identity so tooling edits do not
stale the pinned pose.

3.6 **Docs.** `AGENTS.md` Verification: level 1 is a hard precondition of
level 2/3, the override variable, the journal column, `--uncommitted`.

Level 1, then `mise run test 'tooling'`. **R2**.

## Stage 4 — guard, hook, interactive wrapper

4.1 **`tools/guard.sh`.** Reads one shell command (argument or stdin) and
exits 0 (allow) or 2 with a reason on stderr (deny). Denies: `git reset
--hard`, `git clean`, `git push` (any form), `git stash` (drops other
agents' work in a shared tree), `BIKE_GOLDEN_RECORD=1` (also the
misspelling `BIKE_RECORD_GOLDENS=1`, including `env`/`export` forms)
unless `BIKE_GUARD_ALLOW_GOLDENS=1` is set in the guard's environment, and
`rm -rf` of a path outside the repository (relative paths resolved against
the command's cwd passed by the caller). Matching is on tokens, not
substrings; tests for false positives (`git push` inside a quoted commit
message, `git log --grep reset`). No gate check here: 3.4 enforces that in
`run_tests.sh` itself.
As built (R3/R4 fixes): `git stash list|show` are allowed (read-only);
`rm -rf` is also allowed strictly inside `$TMPDIR`, `/tmp`, `/private/tmp`
and the physical targets of the repo's `.scratch`/`.work` links (the roots
and root-level wildcards stay denied); env wrappers (`env -S/-i`,
`cmake -E env`) and git global options are seen through; quoted text,
heredocs, `[[ ]]`, arithmetic and compound keywords are not commands. It
stays a backstop, not a sandbox (`sudo`, `bash -c`, `eval` not unwrapped).

4.2 **Capture a real agy hook payload first.** A throwaway agy session
(`gemini-4-hydrogen-low`) in a scratch git repo with a logging-only
`PreToolUse` hook; one allowed command and one `git push --dry-run`. Record
the tool name, args shape, `cwd`, `workspacePaths`, and whether the hook
fires under `--dangerously-skip-permissions`. Outcome (2026-11-23,
/tmp/agy-hook-probe): layout
`{"<name>":{"PreToolUse":[{"matcher":"*","hooks":[{"type":"command",...}]}]}}`;
shell tool `run_command`, command in `toolCall.args.CommandLine`, cwd in
`cwd`; `lastUserInput` echoes the prompt (never match on it); the hook
fires and `deny` blocks under `--dangerously-skip-permissions`, so `agym`
keeps skip-permissions.

4.3 **agy adapter.** `.agents/hooks.json` `PreToolUse` hook; the hook runs
with cwd `.agents/`, so its command is `bash ../tools/agy/pretool-guard.sh`
or an absolute path resolved from the payload's `workspacePaths`.
`pretool-guard.sh` extracts the command and its cwd from the payload shape
captured in 4.2, calls `guard.sh`, and prints
`{"decision":"deny","reason":...}` or `{"decision":"allow"}`. If 4.2 shows
the hook does not fire under skip-permissions, `agym` uses `--mode auto`
and the wrapper passes `allowNonWorkspaceAccess` through agy `--settings`
(not a CLI flag).

4.4 **`agym` wrapper.** `tools/agy/agym` (symlinked into
`~/.local/bin` — a launcher, not a skill): `agy
--dangerously-skip-permissions` plus `--add-dir` for the physical paths of
`.scratch` and `.deps` (`pwd -P`, skipped with a note when the SSD is not
mounted), and `--log-file <per-session file>`; passes through all user
args. On exit it reads that log: on 429 it prints the reset time (log line
time + the relative duration) and the exact `agym --conversation=<id>`
line to resume.

4.5 **Claude adapter.** `.claude/settings.json` `PreToolUse` hook on Bash:
`tools/claude/pretool-guard.sh` parses Claude's hook JSON
(`tool_input.command`, `cwd`) with `jq` and calls `guard.sh`; merged with
existing hooks, never replacing them.

Level 1. **R3**.

## Stage 5 — agy-orchestration skill (llm-orchestrators)

5.1 `SKILL.md`: model table adds `gemini-4-carbon-{low,medium,high}` and
`gemini-4-hydrogen-{low,medium,high,xhigh}` (ids carry the effort suffix,
checked with `agy models`); reviews default to `gemini-4-carbon-medium`.

5.2 `agy-run.sh` passes `--log-file <run-dir>/cli.<N>.log`. On a 429 (in
that log or stderr) it exits 4 and writes `quota_reset=<ISO time>`
(computed from the line time + relative duration) to the run dir; no retry,
no model switch. SKILL.md tells the orchestrator to schedule a wakeup at
that time. Replace the running script atomically (sibling file + `mv`);
run the skill's unit tests. As built (R4 I3): only a 429 of the agent's own
model call, after the attempt started and with a future reset, yields exit
4; background-monitor 429s do not; a stale `quota_reset` is removed at the
start of each attempt. Default model: `gemini-4-hydrogen-high` (user,
2026-11-23).

5.3 `agy-run.sh`: optional `--json-schema <file>` passthrough.

## Stage 6 — report completeness and container artefacts

6.1 **End-of-report marker.** `tools/agy/check-reports.sh <dir>
[--expect <n>]`: every `*.md` in `<dir>` must end with the line
`<!-- END REPORT shard=<id> items=<n> -->`; with `--expect`, the number of
report files must equal `<n>`; prints the failing or missing shards and
exits non-zero. Document the marker in agy-orchestration `SKILL.md` as the
required last line of any multi-shard report brief, with the validator as
the coordinator's step before merging shard results. Item counting inside
a report stays out (shapes differ per audit).

6.2 **Container `brain/` on the host.** The fedora container is an Apple
`container` instance created outside `tools/linux/container.sh` (disabled
by a plan-change guard). Add `tools/agy/container-recipe.md` with the
`container run` line mounting the repo at its host path and a host dir
(`/Volumes/SSD_1TB_M2/work/agy-container-gemini`, or
`~/.cache/agy-container-gemini` without the SSD) at `/root/.gemini`.
The physical targets of `.scratch` and of the project skill links
(llm-orchestrators, read-only) are mounted at their host paths too (R4 I4).
Recreating the existing `fedora` container is the user's call.

Level 1. **R4** (stages 5-6 and the whole branch diff).

## Merge

`bash tools/check_change.sh`, then one `bash tools/run_tests.sh native` on
the final tree (it must pass its own new precondition). Fast-forward
`main`; no push. Schedule the metric re-measure for 2026-11-30.

## Out of scope / user actions

- Rotate `GEMINI_GATEWAY_KEY` and the packet.ai bearer token found in
  plaintext in `~/.zsh_history` (2026-11-18, 2026-11-20) and remove those
  lines.
- agy 1.5.10 crashes: check whether 1.5.15 still shows them before acting.
- The 5e1af40a slow failure.
- Pinned pose (resolved 2026-11-23, user ruling "E"): any native/{src,cmake}
  edit made `savage_pneumatic.npz` stale, and dropping the source digest
  from its fingerprint is unsafe because a schema-1 state file restores
  seven mjModel arrays (tendon stiffness/damping, dof_frictionloss, ...)
  over the current model, so the post-load residual recheck would certify
  stale code-derived values. The test profile now relies on the auto-cache
  like `examples/research/viewer_physics_pneumatic.toml`; the fixture and
  its staleness test are gone, `test_pneumatic_settle_restore_round_trip`
  keeps the explicit-file path covered. Optional later physics task: a
  minimal start state (coordinates + physical memory; parameters from the
  current model, derived values recomputed) with a state-contract version
  instead of the source digest.
- Stack budgets (`-Wframe-larger-than`, `-Wstack-usage`) never ran: they
  need an optimized build, not the frontends sweep (dropped in 44979530).
