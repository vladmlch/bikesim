---
name: devin-orchestration
description: Delegate task execution to the local Devin CLI agent while you orchestrate. Use when dispatching implementation or other execution work to an agent, or when the user mentions devin.
---

# Devin orchestration

Devin is the default executor for delegated tasks; you orchestrate. Say so explicitly when you hand a task to your own subagent instead.

`S=.claude/skills/devin-orchestration/scripts`. Run every command from the main checkout root, so the relative paths (and the permission rule on `devin-run.sh`) match.

## Steps

1. **Run directory.** Create `.scratch/devin/<YYYY-MM-DD>-<slug>/` and write `brief.md` from the template below. Done when the brief names the goal, the workdir, and a checkable done-criterion.
2. **Launch.** Bash with `run_in_background: true`:
   `bash $S/devin-run.sh <run-dir> [--workdir <worktree>] [--model <id>]`.
   The model defaults to `swe-2-max` (free). A paid model (Fable, Opus, GPT-6, …) needs the user's approval first: ask when the task is clearly hard or `swe-2-max` failed it twice.
3. **Watchdog.** A second background Bash: `python3 -I $S/devin-watch.py <run-dir>`. It exits on completion or after 15 min with no new Devin message.
4. **On a stall report.** Silence alone is not a hang: a build inside one exec call adds no messages.
   - A descendant process is burning CPU (build, tests) → re-arm the watchdog and wait another 15 min.
   - No descendant, or all idle → `kill <pid>`, write a follow-up that says where it stopped, resume (step 6), and tell the user.
5. **On completion.** Read, in this order, and stop as soon as you know enough:
   1. the report: `tail -c 6000 <run-dir>/stdout.<N>.txt`;
   2. the digest: `python3 -I $S/devin-summary.py <run-dir>`;
   3. `git -C <workdir> diff --stat`, then only the files or hunks you need.
6. **Follow-up.** Write `followup-<N>.md` and run `bash $S/devin-run.sh <run-dir> --prompt <run-dir>/followup-<N>.md` (it resumes the recorded session with `-r`), then re-arm the watchdog. When Devin stopped on a question, answer it yourself if the facts settle it; when the decision is the user's, ask the user first.

## Context budget

Devin's raw output never enters your context. Read only the three sources in step 5. `export.<N>.json` is read only through `devin-summary.py`, never directly. Background launches print one status line; keep it that way.

## Brief template

Write briefs in English (Devin's Russian drifts into anglicism-heavy mush). Devin already loads `AGENTS.md` and `CLAUDE.md` as always-on rules, so the brief carries only the task. One concern per brief; split broader work into sequential briefs.

```markdown
# Task: <one line>

Workdir: <absolute path>
Goal: <what must be true when you are done>
Context: <files, symbols, prior findings, decisions already made>
Done when: <checkable criterion, e.g. the named test passes, the listed behaviour holds>

Rules:
- Keep all work on this machine: no git push, no PRs, no messages to external services.
- Work without asking questions. If blocked, stop and report the blocker and the options you see.

Final report (your last message, at most 40 lines, exactly these sections):
## Result      — done / partial / blocked, one sentence
## Changes     — files changed, one line each
## Commands    — commands run that matter, with pass/fail
## Open issues — anything unresolved or doubtful
```

## Run directory layout

Per attempt `N`: `stdout.N.txt` (all agent narration; the final message is at the end), `stderr.N.txt`, `export.N.json` (ATIF transcript, written only when the attempt ends), `exit_code.N`. Shared: `brief.md`, `followup-N.md`, `attempt` (latest N), `session_id`, `pid`.

## Gotchas

- Print mode cannot show Devin's workspace-trust prompt and fails in an untrusted directory: a fresh worktree may need trusting once by the user (interactive `devin` there).
- Live progress lives in `~/.local/share/devin/cli/sessions.db` (`message_nodes` per `session_id`); the watchdog reads it read-only. The file is tens of GB: query it by `session_id` only.
- After changing a script, run its tests: `python3 -I -m unittest discover -s .claude/skills/devin-orchestration/tests -v`.
