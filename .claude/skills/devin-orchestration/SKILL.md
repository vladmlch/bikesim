---
name: devin-orchestration
description: Delegate task execution to the local Devin CLI agent while you orchestrate. Use when dispatching implementation or other execution work to an agent, or when the user mentions devin.
---

# Devin orchestration

Devin is the default executor for delegated tasks; you orchestrate. Say so explicitly when you hand a task to your own subagent instead.

`S=.claude/skills/devin-orchestration/scripts`, `T=.claude/skills/devin-orchestration/templates`. Run every command from the main checkout root, so the relative paths (and the permission rule on `devin-run.sh`) match.

## Steps

1. **Brief.** Fill a template into `.scratch/devin/<YYYY-MM-DD>-<slug>/brief.md` with `python3 -I $S/devin-brief.py $T/<template>.md <run-dir>/brief.md KEY=VALUE ...` (see Briefs). Done when the script prints `wrote …`: it refuses a missing or unknown field.
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

Every byte you write into a brief or read back from Devin stays in your context for the rest of the session. Spend it only on what Devin cannot read itself and on the verdicts you act on.

- Devin's raw output never enters your context: read only the three sources in step 5. `export.<N>.json` is read only through `devin-summary.py`. Background launches print one status line; keep it that way.
- A reviewer's or implementer's final message carries the verdicts; open its report or review file only for the finding you are about to act on, and then only that section.
- When you review a diff yourself, start it at the first commit Devin authored, not at a base that re-shows your own code; check code the plan dictated verbatim mechanically (compare the file with the plan's code block) instead of reading it.

## Briefs

Write each brief once, as the delta only. A brief you wrote this session is already in your context, and `devin-brief.py` checks the fill, so leave it unread. Read it back — only the section you need — when its text is not in your context or may be wrong:

- after compaction or in a new session (the ledger keeps only the brief's path);
- when Devin returns `NEEDS_CONTEXT` / `BLOCKED`, or its report contradicts what you meant to ask: decide whether Devin or the brief is wrong;
- when you did not write it this session.

The brief's main readers are the Devin runs: the implementer checks it against the repository before working and restates goal and done-criterion in its report (`task.md`), and reviewers get its path and report brief defects separately (`review.md`). A defect in a brief surfaces in those short reports, not in a re-read of yours.

- **By path, not by paste.** Devin reads files itself. Requirements that already exist as a file (a plan task extracted by `task-brief`, a review package, an SDD prompt template, the plan, a spec) go into the brief as a path. The brief carries only what exists nowhere on disk: the goal, the done-criterion, decisions and rulings, the report path.
- **Fixed text lives in the templates.** Rules, the no-push line, the report contract are in `$T/*.md`; fill placeholders, never retype them. `task.md` serves implementers and fixers; `review.md` serves task reviews, re-reviews and any other read-only verdict run, with `INSTRUCTIONS_FILE` pointing at the prompt template it follows.
- **Fill fresh, never derive.** Build every brief from its template. Do not `sed` a previous brief into a new one: the copy carries values you never looked at.
- **Long values from files.** `KEY=@path` takes a value from a file you already have (a findings list, a constraints block copied from the plan), so it is not typed twice.
- English only (Devin's Russian drifts into anglicism-heavy mush). Devin already loads `AGENTS.md` and `CLAUDE.md` as always-on rules, so the brief never restates them. One concern per brief; split broader work into sequential briefs.

## With subagent-driven-development

Devin fills every SDD seat except the final review: implementers, task reviewers, scoped re-reviews and the final fix wave are Devin runs (`task.md` / `review.md`, with `INSTRUCTIONS_FILE` pointing at the matching SDD prompt template). Devin print mode cannot ask questions, so an implementer's questions arrive as `NEEDS_CONTEXT` and you answer through a follow-up.

**The final whole-branch review is yours.** Do it yourself, in this session: no Devin run, no subagent. Read the review package under the Context-budget rules (skip your own commits, check plan-verbatim code mechanically), triage the ledger's deferred minors, and record the verdict in the ledger. Findings go to one Devin fix run; check its fix diff yourself.

## Run directory layout

Per attempt `N`: `stdout.N.txt` (all agent narration; the final message is at the end), `stderr.N.txt`, `export.N.json` (ATIF transcript, written only when the attempt ends), `exit_code.N`. Shared: `brief.md`, `followup-N.md`, `attempt` (latest N), `session_id`, `pid`.

## Gotchas

- Print mode cannot show Devin's workspace-trust prompt and fails in an untrusted directory: a fresh worktree may need trusting once by the user (interactive `devin` there).
- Live progress lives in `~/.local/share/devin/cli/sessions.db` (`message_nodes` per `session_id`); the watchdog reads it read-only. The file is tens of GB: query it by `session_id` only.
- `devin-run.sh` is the script running while Devin works: a Devin task that changes it must replace it atomically (write a sibling file, then `mv`), because bash reads a running script from its open file.
- After changing a script, run its tests: `python3 -I -m unittest discover -s .claude/skills/devin-orchestration/tests -v`.
