# Task: {{TITLE}}

Workdir: {{WORKDIR}}
Goal: {{GOAL}}
Context: {{CONTEXT}}
Done when: {{DONE_WHEN}}

Before you change anything, check this brief against the repository: the files, symbols and commands it names exist, and the done-criterion can be checked. If the brief contradicts the repository or itself, stop with status NEEDS_CONTEXT and name the contradiction.

Rules:
- Keep all work on this machine: no git push, no PRs, no messages to external services.
- Work without asking questions. If blocked, stop and report status BLOCKED or NEEDS_CONTEXT with the specifics and the options you see.
- Do all the work yourself: no subagents, and no reviewer — review is scheduled after you report.
- Stage only the files you changed, with explicit paths; never `git add -A` or `git add .`.

Write the full report to {{REPORT_FILE}}: what you changed (files), the commands you ran with their relevant output (for TDD: RED then GREEN), self-review findings, concerns.

Final message (under 15 lines; the detail lives in the report file):
- Status: DONE | DONE_WITH_CONCERNS | BLOCKED | NEEDS_CONTEXT
- Understood: the goal and the done-criterion in one line, in your own words
- Commits created (short SHA + subject)
- One-line test summary
- Concerns, if any
- The report file path
