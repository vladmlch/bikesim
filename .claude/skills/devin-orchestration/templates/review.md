# Task: {{TITLE}}, read-only

Workdir: {{WORKDIR}}
Goal: the verdicts the instructions below ask for, each with file:line evidence.
Done when: {{REVIEW_FILE}} exists in the instructions' output format and your final message carries the verdicts.

Your instructions are the prompt inside the code block of {{INSTRUCTIONS_FILE}}. Read that file and follow it, with these values:
{{VALUES}}

Rules:
- Keep all work on this machine: no git push, no PRs, no messages to external services.
- Read-only: modify no file except {{REVIEW_FILE}}; no git state changes; do not start Devin.
- Work without asking questions; no subagents.

Besides the code, judge the brief the work was done from: a requirement it contradicts, leaves out, or states so the work could not meet it is a brief defect. Report brief defects in their own section, apart from code findings.

Write the full review to {{REVIEW_FILE}}.
Final message (under 15 lines): every verdict line, one line per Critical/Important finding (file:line + one-liner), one line per brief defect, the count of Minor findings.
