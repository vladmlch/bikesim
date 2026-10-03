# G5 source continuation

This archive preserves the original offline bundle and adds one completed G5
corrective subtask: bounded mechanical-mode search and guarded grip branches.

Start with `project/docs/superpowers/audits/2026-10-03-g5-mode-search.md` for the
changes, test results, exact source-run commands and remaining gates.
Current progress is at
`.superpowers/sdd/2026-10-03-v2-seated-plant/progress.md`.

The bundled dependency wheels and installer are unchanged. The `bike_sim`
wheel was NOT rebuilt, as wheel parity is excluded by the project plan.
Use `project/src` on `PYTHONPATH` when testing/running the updated source;
installing only the original bundled wheel does not include these changes.

No Git history was present in the input ZIP, and none has been manufactured.
Whole-G5 physical qualification and later plan phases remain open.
