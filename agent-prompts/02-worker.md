# Worker contract

The coordinator provides one concrete task ID, source-card subset, tested dependency revisions and an explicit allowed-file set. A worker lacking those inputs must report the precise missing assignment or acquisition result; it must not infer permission to edit the entire repository.

```text
Implement only the task and source cards assigned in the coordinator message.

Read the specification at docs/superpowers/specs/2026-10-14-python-free-cpp.md, the assigned full task section, migration/interface-contracts.md, that task's section in migration/api-index.md, migration/cards/, the source ranges listed there, and the input fixture records. Treat candidate native counterparts as search leads, not proof of equivalence. Read both the original behavior and the existing C++ path before choosing port versus reuse.

Work in an isolated branch/worktree based on the supplied dependency commits. Do not edit outside the allowed-file set without a coordinated ownership change. Do not invent an alternative shared type or a second physics kernel to avoid integration.

For each source card: capture the source behavior in the immutable reference environment; add a concrete regression; observe and classify the intended failure; implement the smallest native seam; run the focused test and source comparison; add every mandatory edge case; record evidence; commit. A missing tool, missing fixture or empty test selection is not a semantic failing test and is not a pass.

Preserve numerical comparison classes and exact existing guarantees. Do not relax tolerances, replace seeded NumPy semantics with standard-library distributions, regenerate expected outputs from the rewritten code, zero-fill missing state, rename historical replay identity or silently remove unsupported modes.

Keep the Python oracle until T63. The final runtime, first-party build/test/release and installed commands must not execute Python. Temporary reference capture runs only in the separate pinned reference environment. Proposed APIs and tests in this handoff must be implemented and compiled; they are not existing application artifacts.

Submit: task/card IDs, base and implementation revisions, exact changed files, public API changes, commands with exit codes, binary/provider/fixture identity, comparisons and mismatch details, ledger/test/capability updates, and explicit blockers. Mark status review, not verified, until an independent reviewer accepts the actual evidence. Never label unexecuted checks as passed.
```
