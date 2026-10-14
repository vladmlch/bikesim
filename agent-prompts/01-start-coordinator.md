# Initial coordinator prompt

Paste the following into an agent with access to the actual repository and this handoff:

```text
You are coordinating the complete removal of Python from bike-sim, preserving required application behavior and existing native work.

Read README.md, docs/superpowers/plans/2026-10-14-python-free-cpp-master.md, docs/superpowers/specs/2026-10-14-python-free-cpp.md, migration/interface-contracts.md, migration/task-graph.json and docs/superpowers/audits/2026-10-14-python-free-cpp-source-audit.md.

Execute T00 only, from docs/superpowers/plans/2026-10-14-python-free-cpp-00-foundation.md. Treat the actual checkout as authoritative; reconcile it against migration/source-manifest.json. Do not overwrite dirty user files, delete Python, regenerate goldens, implement new physics, invent missing hashes or claim any package is verified.

Resolve the actual Git revision, lockfiles and required omitted assets, preserve a reference tag/worktree without overwriting existing names, and update the source/removal/test inventories for checkout differences. Record missing artifacts with exact paths and affected tasks. Harmless wrapper/final-newline differences must be distinguished from semantic source changes.

Deliver the T00 diff, inventory reconciliation, exact commands and exit codes, current blockers, the reference identity, and a proposed T01 assignment. Do not proceed to other packages until the coordinator review accepts this first gate.

For later tasks, dispatch isolated workers using agent-prompts/02-worker.md and review with agent-prompts/03-reviewer.md. One integrator owns shared CMake/public interface merges. Work-package completion dependencies are in task-graph.json; source-card preparation may start only when its real producer interfaces and reference inputs exist. Never run multiple full native gates on the same machine concurrently.
```
