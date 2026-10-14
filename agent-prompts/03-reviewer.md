# Independent reviewer prompt

```text
Review the assigned migration task against docs/superpowers/specs/2026-10-14-python-free-cpp.md, its full task section, source cards and actual diff. Do not approve from the worker's summary alone.

First review specification/behavior: read the original Python code and existing native implementation; trace all source fields, defaults, validators, exceptions and side effects; check that every mandatory case has a concrete test or explicit acquisition blocker. Confirm exact command names, recording/publication/replay semantics and supported/invalid mode behavior. Candidate native names are not equivalence evidence.

Then review native correctness: ownership and destruction on failure, model/data independence, bounds and integer conversions, resource limits, transaction/poison rules, plugin ABI/error containment, strict floating-point and operation-order requirements, isolated rendering, package linkage and absence of an interpreter fallback. Ensure produced signatures exactly match the shared contracts and consumers.

Inspect the test diffs and raw evidence. Confirm the intended binary really ran, tests were selected, parameterized assertions were not reduced to one example, reference fixtures predate the rewrite, hashes/provider identity match, and negative controls fail for the intended reason. A historical failing reference case cannot be relabeled valid. Re-run focused checks where the environment permits and distinguish independent verification from evidence inspection.

Reject silent capability removal, regenerated goldens, unapproved numeric/replay changes, fabricated hashes/results, generic exception swallowing, unsupported tests marked passing, and source suffix deletion without row-level evidence. Existing broad assertions cannot be replaced with weaker smoke tests.

Return a verdict per source card and per work package: accepted, changes required, or blocked. Cite exact files/lines and failed requirement/test IDs; list the minimal corrective changes. Approval closes only the reviewed scope and platform/provider, not the entire migration.
```
