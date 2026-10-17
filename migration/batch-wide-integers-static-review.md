# T51: arbitrary-width batch integers — static follow-up

This follow-up was reviewed and edited from source only. No compiler, configure command, analyzer, test executable, Python reference, simulation, worker, or CLI command was run. The authored regressions below are pending execution; this document does not close T51's execution gates.

## Source contract and diagnosed gap

`src/bike_sim/cli/batch.py::_grid_values` uses Python integers in both `range(start, stop, step)` and the `{start, count}` comprehension. `expand_spec` preserves table insertion order for the Cartesian product, flags, job names and explicit jobs. Its TOML input comes from `tomllib`, so these values can exceed both signed 64-bit integers and binary64's exact-integer range.

The native batch planner previously narrowed range operands through a machine integer or binary64. Additionally, the existing `terrain::parse_track_toml` adapter preserves wide integers only when the immediate key is `seed`. It did not preserve wide `start`, `stop`, `step`, `count`, or integer array entries. Fixing arithmetic alone would therefore leave the CLI rejecting or narrowing an input before planning it.

These findings are source-supported by the Python range/comprehension, native parser conversion of TOML integer nodes to `int64`, and the native batch caller. They have not been reproduced by executing either implementation.

## Implementation

`native/src/batch/integer_detail.hpp` supplies normalized signed decimal comparison, sign-aware addition/subtraction, and conversion of valid TOML integer spellings. All identities and range endpoints remain owned decimal strings. Decimal carry/borrow and radix conversion operate on bounded individual digits; no complete integer passes through `int64`, `uint64`, or binary64. Valid hexadecimal, octal, binary and underscored decimal tokens become canonical decimal values. Invalid signs, prefixes, underscores and decimal leading zeroes are left for the authoritative parser to reject.

`native/src/batch/plan.cpp` now uses that arithmetic for stop-exclusive ranges and integer `{start, count}` values. A zero step rejects; a step pointing away from the stop produces no values. Count is first compared in its arbitrary-width representation against zero and the existing one-million-job bound. Only a positive count already proved within the bound converts to `size_t`. A nonpositive count produces an empty range without evaluating the start, matching the source comprehension. Floating starts retain `start + double(i)` per element, rather than cumulative addition. The overall Cartesian-product and explicit-job bounds remain in place. Integer/float comparisons used for slugs retain exact represented-float comparison semantics.

`native/src/batch/spec.cpp` defines `batch::parse_spec_toml`. A lexical pass identifies integer **value** tokens in documents, arrays and inline tables. It skips basic/literal strings, multiline strings, escaped characters, comments, table headers and keys, including numeric keys. Correct integer tokens become unique quoted string markers, then the existing ordered TOML parser handles the rewritten document. Boolean, float and date tokens are unchanged; the parser still owns structural and noninteger lexical validation.

The adapter restores markers only in parsed value leaves, never object keys. It checks that every expected marker occurs exactly once. The chosen marker prefix is absent from the original text. Since a quoted Unicode escape can decode to a prefix absent from its raw spelling, the restoration pass also detects cooked-string collisions and reparses with another prefix before changing any value. The walk over parsed values is iterative. Source order is preserved because replacements do not move keys or change their relative positions. Comments after table headers are skipped independently of quotes inside the comment.

The public declaration is in `native/src/batch/plan.hpp`. The integrator has connected `cli/batch.cpp` to `parse_spec_toml`, added `spec.cpp` to `bike_batch`, and linked that target with the existing ordered-parser owner `bike_terrain` in `native/cmake/RecordingApplication.cmake`. This follow-up does not modify the track parser's own seed-only behavior.

## Authored regressions — not run

`native/tests/migration/test_t51.cpp::wide_toml_and_ranges` contains positive and negative cases for:

- A wide seed count range crossed with an ordinary floating grid, preserving flag and Cartesian order.
- An arbitrary-width negative step crossing zero, a negative wide start, and a decimal carry beyond 128-bit width.
- Wide hexadecimal, octal, binary and underscored decimal values inside arrays.
- Ordered `[[job]]` entries with integers wider than 128 bits and one default job for an empty spec.
- Numeric document/inline-table keys, quoted numeric strings, multiline strings, comments after headers, comments inside arrays, nested array/table transitions, empty nested arrays, boolean/float/date types, and Unicode-escaped marker collisions.
- Floating start/count behavior, a wide negative count producing an empty range, zero-step rejection and a positive count beyond the job limit.
- Invalid integer forms, missing array separators and unterminated strings remaining parser errors.

The previously authored serial/parallel worker, cancellation, exact numeric slug, argv, output-collision and seed-sweep regressions remain in the same source and were also not run. Compiler/header qualification, behavior of the pinned TOML implementation, and full T51 execution still require the future authorized verification pass.

## Remaining resource limits

The existing 512 MiB document limit and one-million-job limit remain explicit. Each append checks the rewritten TOML's size before growing it, and the result also passes through the parser's document limit. A near-limit input containing many very short integers can exceed that limit after marker expansion and reject. Arbitrary width means absence of machine-integer narrowing; it does not promise unbounded memory or input size.
